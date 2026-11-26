import math
import unittest

GRAVITY_MPS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MotionEstimatorRef:
    """Reference verification model matching components/motion_estimator/motion_estimator.c."""

    def __init__(self):
        self.beta = 0.06
        self.alpha = 0.35
        self.accel_deadband_g = 0.045
        self.gyro_deadband_dps = 3.0
        self.zupt_hold = 16
        self.vel_decay = 0.985
        self.pos_return_decay = 0.94
        self.vel_dir_thresh = 0.03
        self.rot_dir_thresh = 15.0

        self.q = [1.0, 0.0, 0.0, 0.0]
        self.lin_acc = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.pos = [0.0, 0.0, 0.0]
        self.still_count = 0

    def update(self, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, dt):
        gx, gy, gz = gx_dps * DEG_TO_RAD, gy_dps * DEG_TO_RAD, gz_dps * DEG_TO_RAD
        q0, q1, q2, q3 = self.q

        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        acc_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g
        if acc_norm_sq > 1e-8:
            inv = 1.0 / math.sqrt(acc_norm_sq)
            ax, ay, az = ax_g * inv, ay_g * inv, az_g * inv
            s0 = 4 * q0 * q2 * q2 + 2 * q2 * ax + 4 * q0 * q1 * q1 - 2 * q1 * ay
            s1 = 4 * q1 * q3 * q3 - 2 * q3 * ax + 4 * q0 * q0 * q1 - 2 * q0 * ay - 4 * q1 + 8 * q1 * q1 * q1 + 8 * q1 * q2 * q2 + 4 * q1 * az
            s2 = 4 * q0 * q0 * q2 + 2 * q0 * ax + 4 * q2 * q3 * q3 - 2 * q3 * ay - 4 * q2 + 8 * q2 * q1 * q1 + 8 * q2 * q2 * q2 + 4 * q2 * az
            s3 = 4 * q1 * q1 * q3 - 2 * q1 * ax + 4 * q2 * q2 * q3 - 2 * q2 * ay
            s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3
            if s_norm_sq > 1e-12:
                inv_s = 1.0 / math.sqrt(s_norm_sq)
                qDot1 -= self.beta * s0 * inv_s
                qDot2 -= self.beta * s1 * inv_s
                qDot3 -= self.beta * s2 * inv_s
                qDot4 -= self.beta * s3 * inv_s

        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt
        qn = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = [q0 / qn, q1 / qn, q2 / qn, q3 / qn]
        q0, q1, q2, q3 = self.q

        awx = (1 - 2 * (q2 * q2 + q3 * q3)) * ax_g + 2 * (q1 * q2 - q0 * q3) * ay_g + 2 * (q1 * q3 + q0 * q2) * az_g
        awy = 2 * (q1 * q2 + q0 * q3) * ax_g + (1 - 2 * (q1 * q1 + q3 * q3)) * ay_g + 2 * (q2 * q3 - q0 * q1) * az_g
        awz = 2 * (q1 * q3 - q0 * q2) * ax_g + 2 * (q2 * q3 + q0 * q1) * ay_g + (1 - 2 * (q1 * q1 + q2 * q2)) * az_g

        lin_g = [awx, awy, awz - 1.0]
        for i in range(3):
            self.lin_acc[i] = self.alpha * self.lin_acc[i] + (1 - self.alpha) * (lin_g[i] * GRAVITY_MPS2)

        lin_mag_g = math.sqrt(sum(v * v for v in self.lin_acc)) / GRAVITY_MPS2
        gyro_mag = math.sqrt(gx_dps ** 2 + gy_dps ** 2 + gz_dps ** 2)

        if lin_mag_g < self.accel_deadband_g and gyro_mag < self.gyro_deadband_dps:
            self.still_count = min(self.still_count + 1, self.zupt_hold)
        else:
            self.still_count = 0

        stationary = self.still_count >= self.zupt_hold
        deadband_mps2 = self.accel_deadband_g * GRAVITY_MPS2

        if stationary:
            self.vel = [0.0, 0.0, 0.0]
            self.pos = [0.0 if abs(p * self.pos_return_decay) < 1e-4 else p * self.pos_return_decay for p in self.pos]
        else:
            for i in range(3):
                a = 0.0 if abs(self.lin_acc[i]) < deadband_mps2 * 0.7 else self.lin_acc[i]
                self.vel[i] = clamp(self.vel[i] * self.vel_decay + a * dt, -3.0, 3.0)
                self.pos[i] = clamp(self.pos[i] + self.vel[i] * dt, -0.5, 0.5)

        direction = "STATIC"
        if not stationary:
            abs_v = [abs(v) for v in self.vel]
            vi = abs_v.index(max(abs_v))
            if abs_v[vi] >= self.vel_dir_thresh:
                direction = ("+" if self.vel[vi] >= 0 else "-") + ["X", "Y", "Z"][vi]
            else:
                abs_a = [abs(a) for a in self.lin_acc]
                ai = abs_a.index(max(abs_a))
                if abs_a[ai] >= deadband_mps2:
                    direction = ("+" if self.lin_acc[ai] >= 0 else "-") + ["X", "Y", "Z"][ai]
                elif gyro_mag >= self.rot_dir_thresh:
                    direction = "ROT"

        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * RAD_TO_DEG
        pitch = math.asin(clamp(2 * (q0 * q2 - q3 * q1), -1.0, 1.0)) * RAD_TO_DEG
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * RAD_TO_DEG
        return {"roll": roll, "pitch": pitch, "yaw": yaw, "dir": direction, "stationary": stationary}


class TestMotionEstimator(unittest.TestCase):
    def test_stationary(self):
        est = MotionEstimatorRef()
        for _ in range(100):
            st = est.update(0, 0, 1.0, 0, 0, 0, 0.005)
        self.assertTrue(st["stationary"])
        self.assertEqual(st["dir"], "STATIC")
        self.assertAlmostEqual(st["roll"], 0.0, delta=0.5)

    def test_push_pos_x_and_zupt(self):
        est = MotionEstimatorRef()
        for _ in range(40):
            est.update(0, 0, 1.0, 0, 0, 0, 0.005)
        for _ in range(30):
            st = est.update(0.4, 0, 1.0, 0, 0, 0, 0.005)
        self.assertEqual(st["dir"], "+X")
        self.assertGreater(est.vel[0], 0.1)
        for _ in range(40):
            st = est.update(0, 0, 1.0, 0, 0, 0, 0.005)
        self.assertTrue(st["stationary"])
        self.assertEqual(st["dir"], "STATIC")
        self.assertEqual(est.vel[0], 0.0)

    def test_tilted_world_frame_forward(self):
        est = MotionEstimatorRef()
        rad45 = 45.0 * DEG_TO_RAD
        for i in range(200):
            a = (i + 1) * 0.005 * 45.0 * DEG_TO_RAD
            est.update(0, math.sin(a), math.cos(a), 45.0, 0, 0, 0.005)
        for _ in range(60):
            st = est.update(0, math.sin(rad45), math.cos(rad45), 0, 0, 0, 0.005)
        self.assertAlmostEqual(st["roll"], 45.0, delta=2.0)
        self.assertTrue(st["stationary"])

        ay_b = 0.5 * math.cos(rad45) + 1.0 * math.sin(rad45)
        az_b = -0.5 * math.sin(rad45) + 1.0 * math.cos(rad45)
        for _ in range(25):
            st = est.update(0, ay_b, az_b, 0, 0, 0, 0.005)
        self.assertEqual(st["dir"], "+Y")
        self.assertGreater(est.vel[1], 0.1)
        self.assertLess(abs(est.vel[2]), abs(est.vel[1]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
