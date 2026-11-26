#include "motion_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define GRAVITY_MPS2 9.80665f
#define DEG_TO_RAD   0.01745329251994329577f
#define RAD_TO_DEG   57.2957795130823208768f

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void quat_to_euler_deg(const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    float sinp = clampf(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f);
    *pitch_deg = asinf(sinp) * RAD_TO_DEG;

    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw_deg = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

static void euler_rad_to_quat(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static void madgwick_update_6dof(float q[4], float beta,
                                 float gx_rad, float gy_rad, float gz_rad,
                                 float ax, float ay, float az,
                                 float dt)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    float qDot1 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
    float qDot2 = 0.5f * ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
    float qDot3 = 0.5f * ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
    float qDot4 = 0.5f * ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

    float acc_norm_sq = ax * ax + ay * ay + az * az;
    if (acc_norm_sq > 1e-8f) {
        float inv_norm = 1.0f / sqrtf(acc_norm_sq);
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;

        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        float s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (s_norm_sq > 1e-12f) {
            float inv_s = 1.0f / sqrtf(s_norm_sq);
            qDot1 -= beta * s0 * inv_s;
            qDot2 -= beta * s1 * inv_s;
            qDot3 -= beta * s2 * inv_s;
            qDot4 -= beta * s3 * inv_s;
        }
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_norm > 1e-8f) {
        float inv_q = 1.0f / q_norm;
        q[0] = q0 * inv_q;
        q[1] = q1 * inv_q;
        q[2] = q2 * inv_q;
        q[3] = q3 * inv_q;
    } else {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
    }
}

static motion_dir_t axis_to_dir(int axis, float value)
{
    if (axis == 0) return (value >= 0.0f) ? MOTION_DIR_POS_X : MOTION_DIR_NEG_X;
    if (axis == 1) return (value >= 0.0f) ? MOTION_DIR_POS_Y : MOTION_DIR_NEG_Y;
    return (value >= 0.0f) ? MOTION_DIR_POS_Z : MOTION_DIR_NEG_Z;
}

static int dominant_axis(const float v[3], float *out_abs_max)
{
    float ax = fabsf(v[0]);
    float ay = fabsf(v[1]);
    float az = fabsf(v[2]);

    if (ax >= ay && ax >= az) {
        *out_abs_max = ax;
        return 0;
    }
    if (ay >= ax && ay >= az) {
        *out_abs_max = ay;
        return 1;
    }
    *out_abs_max = az;
    return 2;
}

motion_estimator_config_t motion_estimator_default_config(void)
{
    motion_estimator_config_t cfg = {
        .beta = 0.06f,
        .lin_acc_lpf_alpha = 0.35f,
        .accel_deadband_g = 0.045f,
        .gyro_deadband_dps = 3.0f,
        .zupt_hold_samples = 16,
        .vel_decay = 0.985f,
        .pos_return_decay = 0.94f,
        .vel_dir_thresh_mps = 0.03f,
        .rot_dir_thresh_dps = 15.0f,
    };
    return cfg;
}

void motion_estimator_init(motion_estimator_t *est, const motion_estimator_config_t *cfg)
{
    if (!est) return;
    memset(est, 0, sizeof(*est));
    est->cfg = cfg ? *cfg : motion_estimator_default_config();
    est->q[0] = 1.0f;
}

void motion_estimator_reset(motion_estimator_t *est)
{
    if (!est) return;
    motion_estimator_config_t saved = est->cfg;
    motion_estimator_init(est, &saved);
}

void motion_estimator_zero_yaw(motion_estimator_t *est)
{
    if (!est) return;
    float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
    quat_to_euler_deg(est->q, &roll_deg, &pitch_deg, &yaw_deg);
    euler_rad_to_quat(roll_deg * DEG_TO_RAD, pitch_deg * DEG_TO_RAD, 0.0f, est->q);

    memset(est->vel_mps, 0, sizeof(est->vel_mps));
    memset(est->pos_m, 0, sizeof(est->pos_m));
    est->still_count = est->cfg.zupt_hold_samples;
}

void motion_estimator_update(motion_estimator_t *est,
                             float ax_g, float ay_g, float az_g,
                             float gx_dps, float gy_dps, float gz_dps,
                             float dt_s,
                             motion_state_t *out_state)
{
    if (!est || dt_s <= 0.0f) return;

    /* 1. Update 6-DOF Madgwick quaternion */
    madgwick_update_6dof(est->q, est->cfg.beta,
                         gx_dps * DEG_TO_RAD, gy_dps * DEG_TO_RAD, gz_dps * DEG_TO_RAD,
                         ax_g, ay_g, az_g, dt_s);

    /* 2. Rotate body-frame acceleration into world frame and subtract gravity (+1g on Z) */
    float q0 = est->q[0], q1 = est->q[1], q2 = est->q[2], q3 = est->q[3];
    float awx_g = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ax_g
                + 2.0f * (q1 * q2 - q0 * q3) * ay_g
                + 2.0f * (q1 * q3 + q0 * q2) * az_g;
    float awy_g = 2.0f * (q1 * q2 + q0 * q3) * ax_g
                + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * ay_g
                + 2.0f * (q2 * q3 - q0 * q1) * az_g;
    float awz_g = 2.0f * (q1 * q3 - q0 * q2) * ax_g
                + 2.0f * (q2 * q3 + q0 * q1) * ay_g
                + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * az_g;

    float lin_g[3] = {awx_g, awy_g, awz_g - 1.0f};
    float alpha = clampf(est->cfg.lin_acc_lpf_alpha, 0.0f, 0.99f);
    for (int i = 0; i < 3; i++) {
        float target = lin_g[i] * GRAVITY_MPS2;
        est->lin_acc_mps2[i] = alpha * est->lin_acc_mps2[i] + (1.0f - alpha) * target;
    }

    float lin_mag_g = sqrtf(est->lin_acc_mps2[0] * est->lin_acc_mps2[0]
                          + est->lin_acc_mps2[1] * est->lin_acc_mps2[1]
                          + est->lin_acc_mps2[2] * est->lin_acc_mps2[2]) / GRAVITY_MPS2;
    float gyro_mag_dps = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);

    /* 3. Dual ZUPT (Accel + Gyro stillness check) */
    bool is_still_sample = (lin_mag_g < est->cfg.accel_deadband_g) && (gyro_mag_dps < est->cfg.gyro_deadband_dps);
    if (is_still_sample) {
        if (est->still_count < est->cfg.zupt_hold_samples) {
            est->still_count++;
        }
    } else {
        est->still_count = 0;
    }

    bool stationary = (est->still_count >= est->cfg.zupt_hold_samples);
    float deadband_mps2 = est->cfg.accel_deadband_g * GRAVITY_MPS2;

    if (stationary) {
        for (int i = 0; i < 3; i++) {
            est->vel_mps[i] = 0.0f;
            est->pos_m[i] *= est->cfg.pos_return_decay;
            if (fabsf(est->pos_m[i]) < 1e-4f) {
                est->pos_m[i] = 0.0f;
            }
        }
    } else {
        for (int i = 0; i < 3; i++) {
            float a = est->lin_acc_mps2[i];
            if (fabsf(a) < deadband_mps2 * 0.7f) {
                a = 0.0f;
            }
            est->vel_mps[i] = clampf(est->vel_mps[i] * est->cfg.vel_decay + a * dt_s, -3.0f, 3.0f);
            est->pos_m[i] = clampf(est->pos_m[i] + est->vel_mps[i] * dt_s, -0.5f, 0.5f);
        }
    }

    /* 4. Classify primary direction of motion */
    motion_dir_t dir = MOTION_DIR_STATIC;
    if (!stationary) {
        float max_vel = 0.0f;
        int vel_axis = dominant_axis(est->vel_mps, &max_vel);
        if (max_vel >= est->cfg.vel_dir_thresh_mps) {
            dir = axis_to_dir(vel_axis, est->vel_mps[vel_axis]);
        } else {
            float max_acc = 0.0f;
            int acc_axis = dominant_axis(est->lin_acc_mps2, &max_acc);
            if (max_acc >= deadband_mps2) {
                dir = axis_to_dir(acc_axis, est->lin_acc_mps2[acc_axis]);
            } else if (gyro_mag_dps >= est->cfg.rot_dir_thresh_dps) {
                dir = MOTION_DIR_ROTATING;
            }
        }
    }

    if (out_state) {
        out_state->qw = est->q[0];
        out_state->qx = est->q[1];
        out_state->qy = est->q[2];
        out_state->qz = est->q[3];
        quat_to_euler_deg(est->q, &out_state->roll_deg, &out_state->pitch_deg, &out_state->yaw_deg);
        memcpy(out_state->lin_acc_mps2, est->lin_acc_mps2, sizeof(out_state->lin_acc_mps2));
        memcpy(out_state->vel_mps, est->vel_mps, sizeof(out_state->vel_mps));
        memcpy(out_state->pos_m, est->pos_m, sizeof(out_state->pos_m));
        out_state->dir = dir;
        out_state->stationary = stationary;
    }
}

const char *motion_dir_to_str(motion_dir_t dir)
{
    switch (dir) {
    case MOTION_DIR_STATIC:   return "STATIC";
    case MOTION_DIR_POS_X:    return "+X";
    case MOTION_DIR_NEG_X:    return "-X";
    case MOTION_DIR_POS_Y:    return "+Y";
    case MOTION_DIR_NEG_Y:    return "-Y";
    case MOTION_DIR_POS_Z:    return "+Z";
    case MOTION_DIR_NEG_Z:    return "-Z";
    case MOTION_DIR_ROTATING: return "ROT";
    default:                  return "STATIC";
    }
}
