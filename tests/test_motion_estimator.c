#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_estimator.h"

#define DT_200HZ 0.005f

static int near_f(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

static void test_stationary_flat(void)
{
    motion_estimator_t est;
    motion_estimator_config_t cfg = motion_estimator_default_config();
    motion_estimator_init(&est, &cfg);

    motion_state_t st = {0};
    for (int i = 0; i < 100; i++) {
        motion_estimator_update(&est, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }

    assert(st.stationary == true);
    assert(st.dir == MOTION_DIR_STATIC);
    assert(near_f(st.roll_deg, 0.0f, 0.5f));
    assert(near_f(st.pitch_deg, 0.0f, 0.5f));
    assert(near_f(st.lin_acc_mps2[0], 0.0f, 0.05f));
    assert(near_f(st.lin_acc_mps2[1], 0.0f, 0.05f));
    assert(near_f(st.lin_acc_mps2[2], 0.0f, 0.05f));
    assert(near_f(st.vel_mps[0], 0.0f, 1e-4f));
    printf("[PASS] test_stationary_flat\n");
}

static void test_pure_pitch_keeps_world_lin_acc_zero(void)
{
    motion_estimator_t est;
    motion_estimator_config_t cfg = motion_estimator_default_config();
    motion_estimator_init(&est, &cfg);

    /* Pitch nose up by 45 deg around +Y? Wait: nose up around Y is -45 deg in right-hand rule:
       Rotating +45 deg around +X (Roll) tilts +Z toward -Y and +Y toward +Z:
       Body gravity vector when rolled +45 deg around X: ax=0, ay=sin(45), az=cos(45). */
    const float roll_rate_dps = 45.0f; /* 1 second = 45 deg */
    const float deg2rad = 0.0174532925f;
    motion_state_t st = {0};

    for (int i = 0; i < 200; i++) {
        float angle_rad = (i + 1) * DT_200HZ * roll_rate_dps * deg2rad;
        float ax = 0.0f;
        float ay = sinf(angle_rad);
        float az = cosf(angle_rad);
        motion_estimator_update(&est, ax, ay, az, roll_rate_dps, 0.0f, 0.0f, DT_200HZ, &st);
    }

    /* Hold steady at 45 deg roll for 0.5s */
    float angle_rad = 45.0f * deg2rad;
    for (int i = 0; i < 100; i++) {
        motion_estimator_update(&est, 0.0f, sinf(angle_rad), cosf(angle_rad), 0.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }

    assert(near_f(st.roll_deg, 45.0f, 2.0f));
    assert(near_f(st.lin_acc_mps2[0], 0.0f, 0.25f));
    assert(near_f(st.lin_acc_mps2[1], 0.0f, 0.25f));
    assert(near_f(st.lin_acc_mps2[2], 0.0f, 0.25f));
    assert(st.stationary == true);
    printf("[PASS] test_pure_pitch_keeps_world_lin_acc_zero (roll=%.2f deg)\n", st.roll_deg);
}

static void test_translation_pos_x_and_zupt_recovery(void)
{
    motion_estimator_t est;
    motion_estimator_config_t cfg = motion_estimator_default_config();
    motion_estimator_init(&est, &cfg);

    motion_state_t st = {0};
    /* Settle */
    for (int i = 0; i < 40; i++) {
        motion_estimator_update(&est, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }

    /* Push +X (+0.4g for 150ms = 30 samples) */
    for (int i = 0; i < 30; i++) {
        motion_estimator_update(&est, 0.4f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }
    assert(st.dir == MOTION_DIR_POS_X);
    assert(st.vel_mps[0] > 0.1f);
    assert(st.pos_m[0] > 0.0f);

    /* Stop (0g linear) for 200ms = 40 samples -> Dual ZUPT must zero velocity */
    for (int i = 0; i < 40; i++) {
        motion_estimator_update(&est, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }
    assert(st.stationary == true);
    assert(st.dir == MOTION_DIR_STATIC);
    assert(near_f(st.vel_mps[0], 0.0f, 1e-4f));
    printf("[PASS] test_translation_pos_x_and_zupt_recovery\n");
}

static void test_tilted_world_frame_forward_motion(void)
{
    motion_estimator_t est;
    motion_estimator_config_t cfg = motion_estimator_default_config();
    motion_estimator_init(&est, &cfg);

    /* Hold board rolled +45 deg around X, then accelerate along World +Y (Forward).
       In World frame, +Y acceleration = (0, +0.5g, 0) plus gravity (0, 0, +1g) => (0, +0.5g, +1g).
       Projected into body frame rotated +45 deg around X:
       ay_body = +0.5*cos(45) + 1.0*sin(45)
       az_body = -0.5*sin(45) + 1.0*cos(45) */
    const float rad45 = 45.0f * 0.0174532925f;
    motion_state_t st = {0};

    /* First reach 45 deg roll smoothly */
    for (int i = 0; i < 200; i++) {
        float a = (i + 1) * DT_200HZ * 45.0f * 0.0174532925f;
        motion_estimator_update(&est, 0.0f, sinf(a), cosf(a), 45.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }
    for (int i = 0; i < 60; i++) {
        motion_estimator_update(&est, 0.0f, sinf(rad45), cosf(rad45), 0.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }

    /* Accelerate along World +Y (+0.5g) while staying tilted 45 deg */
    float ay_body = 0.5f * cosf(rad45) + 1.0f * sinf(rad45);
    float az_body = -0.5f * sinf(rad45) + 1.0f * cosf(rad45);
    for (int i = 0; i < 25; i++) {
        motion_estimator_update(&est, 0.0f, ay_body, az_body, 0.0f, 0.0f, 0.0f, DT_200HZ, &st);
    }

    assert(st.dir == MOTION_DIR_POS_Y);
    assert(st.vel_mps[1] > 0.1f);
    assert(fabsf(st.vel_mps[2]) < fabsf(st.vel_mps[1]));
    printf("[PASS] test_tilted_world_frame_forward_motion (vy=%.2f, vz=%.2f, dir=%s)\n",
           st.vel_mps[1], st.vel_mps[2], motion_dir_to_str(st.dir));
}

int main(void)
{
    test_stationary_flat();
    test_pure_pitch_keeps_world_lin_acc_zero();
    test_translation_pos_x_and_zupt_recovery();
    test_tilted_world_frame_forward_motion();
    printf("All motion_estimator tests passed!\n");
    return 0;
}
