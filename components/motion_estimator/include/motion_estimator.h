#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTION_DIR_STATIC = 0,
    MOTION_DIR_POS_X,
    MOTION_DIR_NEG_X,
    MOTION_DIR_POS_Y,
    MOTION_DIR_NEG_Y,
    MOTION_DIR_POS_Z,
    MOTION_DIR_NEG_Z,
    MOTION_DIR_ROTATING,
} motion_dir_t;

typedef struct {
    float beta;               /* Madgwick filter gain (default ~0.06) */
    float lin_acc_lpf_alpha;  /* EMA factor for linear acceleration (0..1, higher = smoother) */
    float accel_deadband_g;   /* Stillness threshold for linear acceleration (g) */
    float gyro_deadband_dps;  /* Stillness threshold for gyro magnitude (deg/s) */
    uint16_t zupt_hold_samples; /* Consecutive still samples before ZUPT triggers */
    float vel_decay;          /* Per-step velocity damping when moving */
    float pos_return_decay;   /* Per-step centering decay during ZUPT */
    float vel_dir_thresh_mps; /* Velocity threshold to classify translation direction */
    float rot_dir_thresh_dps; /* Gyro threshold to classify pure rotation */
} motion_estimator_config_t;

typedef struct {
    float qw, qx, qy, qz;
    float roll_deg, pitch_deg, yaw_deg;
    float lin_acc_mps2[3];
    float vel_mps[3];
    float pos_m[3];
    motion_dir_t dir;
    bool stationary;
} motion_state_t;

typedef struct {
    motion_estimator_config_t cfg;
    float q[4];
    float lin_acc_mps2[3];
    float vel_mps[3];
    float pos_m[3];
    uint16_t still_count;
} motion_estimator_t;

/**
 * @brief Return default configuration tuned for 200 Hz MPU6050 handheld motion.
 */
motion_estimator_config_t motion_estimator_default_config(void);

/**
 * @brief Initialize motion estimator state and quaternion to identity.
 */
void motion_estimator_init(motion_estimator_t *est, const motion_estimator_config_t *cfg);

/**
 * @brief Reset full orientation, velocity, and simulated position.
 */
void motion_estimator_reset(motion_estimator_t *est);

/**
 * @brief Reset Yaw to 0 and simulated position to origin while preserving Roll/Pitch.
 */
void motion_estimator_zero_yaw(motion_estimator_t *est);

/**
 * @brief Update 6-DOF fusion, world-frame linear acceleration, Dual ZUPT, and direction.
 */
void motion_estimator_update(motion_estimator_t *est,
                             float ax_g, float ay_g, float az_g,
                             float gx_dps, float gy_dps, float gz_dps,
                             float dt_s,
                             motion_state_t *out_state);

/**
 * @brief Return short string tag for motion_dir_t ("STATIC", "+X", "-X", ..., "ROT").
 */
const char *motion_dir_to_str(motion_dir_t dir);

#ifdef __cplusplus
}
#endif
