#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "hal_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR_LOW  0x68
#define MPU6050_I2C_ADDR_HIGH 0x69
#define MPU6050_WHO_AM_I_VAL  0x68

typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,
    MPU6050_ACCEL_FS_4G  = 1,
    MPU6050_ACCEL_FS_8G  = 2,
    MPU6050_ACCEL_FS_16G = 3,
} mpu6050_accel_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,
    MPU6050_GYRO_FS_500DPS  = 1,
    MPU6050_GYRO_FS_1000DPS = 2,
    MPU6050_GYRO_FS_2000DPS = 3,
} mpu6050_gyro_fs_t;

typedef struct {
    hal_i2c_bus_config_t i2c_bus;
    uint8_t dev_addr;
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t gyro_fs;
    uint8_t dlpf_cfg;     /* 0..6, 3 = ~44Hz accel / 42Hz gyro @ 1kHz internal */
    uint8_t smplrt_div;   /* Sample Rate = 1kHz / (1 + smplrt_div) when dlpf_cfg 1..6 */
} mpu6050_config_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu6050_raw_t;

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float temp_c;
} mpu6050_data_t;

typedef struct {
    float accel_bias_g[3];
    float gyro_bias_dps[3];
    bool valid;
} mpu6050_calib_t;

typedef struct mpu6050_dev_s *mpu6050_handle_t;

/**
 * @brief Initialize HAL I2C bus and configure MPU6050 sensor.
 */
esp_err_t mpu6050_init(const mpu6050_config_t *config, mpu6050_handle_t *out_handle);

/**
 * @brief Release MPU6050 device and HAL I2C bus resources.
 */
esp_err_t mpu6050_deinit(mpu6050_handle_t handle);

/**
 * @brief Read WHO_AM_I register (expected 0x68 for standard MPU6050).
 */
esp_err_t mpu6050_read_who_am_i(mpu6050_handle_t handle, uint8_t *who_am_i);

/**
 * @brief Burst-read 14 bytes (Accel XYZ, Temp, Gyro XYZ) in one I2C transaction.
 */
esp_err_t mpu6050_read_raw(mpu6050_handle_t handle, mpu6050_raw_t *raw);

/**
 * @brief Read scaled, bias-corrected acceleration (g), angular velocity (deg/s), and temperature (C).
 */
esp_err_t mpu6050_read_scaled(mpu6050_handle_t handle, mpu6050_data_t *data);

/**
 * @brief Calibrate stationary gyro and accel bias assuming device rests flat (+Z up).
 */
esp_err_t mpu6050_calibrate_stationary(mpu6050_handle_t handle, uint16_t samples, uint32_t delay_ms);

/**
 * @brief Copy current calibration offsets.
 */
esp_err_t mpu6050_get_calibration(mpu6050_handle_t handle, mpu6050_calib_t *out_calib);

#ifdef __cplusplus
}
#endif
