#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal_i2c_config.h"
#include "motion_estimator.h"
#include "mpu6050.h"

static const char *TAG = "main";

#define SAMPLE_PERIOD_MS 5 /* 200 Hz */
#define STREAM_DECIMATION 4 /* 50 Hz JSON output */

static void process_serial_commands(mpu6050_handle_t imu, motion_estimator_t *est, char *cmd_buf, size_t *cmd_len, size_t cap)
{
    char ch;
    while (read(STDIN_FILENO, &ch, 1) == 1) {
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            cmd_buf[*cmd_len] = '\0';
            if (strcmp(cmd_buf, "CAL") == 0) {
                ESP_LOGI(TAG, "Recalibrating stationary bias...");
                if (mpu6050_calibrate_stationary(imu, 300, 5) == ESP_OK) {
                    motion_estimator_reset(est);
                    printf("{\"evt\":\"calibrated\"}\n");
                }
            } else if (strcmp(cmd_buf, "RESET") == 0) {
                motion_estimator_zero_yaw(est);
                printf("{\"evt\":\"reset\"}\n");
            }
            *cmd_len = 0;
        } else if (*cmd_len + 1 < cap) {
            cmd_buf[(*cmd_len)++] = ch;
        } else {
            *cmd_len = 0;
        }
    }
}

void app_main(void)
{
    /* Make stdin non-blocking so CAL / RESET commands never stall the 200 Hz IMU loop */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    mpu6050_config_t imu_cfg = {
        .i2c_bus = HAL_I2C_BUS_DEFAULT_CONFIG(),
        .dev_addr = HAL_MPU6050_I2C_ADDR,
        .accel_fs = MPU6050_ACCEL_FS_4G,
        .gyro_fs = MPU6050_GYRO_FS_500DPS,
        .dlpf_cfg = 3,
        .smplrt_div = 4, /* 1000 / (1 + 4) = 200 Hz */
    };

    mpu6050_handle_t imu = NULL;
    esp_err_t err = mpu6050_init(&imu_cfg, &imu);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed (%s). Check wiring: SDA=GPIO%d, SCL=GPIO%d, VCC=3V3/5V, GND=GND, AD0=GND",
                 esp_err_to_name(err), HAL_I2C_SDA_PIN, HAL_I2C_SCL_PIN);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Keep device flat and still for initial calibration (~1.5s)...");
    ESP_ERROR_CHECK(mpu6050_calibrate_stationary(imu, 300, 5));

    motion_estimator_t estimator;
    motion_estimator_config_t est_cfg = motion_estimator_default_config();
    motion_estimator_init(&estimator, &est_cfg);

    TickType_t last_wake = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();
    uint32_t sample_idx = 0;

    char cmd_buf[32];
    size_t cmd_len = 0;

    ESP_LOGI(TAG, "Streaming 6-DOF motion telemetry at 50 Hz (200 Hz internal fusion)");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        process_serial_commands(imu, &estimator, cmd_buf, &cmd_len, sizeof(cmd_buf));

        mpu6050_data_t imu_data;
        if (mpu6050_read_scaled(imu, &imu_data) != ESP_OK) {
            ESP_LOGW(TAG, "I2C read error");
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - prev_us) * 1e-6f;
        prev_us = now_us;
        if (dt_s <= 0.0f || dt_s > 0.05f) {
            dt_s = (float)SAMPLE_PERIOD_MS * 1e-3f;
        }

        motion_state_t state;
        motion_estimator_update(&estimator,
                                imu_data.ax_g, imu_data.ay_g, imu_data.az_g,
                                imu_data.gx_dps, imu_data.gy_dps, imu_data.gz_dps,
                                dt_s, &state);

        if ((++sample_idx % STREAM_DECIMATION) == 0) {
            uint32_t t_ms = (uint32_t)(now_us / 1000);
            printf("{\"t\":%lu,\"qw\":%.3f,\"qx\":%.3f,\"qy\":%.3f,\"qz\":%.3f,"
                   "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
                   "\"vx\":%.2f,\"vy\":%.2f,\"vz\":%.2f,"
                   "\"px\":%.3f,\"py\":%.3f,\"pz\":%.3f,"
                   "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"dir\":\"%s\"}\n",
                   (unsigned long)t_ms,
                   state.qw, state.qx, state.qy, state.qz,
                   state.lin_acc_mps2[0], state.lin_acc_mps2[1], state.lin_acc_mps2[2],
                   state.vel_mps[0], state.vel_mps[1], state.vel_mps[2],
                   state.pos_m[0], state.pos_m[1], state.pos_m[2],
                   state.roll_deg, state.pitch_deg, state.yaw_deg,
                   motion_dir_to_str(state.dir));
        }
    }
}
