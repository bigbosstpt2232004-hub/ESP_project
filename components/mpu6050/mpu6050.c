#include "mpu6050.h"
#include "hal_i2c_config.h"

#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

struct mpu6050_dev_s {
    hal_i2c_bus_handle_t bus_handle;
    hal_i2c_dev_handle_t dev_handle;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
    mpu6050_calib_t calib;
};

static float accel_scale_from_fs(mpu6050_accel_fs_t fs)
{
    switch (fs) {
    case MPU6050_ACCEL_FS_2G:  return 16384.0f;
    case MPU6050_ACCEL_FS_4G:  return 8192.0f;
    case MPU6050_ACCEL_FS_8G:  return 4096.0f;
    case MPU6050_ACCEL_FS_16G: return 2048.0f;
    default:                   return 8192.0f;
    }
}

static float gyro_scale_from_fs(mpu6050_gyro_fs_t fs)
{
    switch (fs) {
    case MPU6050_GYRO_FS_250DPS:  return 131.0f;
    case MPU6050_GYRO_FS_500DPS:  return 65.5f;
    case MPU6050_GYRO_FS_1000DPS: return 32.8f;
    case MPU6050_GYRO_FS_2000DPS: return 16.4f;
    default:                      return 65.5f;
    }
}

static int16_t be16_to_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

esp_err_t mpu6050_init(const mpu6050_config_t *config, mpu6050_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    int sda_lvl = 1, scl_lvl = 1;
    hal_i2c_check_lines(config->i2c_bus.sda_pin, config->i2c_bus.scl_pin, &sda_lvl, &scl_lvl);
    if (sda_lvl == 0 || scl_lvl == 0) {
        ESP_LOGE(TAG, "I2C line stuck LOW on SDA=%d or SCL=%d! Check power/wiring.",
                 config->i2c_bus.sda_pin, config->i2c_bus.scl_pin);
    }

    struct mpu6050_dev_s *dev = calloc(1, sizeof(struct mpu6050_dev_s));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Out of memory");

    esp_err_t ret = hal_i2c_bus_init(&config->i2c_bus, &dev->bus_handle);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    uint8_t target_addr = config->dev_addr;
    esp_err_t probe_ret = hal_i2c_probe(dev->bus_handle, target_addr, HAL_I2C_TIMEOUT_MS);
    if (probe_ret != ESP_OK) {
        uint8_t alt_addr = (target_addr == MPU6050_I2C_ADDR_LOW) ? MPU6050_I2C_ADDR_HIGH : MPU6050_I2C_ADDR_LOW;
        if (hal_i2c_probe(dev->bus_handle, alt_addr, HAL_I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGW(TAG, "MPU6050 not at 0x%02X, found at 0x%02X (AD0 pin state)", target_addr, alt_addr);
            target_addr = alt_addr;
        } else {
            if (sda_lvl == 1 && scl_lvl == 1) {
                hal_i2c_bus_deinit(dev->bus_handle);
                dev->bus_handle = NULL;

                hal_i2c_bus_config_t swap_cfg = config->i2c_bus;
                swap_cfg.sda_pin = config->i2c_bus.scl_pin;
                swap_cfg.scl_pin = config->i2c_bus.sda_pin;
                hal_i2c_bus_handle_t swap_bus = NULL;
                if (hal_i2c_bus_init(&swap_cfg, &swap_bus) == ESP_OK) {
                    if (hal_i2c_probe(swap_bus, MPU6050_I2C_ADDR_LOW, 50) == ESP_OK ||
                        hal_i2c_probe(swap_bus, MPU6050_I2C_ADDR_HIGH, 50) == ESP_OK) {
                        ESP_LOGE(TAG, "FOUND MPU6050 WITH SWAPPED WIRES! Swap GPIO%d and GPIO%d.",
                                 config->i2c_bus.sda_pin, config->i2c_bus.scl_pin);
                    }
                    hal_i2c_bus_deinit(swap_bus);
                }
            }
            ESP_LOGE(TAG, "MPU6050 not detected on I2C bus (SDA=%d, SCL=%d, probe=%s)",
                     config->i2c_bus.sda_pin, config->i2c_bus.scl_pin, esp_err_to_name(probe_ret));
            if (dev->bus_handle) {
                hal_i2c_bus_deinit(dev->bus_handle);
            }
            free(dev);
            return ESP_ERR_NOT_FOUND;
        }
    }

    ret = hal_i2c_add_device(dev->bus_handle, target_addr, config->i2c_bus.freq_hz, &dev->dev_handle);
    if (ret != ESP_OK) {
        mpu6050_deinit(dev);
        return ret;
    }

    uint8_t who_am_i = 0;
    ret = mpu6050_read_who_am_i(dev, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        mpu6050_deinit(dev);
        return ret;
    }
    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I=0x%02X (expected 0x%02X); continuing for compatible IMU",
                 who_am_i, MPU6050_WHO_AM_I_VAL);
    } else {
        ESP_LOGI(TAG, "MPU6050 detected at 0x%02X (WHO_AM_I=0x%02X)", target_addr, who_am_i);
    }

    ESP_GOTO_ON_ERROR(hal_i2c_write_reg(dev->dev_handle, MPU6050_REG_PWR_MGMT_1, 0x80, HAL_I2C_TIMEOUT_MS),
                      fail, TAG, "Reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_GOTO_ON_ERROR(hal_i2c_write_reg(dev->dev_handle, MPU6050_REG_PWR_MGMT_1, 0x01, HAL_I2C_TIMEOUT_MS),
                      fail, TAG, "Wakeup failed");
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_GOTO_ON_ERROR(hal_i2c_write_reg(dev->dev_handle, MPU6050_REG_CONFIG, config->dlpf_cfg & 0x07, HAL_I2C_TIMEOUT_MS),
                      fail, TAG, "DLPF config failed");
    ESP_GOTO_ON_ERROR(hal_i2c_write_reg(dev->dev_handle, MPU6050_REG_SMPLRT_DIV, config->smplrt_div, HAL_I2C_TIMEOUT_MS),
                      fail, TAG, "Sample rate div failed");
    ESP_GOTO_ON_ERROR(hal_i2c_write_reg(dev->dev_handle, MPU6050_REG_GYRO_CONFIG, (uint8_t)((config->gyro_fs & 0x03) << 3), HAL_I2C_TIMEOUT_MS),
                      fail, TAG, "Gyro config failed");
    ESP_GOTO_ON_ERROR(hal_i2c_write_reg(dev->dev_handle, MPU6050_REG_ACCEL_CONFIG, (uint8_t)((config->accel_fs & 0x03) << 3), HAL_I2C_TIMEOUT_MS),
                      fail, TAG, "Accel config failed");

    dev->accel_lsb_per_g = accel_scale_from_fs(config->accel_fs);
    dev->gyro_lsb_per_dps = gyro_scale_from_fs(config->gyro_fs);
    *out_handle = dev;
    return ESP_OK;

fail:
    mpu6050_deinit(dev);
    return ret;
}

esp_err_t mpu6050_deinit(mpu6050_handle_t handle)
{
    if (!handle) return ESP_OK;
    if (handle->dev_handle) hal_i2c_remove_device(handle->dev_handle);
    if (handle->bus_handle) hal_i2c_bus_deinit(handle->bus_handle);
    free(handle);
    return ESP_OK;
}

esp_err_t mpu6050_read_who_am_i(mpu6050_handle_t handle, uint8_t *who_am_i)
{
    ESP_RETURN_ON_FALSE(handle && who_am_i, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    return hal_i2c_read_regs(handle->dev_handle, MPU6050_REG_WHO_AM_I, who_am_i, 1, HAL_I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_read_raw(mpu6050_handle_t handle, mpu6050_raw_t *raw)
{
    ESP_RETURN_ON_FALSE(handle && raw, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uint8_t buf[14];
    ESP_RETURN_ON_ERROR(hal_i2c_read_regs(handle->dev_handle, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf), HAL_I2C_TIMEOUT_MS),
                        TAG, "Burst read failed");

    raw->ax   = be16_to_i16(&buf[0]);
    raw->ay   = be16_to_i16(&buf[2]);
    raw->az   = be16_to_i16(&buf[4]);
    raw->temp = be16_to_i16(&buf[6]);
    raw->gx   = be16_to_i16(&buf[8]);
    raw->gy   = be16_to_i16(&buf[10]);
    raw->gz   = be16_to_i16(&buf[12]);
    return ESP_OK;
}

esp_err_t mpu6050_read_scaled(mpu6050_handle_t handle, mpu6050_data_t *data)
{
    ESP_RETURN_ON_FALSE(handle && data, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    mpu6050_raw_t raw;
    ESP_RETURN_ON_ERROR(mpu6050_read_raw(handle, &raw), TAG, "Raw read failed");

    data->ax_g   = ((float)raw.ax / handle->accel_lsb_per_g) - handle->calib.accel_bias_g[0];
    data->ay_g   = ((float)raw.ay / handle->accel_lsb_per_g) - handle->calib.accel_bias_g[1];
    data->az_g   = ((float)raw.az / handle->accel_lsb_per_g) - handle->calib.accel_bias_g[2];
    data->gx_dps = ((float)raw.gx / handle->gyro_lsb_per_dps) - handle->calib.gyro_bias_dps[0];
    data->gy_dps = ((float)raw.gy / handle->gyro_lsb_per_dps) - handle->calib.gyro_bias_dps[1];
    data->gz_dps = ((float)raw.gz / handle->gyro_lsb_per_dps) - handle->calib.gyro_bias_dps[2];
    data->temp_c = ((float)raw.temp / 340.0f) + 36.53f;
    return ESP_OK;
}

esp_err_t mpu6050_calibrate_stationary(mpu6050_handle_t handle, uint16_t samples, uint32_t delay_ms)
{
    ESP_RETURN_ON_FALSE(handle && samples > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    double sum_a[3] = {0.0, 0.0, 0.0};
    double sum_g[3] = {0.0, 0.0, 0.0};

    for (uint16_t i = 0; i < samples; i++) {
        mpu6050_raw_t raw;
        ESP_RETURN_ON_ERROR(mpu6050_read_raw(handle, &raw), TAG, "Calibration read failed");
        sum_a[0] += (double)raw.ax / handle->accel_lsb_per_g;
        sum_a[1] += (double)raw.ay / handle->accel_lsb_per_g;
        sum_a[2] += (double)raw.az / handle->accel_lsb_per_g;
        sum_g[0] += (double)raw.gx / handle->gyro_lsb_per_dps;
        sum_g[1] += (double)raw.gy / handle->gyro_lsb_per_dps;
        sum_g[2] += (double)raw.gz / handle->gyro_lsb_per_dps;
        if (delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }

    handle->calib.accel_bias_g[0] = (float)(sum_a[0] / samples);
    handle->calib.accel_bias_g[1] = (float)(sum_a[1] / samples);
    handle->calib.accel_bias_g[2] = (float)(sum_a[2] / samples - 1.0);

    handle->calib.gyro_bias_dps[0] = (float)(sum_g[0] / samples);
    handle->calib.gyro_bias_dps[1] = (float)(sum_g[1] / samples);
    handle->calib.gyro_bias_dps[2] = (float)(sum_g[2] / samples);
    handle->calib.valid = true;

    ESP_LOGI(TAG, "Calibrated: accel_bias=(%.4f, %.4f, %.4f)g, gyro_bias=(%.3f, %.3f, %.3f)dps",
             handle->calib.accel_bias_g[0], handle->calib.accel_bias_g[1], handle->calib.accel_bias_g[2],
             handle->calib.gyro_bias_dps[0], handle->calib.gyro_bias_dps[1], handle->calib.gyro_bias_dps[2]);
    return ESP_OK;
}

esp_err_t mpu6050_get_calibration(mpu6050_handle_t handle, mpu6050_calib_t *out_calib)
{
    ESP_RETURN_ON_FALSE(handle && out_calib, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    *out_calib = handle->calib;
    return ESP_OK;
}
