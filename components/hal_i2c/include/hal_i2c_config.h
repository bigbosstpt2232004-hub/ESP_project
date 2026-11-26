#pragma once

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Central Hardware Abstraction Layer (HAL) configuration for ESP32-S3 N16R8 I2C & MPU6050.
 *
 * Avoid GPIO26-37 (Octal PSRAM/Flash), GPIO19/20 (Native USB), and GPIO0/3/45/46 (Strapping).
 * Default safe pins verified at logic HIGH on board: SDA=GPIO10, SCL=GPIO11.
 */

#ifdef CONFIG_MPU6050_I2C_SDA
#define HAL_I2C_SDA_PIN CONFIG_MPU6050_I2C_SDA
#else
#define HAL_I2C_SDA_PIN 10
#endif

#ifdef CONFIG_MPU6050_I2C_SCL
#define HAL_I2C_SCL_PIN CONFIG_MPU6050_I2C_SCL
#else
#define HAL_I2C_SCL_PIN 11
#endif

#ifdef CONFIG_MPU6050_I2C_FREQ_HZ
#define HAL_I2C_FREQ_HZ CONFIG_MPU6050_I2C_FREQ_HZ
#else
#define HAL_I2C_FREQ_HZ 400000
#endif

#ifdef CONFIG_MPU6050_I2C_ADDR
#define HAL_MPU6050_I2C_ADDR CONFIG_MPU6050_I2C_ADDR
#else
#define HAL_MPU6050_I2C_ADDR 0x68
#endif

#define HAL_I2C_PORT_NUM       0
#define HAL_I2C_TIMEOUT_MS     100
#define HAL_I2C_GLITCH_CNT     7
#define HAL_I2C_INTERNAL_PULLUP true

#define HAL_I2C_BUS_DEFAULT_CONFIG() \
    {                                \
        .port = HAL_I2C_PORT_NUM,    \
        .sda_pin = HAL_I2C_SDA_PIN,  \
        .scl_pin = HAL_I2C_SCL_PIN,  \
        .freq_hz = HAL_I2C_FREQ_HZ,  \
        .enable_internal_pullup = HAL_I2C_INTERNAL_PULLUP, \
    }

#ifdef __cplusplus
}
#endif
