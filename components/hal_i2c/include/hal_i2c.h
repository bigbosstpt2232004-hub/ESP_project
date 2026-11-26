#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int port;
    int sda_pin;
    int scl_pin;
    uint32_t freq_hz;
    bool enable_internal_pullup;
} hal_i2c_bus_config_t;

typedef struct hal_i2c_bus_s *hal_i2c_bus_handle_t;
typedef struct hal_i2c_dev_s *hal_i2c_dev_handle_t;

/**
 * @brief Sample idle logic levels of SDA and SCL lines with internal pull-ups enabled.
 */
void hal_i2c_check_lines(int sda_pin, int scl_pin, int *out_sda_level, int *out_scl_level);

/**
 * @brief Initialize an I2C master bus via HAL.
 */
esp_err_t hal_i2c_bus_init(const hal_i2c_bus_config_t *config, hal_i2c_bus_handle_t *out_bus);

/**
 * @brief Deinitialize an I2C master bus.
 */
esp_err_t hal_i2c_bus_deinit(hal_i2c_bus_handle_t bus);

/**
 * @brief Probe a 7-bit I2C slave address on the bus.
 */
esp_err_t hal_i2c_probe(hal_i2c_bus_handle_t bus, uint8_t dev_addr, int timeout_ms);

/**
 * @brief Attach a 7-bit I2C slave device to the HAL bus.
 */
esp_err_t hal_i2c_add_device(hal_i2c_bus_handle_t bus, uint8_t dev_addr, uint32_t freq_hz, hal_i2c_dev_handle_t *out_dev);

/**
 * @brief Detach an I2C slave device from the HAL bus.
 */
esp_err_t hal_i2c_remove_device(hal_i2c_dev_handle_t dev);

/**
 * @brief Write a single byte to a device register.
 */
esp_err_t hal_i2c_write_reg(hal_i2c_dev_handle_t dev, uint8_t reg_addr, uint8_t value, int timeout_ms);

/**
 * @brief Burst-read multiple bytes starting from a device register.
 */
esp_err_t hal_i2c_read_regs(hal_i2c_dev_handle_t dev, uint8_t reg_addr, uint8_t *data, size_t len, int timeout_ms);

#ifdef __cplusplus
}
#endif
