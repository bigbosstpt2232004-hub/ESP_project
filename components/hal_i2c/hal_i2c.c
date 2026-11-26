#include "hal_i2c.h"
#include "hal_i2c_config.h"

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hal_i2c";

struct hal_i2c_bus_s {
    i2c_master_bus_handle_t idf_bus;
};

struct hal_i2c_dev_s {
    i2c_master_dev_handle_t idf_dev;
};

void hal_i2c_check_lines(int sda_pin, int scl_pin, int *out_sda_level, int *out_scl_level)
{
    int pins[2] = {sda_pin, scl_pin};
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(pins[i], GPIO_PULLUP_ONLY);
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    int sda = gpio_get_level(sda_pin);
    int scl = gpio_get_level(scl_pin);
    if (out_sda_level) *out_sda_level = sda;
    if (out_scl_level) *out_scl_level = scl;

    ESP_LOGI(TAG, "Line check: SDA(GPIO%d)=%d, SCL(GPIO%d)=%d", sda_pin, sda, scl_pin, scl);
}

esp_err_t hal_i2c_bus_init(const hal_i2c_bus_config_t *config, hal_i2c_bus_handle_t *out_bus)
{
    ESP_RETURN_ON_FALSE(config && out_bus, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    struct hal_i2c_bus_s *bus = calloc(1, sizeof(struct hal_i2c_bus_s));
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_NO_MEM, TAG, "Out of memory");

    i2c_master_bus_config_t idf_cfg = {
        .i2c_port = (i2c_port_num_t)config->port,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = HAL_I2C_GLITCH_CNT,
        .flags.enable_internal_pullup = config->enable_internal_pullup,
    };

    esp_err_t ret = i2c_new_master_bus(&idf_cfg, &bus->idf_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        free(bus);
        return ret;
    }

    *out_bus = bus;
    return ESP_OK;
}

esp_err_t hal_i2c_bus_deinit(hal_i2c_bus_handle_t bus)
{
    if (!bus) return ESP_OK;
    esp_err_t ret = ESP_OK;
    if (bus->idf_bus) {
        ret = i2c_del_master_bus(bus->idf_bus);
    }
    free(bus);
    return ret;
}

esp_err_t hal_i2c_probe(hal_i2c_bus_handle_t bus, uint8_t dev_addr, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(bus && bus->idf_bus, ESP_ERR_INVALID_ARG, TAG, "Invalid bus handle");
    return i2c_master_probe(bus->idf_bus, dev_addr, timeout_ms);
}

esp_err_t hal_i2c_add_device(hal_i2c_bus_handle_t bus, uint8_t dev_addr, uint32_t freq_hz, hal_i2c_dev_handle_t *out_dev)
{
    ESP_RETURN_ON_FALSE(bus && bus->idf_bus && out_dev, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    struct hal_i2c_dev_s *dev = calloc(1, sizeof(struct hal_i2c_dev_s));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Out of memory");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = freq_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus->idf_bus, &dev_cfg, &dev->idf_dev);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    *out_dev = dev;
    return ESP_OK;
}

esp_err_t hal_i2c_remove_device(hal_i2c_dev_handle_t dev)
{
    if (!dev) return ESP_OK;
    esp_err_t ret = ESP_OK;
    if (dev->idf_dev) {
        ret = i2c_master_bus_rm_device(dev->idf_dev);
    }
    free(dev);
    return ret;
}

esp_err_t hal_i2c_write_reg(hal_i2c_dev_handle_t dev, uint8_t reg_addr, uint8_t value, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(dev && dev->idf_dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device handle");
    uint8_t buf[2] = {reg_addr, value};
    return i2c_master_transmit(dev->idf_dev, buf, sizeof(buf), timeout_ms);
}

esp_err_t hal_i2c_read_regs(hal_i2c_dev_handle_t dev, uint8_t reg_addr, uint8_t *data, size_t len, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(dev && dev->idf_dev && data && len > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    return i2c_master_transmit_receive(dev->idf_dev, &reg_addr, 1, data, len, timeout_ms);
}
