#include "i2c_gps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "GPS_I2C";

i2c_master_dev_handle_t gps_dev = NULL;

// -------------------- INIT --------------------
esp_err_t gps_i2c_init_full(i2c_port_t i2c_port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t freq)
{
    // 1. Configure master bus
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = 1 }
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus");
        return ret;
    }

    // 2. Add GPS as a device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = GPS_I2C_ADDR,
        .scl_speed_hz    = freq
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &gps_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPS device to bus");
        return ret;
    }

    ESP_LOGI(TAG, "GPS I2C initialized successfully");
    return ESP_OK;
}
// -------------------- READ --------------------
esp_err_t gps_i2c_read_sentence(char *out_sentence, size_t max_len, uint32_t timeout_ms)
{
if (!gps_dev || !out_sentence) return ESP_ERR_INVALID_ARG;

uint8_t buf[GPS_SENTENCE_MAX_LEN];
memset(buf, 0, sizeof(buf));

// Try to read up to max_len bytes
esp_err_t ret = i2c_master_receive(gps_dev, buf, max_len, pdMS_TO_TICKS(timeout_ms));
if (ret != ESP_OK) return ret;

// Copy into caller buffer
strncpy(out_sentence, (char *)buf, max_len - 1);
out_sentence[max_len - 1] = '\0';

// ESP_LOGI(TAG, "Received NMEA: %s", out_sentence);
return ESP_OK;

}

// -------------------- WRITE --------------------
esp_err_t gps_send_cmd(const char *cmd)
{
if (!gps_dev || !cmd) return ESP_ERR_INVALID_ARG;
size_t len = strlen(cmd);

ESP_LOGI(TAG, "Sending PMTK: %s", cmd);
return i2c_master_transmit(gps_dev, (const uint8_t*)cmd, len, pdMS_TO_TICKS(100));

}

// -------------------- PMTK HELPERS --------------------
// Update rate: interval in ms (1000 = 1Hz, 200 = 5Hz, 100 = 10Hz)
esp_err_t gps_set_update_rate(uint32_t interval_ms)
{
char cmd[GPS_CMD_MAX_LEN];
snprintf(cmd, sizeof(cmd), "$PMTK220,%u*1F\r\n", (unsigned int)interval_ms);
return gps_send_cmd(cmd);
}

// Enable only RMC + GGA (typical)
esp_err_t gps_enable_rmc_gga(void)
{
return gps_send_cmd("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
}

// Enable all sentences
esp_err_t gps_enable_all_sentences(void)
{
return gps_send_cmd("$PMTK314,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1*2C\r\n");
}

// Hot start (reuse almanac/ephemeris)
esp_err_t gps_hot_start(void)
{
return gps_send_cmd("$PMTK101*32\r\n");
}

// Cold start (clear all data, full re-acquire)
esp_err_t gps_cold_start(void)
{
return gps_send_cmd("$PMTK103*30\r\n");
}

// Query firmware release
esp_err_t gps_query_fw(void)
{
return gps_send_cmd("$PMTK605*31\r\n");
}
