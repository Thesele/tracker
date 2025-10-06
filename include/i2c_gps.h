#ifndef GPS_I2C_H
#define GPS_I2C_H

#include "driver/i2c_master.h"
#include "esp_err.h"

// Default Adafruit Ultimate GPS I²C address
#define GPS_I2C_ADDR  0x10

// Max buffer sizes
#define GPS_SENTENCE_MAX_LEN  128
#define GPS_CMD_MAX_LEN       64
#define GPS_READ_TIMEOUT_MS   500

// GPS device handle
extern i2c_master_dev_handle_t gps_dev;

// Initialize GPS I²C master bus and device
esp_err_t gps_i2c_init_full(i2c_port_t i2c_port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t freq);

// Read one NMEA sentence into buffer (blocking until something is available)
esp_err_t gps_i2c_read_sentence(char *out_sentence, size_t max_len, uint32_t timeout_ms);

// PMTK command helpers
esp_err_t gps_send_cmd(const char *cmd);
esp_err_t gps_set_update_rate(uint32_t interval_ms);
esp_err_t gps_enable_rmc_gga(void);
esp_err_t gps_enable_all_sentences(void);
esp_err_t gps_hot_start(void);
esp_err_t gps_cold_start(void);
esp_err_t gps_query_fw(void);

#endif // GPS_I2C_H
