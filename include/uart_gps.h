#ifndef UART_GPS_H
#define UART_GPS_H

#include "esp_err.h"
#include <stdint.h>

// Define the max length for a GPS command
#define GPS_CMD_MAX_LEN 100

/**
 * @brief Configures the GPS module with default settings over UART.
 * * This function sends commands to the GPS to set the update rate
 * and enable RMC + GGA sentences.
 * Assumes uart_driver_init() has already been called.
 */
esp_err_t gps_uart_init(void);

// --- PMTK Helper Functions ---

/**
 * @brief Set the GPS update rate.
 * @param interval_ms Interval in milliseconds (e.g., 1000 for 1Hz, 200 for 5Hz)
 */
esp_err_t gps_set_update_rate(uint32_t interval_ms);

/**
 * @brief Enable only RMC and GGA NMEA sentences (common configuration).
 */
esp_err_t gps_enable_rmc_gga(void);

/**
 * @brief Enable all NMEA sentences.
 */
esp_err_t gps_enable_all_sentences(void);

/**
 * @brief Send a Hot Start command to the GPS.
 */
esp_err_t gps_hot_start(void);

/**
 * @brief Send a Cold Start command (clears all data).
 */
esp_err_t gps_cold_start(void);

/**
 * @brief Query the GPS for its firmware version.
 */
esp_err_t gps_query_fw(void);

#endif // UART_GPS_H
