#include "uart_gps.h"
#include "uart_driver.h" // Assumed to define uart_driver_send
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "UART_GPS";

// Define which UART port the main GPS is on
#define GPS_UART_PORT UART_NUM_2

/**
 * @brief Calculates and appends the NMEA checksum to a command string.
 * @param s The command string, must start with '$' and have a '*' at the end.
 */
static void set_checksum(char *s)
{
    char appnd[6] = {0}; // Room for "*", 2-char checksum, "\r\n", and null
    if (!s || *s != '$') return;

    const char *p = s + 1; // Skip the '$'
    uint8_t cs = 0;

    // Calculate checksum up to the '*'
    while (*p && *p != '*') {
        cs ^= (uint8_t)(*p++);
    }

    // Append the checksum and CRLF
    snprintf(appnd, sizeof(appnd), "%02X\r\n", cs);
    strcat(s, appnd); // Append "XX\r\n"
}

/**
 * @brief Internal function to send a command to the GPS UART port.
 */
static esp_err_t gps_send_cmd(const char *cmd)
{
    ESP_LOGI(TAG, "Sending PMTK to UART%d: %s", GPS_UART_PORT, cmd);
    // Use the generic uart_driver_send function, ensuring it's targeting the GPS port
    return uart_driver_send(GPS_UART_PORT, cmd);
}

// -------------------- INIT --------------------
/**
 * @brief Initialize the GPS module with standard settings.
 */
esp_err_t gps_uart_init(void)
{
    ESP_LOGI(TAG, "Configuring GPS module on UART%d...", GPS_UART_PORT);
    
    // Give the GPS a moment to boot up if we just powered on
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    // Set 1Hz update rate
    esp_err_t ret = gps_set_update_rate(1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set update rate");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay between commands

    // Enable only RMC and GGA
    ret = gps_enable_rmc_gga();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NMEA sentences");
        return ret;
    }

    ESP_LOGI(TAG, "GPS module configured successfully via UART.");
    return ESP_OK;
}


// -------------------- PMTK HELPERS --------------------

// Update rate: interval in ms (1000 = 1Hz, 200 = 5Hz, 100 = 10Hz)
esp_err_t gps_set_update_rate(uint32_t interval_ms)
{
    char cmd[GPS_CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), "$PMTK220,%u*", (unsigned int)interval_ms);
    set_checksum(cmd); // Appends checksum and \r\n
    return gps_send_cmd(cmd);
}

// Enable only RMC + GGA (typical)
esp_err_t gps_enable_rmc_gga(void)
{
    // Checksum for this command is *28
    return gps_send_cmd("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
}

// Enable all sentences
esp_err_t gps_enable_all_sentences(void)
{
    // Checksum for this command is *2C
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
