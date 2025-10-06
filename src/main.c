#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "i2c_gps.h" //for the adafruit gps over i2c
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

#include "nmea_parser.h"
#include "uart_driver.h"

extern gps_fix_t received_fix; 
 gps_fix_t myfix;

// i2c definitions
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO           GPIO_NUM_22          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21          /*!< gpio number for I2C*/
#define I2C_MASTER_FREQ_HZ          100000     /*!< I2C master clock frequency */



static void gps_task(void *arg)
{
    static const char *TAG = "GPS_TASK";
    char sentence[GPS_SENTENCE_MAX_LEN] = {0};

    ESP_LOGI(TAG, "GPS task started, waiting for fix...");

    while (1) {
        esp_err_t ret = gps_i2c_read_sentence(sentence, sizeof(sentence), GPS_READ_TIMEOUT_MS);
        if (ret == ESP_OK) {
            // Process the NMEA sentence (your parser handles this)
            ESP_LOGI(TAG, "NMEA: %s", sentence);

            // Example: check if GGA sentence has valid fix
            if (nmea_parser_parse_sentence(sentence, &myfix) == ESP_OK) {
                if (myfix.valid) {
                    ESP_LOGI(TAG, "Got fix: Lat=%.6f, Lon=%.6f, Alt=%.2f, Sats=%d",
                             myfix.lat, myfix.lon, myfix.alt, myfix.sats);
                } else {
                    ESP_LOGI(TAG, "No valid fix yet.");
                }
            }
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Error reading GPS: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // small delay to avoid hammering I2C bus
    }
}

void app_main() {
//  initialize peripherals
// Initialize I2C
    ESP_ERROR_CHECK(gps_i2c_init_full(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ ));
// Configure GPS once at setup 


    ESP_ERROR_CHECK(gps_set_update_rate(100)); // Set update rate to 1 second
    ESP_ERROR_CHECK(gps_enable_rmc_gga()); // Enable RMC and GGA sentences
    
    vTaskDelay(pdMS_TO_TICKS(4000)); // Wait for GPS to initialize
    

// Initialize UART
   uart_driver_init();

    //run tasks
   uart_driver_start_rx_task();
   xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    
  
}