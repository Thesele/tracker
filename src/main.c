#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "i2c_gps.h" //for the adafruit gps over i2c
#include "gps_task.h"
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


void app_main() {
//  initialize peripherals
// Initialize I2C
    ESP_ERROR_CHECK(gps_i2c_init_full(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ ));
// Configure GPS once at setup 


    ESP_ERROR_CHECK(gps_set_update_rate(3000)); // Set update rate to 1 second
    ESP_ERROR_CHECK(gps_enable_rmc_gga()); // Enable RMC and GGA sentences
    
    vTaskDelay(pdMS_TO_TICKS(4000)); // Wait for GPS to initialize
    

// Initialize UART
   uart_driver_init();

    //run tasks
   uart_driver_start_rx_task();
   xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    
  
}