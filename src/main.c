#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "i2c_gps.h" //for the adafruit gps over i2c
#include "gps_task.h"
#include "nmea_parser.h"
#include "uart_driver.h"
#include "wifi_commands.h" //for the wifi ap and tcp server
#include "control_task.h" //for the control task
#include "servo_control.h" //for the servo control task

extern gps_fix_t received_fix; 
 gps_fix_t myfix;
 const char *device_id = "SKRIPSIE_TRACKER_001"; // set your device id here
 float current_az = 0.0;
 float current_el = 0.0;

// i2c definitions
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO           GPIO_NUM_22          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21          /*!< gpio number for I2C*/
#define I2C_MASTER_FREQ_HZ          100000     /*!< I2C master clock frequency */

// SERVO PINS
#define SERVO_AZ_PIN                GPIO_NUM_18
#define SERVO_EL_PIN                GPIO_NUM_19


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

                 // Initialize control task
    ESP_ERROR_CHECK(control_init());

                 // initilize servo control
    servo_config_t cfg[SERVO_COUNT] = {
        [SERVO_AZ] = {
            .min_pulse_us = 500,
            .neutral_pulse_us = 1500,
            .max_pulse_us = 2500,
            .min_deg = -135.0f,
            .max_deg = 135.0f,
            .pwm_pin = SERVO_AZ_PIN ,
            .fb_channel = ADC_CHANNEL_6 // GPIO34
        },
        [SERVO_EL] = {
            .min_pulse_us = 500,
            .neutral_pulse_us = 1500,
            .max_pulse_us = 2500,
            .min_deg = -135.0f,
            .max_deg = 135.0f,
            .pwm_pin = SERVO_EL_PIN ,
            .fb_channel = ADC_CHANNEL_7 // GPIO35
        }
    };


// adc_channel_t fb_channels[SERVO_COUNT] = { ADC_CHANNEL_6, ADC_CHANNEL_7 }; // GPIO34, GPIO35

    servo_control_init(cfg);

    servo_set_angle(SERVO_AZ, 0.0);
    servo_set_angle(SERVO_EL, 0.0);



    //run tasks
    uart_driver_start_rx_task();
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    wifi_ap_start("TrackerAP", "12345678"); //ssid and password  //initialize wifi ap and tcp server
    ESP_ERROR_CHECK(control_start());

    
  
}