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
#include "uart_gps.h"
#include "nmea_parser.h"
#include "uart_driver.h" // my uart driver for nmea sentence parsing
#include "imu_task.h"
#include "wifi_commands.h" //for the wifi ap and tcp server
#include "control_task.h" //for the control task
#include "servo_control.h" //for the servo control task

extern gps_fix_t received_fix; 
 gps_fix_t myfix;
 const char *device_id = "SKRIPSIE_TRACKER_001"; // set your device id here
 float current_az = 0.0;
 float current_el = 0.0;

// i2c definitions
// #define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
// #define I2C_MASTER_SCL_IO           GPIO_NUM_22          /*!< gpio number for I2C master clock */
// #define I2C_MASTER_SDA_IO           GPIO_NUM_21          /*!< gpio number for I2C*/
// #define I2C_MASTER_FREQ_HZ          100000     /*!< I2C master clock frequency */

// SERVO PINS
#define SERVO_AZ_PIN                GPIO_NUM_18
#define SERVO_EL_PIN                GPIO_NUM_19


void app_main() {
/*
 ***************************************************************************************                                    
                                     initialize peripherals
****************************************************************************************                                     
                                     */
                          
                  // Initialize UART
    uart_driver_init(); //modified to init 3 uarts, including one for gps... I think my i2c line is messed up( n recent overvoltage)
   
                       // Initialize IMU
    imu_task_start();

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
    gps_fix_t pseudo_gps_fix ={.lat = -33.928,.lon = 18.86,.alt =130,.valid = 1,}; //dummy gps
    control_set_my_fix(&pseudo_gps_fix); //set my fix to cape town for initial testing
    

// adc_channel_t fb_channels[SERVO_COUNT] = { ADC_CHANNEL_6, ADC_CHANNEL_7 }; // GPIO34, GPIO35

    servo_control_init(cfg);

    servo_set_angle(SERVO_AZ, 0.0);
    servo_set_angle(SERVO_EL, 0.0);



    //run wifi task
    wifi_ap_start("TrackerAP", "12345678"); //ssid and password  //initializes wifi ap and tcp server
    ESP_ERROR_CHECK(control_start());

    
  
}