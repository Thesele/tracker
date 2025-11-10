#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h" // Standard UART driver
#include "esp_log.h"
#include "nmea_parser.h"
#include "control_task.h" 
#include "uart_gps.h"
static const char *TAG = "UART_DRIVER";

// Define UART buffer sizes
#define UART_BUF_SIZE 1024
// Default baud rate, can be overridden by specific init logic if needed
#define UART_BAUD_RATE 115200 
// GPS Baud rate (ADAFUIT GPS default) weird how they don't have proper datasheet for their stuff
#define GPS_BAUD_RATE 9600 

// Parameters for the UART event task
typedef struct {
    uart_port_t port;
    QueueHandle_t event_queue;
} uart_task_param_t;

// --- Pin Definitions ---
// UART0 (Default Monitor/ESP-IDF Log)
#define UART0_TX_PIN   GPIO_NUM_1
#define UART0_RX_PIN   GPIO_NUM_3
// UART1 (Field/Target GPS)
#define UART1_TX_PIN   GPIO_NUM_17
#define UART1_RX_PIN   GPIO_NUM_16
// UART2 (Main/My GPS)
#define UART2_TX_PIN   GPIO_NUM_22 // Using GPIO 22 for TX
#define UART2_RX_PIN   GPIO_NUM_23 // Using GPIO 23 for RX


// --- Fix Storage ---
gps_fix_t uart_fix_0; // For UART0
gps_fix_t uart_fix_1; // For UART1
gps_fix_t uart_fix_2; // For UART2 (Our main GPS)



// --- Queues ---
static QueueHandle_t uart0_queue, uart1_queue, uart2_queue;


// ========================= RX TASK =========================
/**
 * @brief Common UART RX Task for all ports
 * * This task waits for UART events and processes incoming data.
 * - UART0/1 data is treated as "target_fix"
 * - UART2 data is treated as "my_fix"
 * * NOTE: This parsing logic assumes that one UART_DATA event contains one
 * complete NMEA or custom sentence. This might be unreliable if sentences
 * are split across buffer flushes.
 */
static void uart_rx_task(void *arg) {
     uint8_t my_fix_counter = 0;

    uart_task_param_t *params = (uart_task_param_t *)arg;
    uart_event_t event;
    // Use a larger buffer to hold potential NMEA sentences
    uint8_t *uartData = (uint8_t *)malloc(UART_BUF_SIZE);
    if (uartData == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for UART RX buffer on UART%d", params->port);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "UART RX Task started for UART%d", params->port);

    for (;;) {
        // Wait forever for the next event
        if (xQueueReceive(params->event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                // Event of UART receiving data
                case UART_DATA: {
                    memset(uartData, 0, UART_BUF_SIZE);
                    int len = uart_read_bytes(params->port, uartData, event.size, pdMS_TO_TICKS(100));
                    ESP_LOGI(TAG, "UART%d Received %d bytes", params->port, len);

                    if (len > 0) {
                        // Log the raw string data for debugging
                                                           // ESP_LOG_BUFFER_HEXDUMP(TAG, uartData, len, ESP_LOG_DEBUG);
                        
                        // Check for NMEA sentence
                        if (uartData[0] == '$') {
                            gps_fix_t *fix_ref = NULL;

                            // Assign the correct global fix structure
                            if (params->port == UART_NUM_0) {
                                fix_ref = &uart_fix_0;
                            } else if (params->port == UART_NUM_1) {
                                fix_ref = &uart_fix_1;
                            } else if (params->port == UART_NUM_2) {
                                fix_ref = &uart_fix_2;
                            }

                            if (fix_ref) {
                                if (nmea_parser_parse_sentence((char*)uartData, fix_ref) == ESP_OK) {
                                    
                                    // === LOGIC FOR OUR MAIN GPS (UART2) ===
                                    if (params->port == UART_NUM_2) {
                                        if (fix_ref->valid) {
                                            ESP_LOGI("GPS_MY_FIX", "UART%d: Lat=%.6f, Lon=%.6f, Alt=%.2f, Fix=%d, Sats=%d",
                                                     params->port, fix_ref->lat, fix_ref->lon,
                                                     fix_ref->alt, fix_ref->fix_quality, fix_ref->sats);
                                            control_set_my_fix(fix_ref); // Update shared state with my location
                                            my_fix_counter == 10? vTaskDelete(NULL): my_fix_counter++ ; // Stop task after getting a valid fix 10 times
                                        } else {
                                            ESP_LOGW("GPS_MY_FIX", "UART%d: NMEA parsed but fix is not valid.", params->port);
                                        }
                                    } 
                                    // === LOGIC FOR TARGET GPS (UART0/1) ===
                                    else {
                                        ESP_LOGI("GPS_TARGET", "UART%d: Lat=%.6f, Lon=%.6f, Alt=%.2f, Fix=%d, Sats=%d",
                                                 params->port, fix_ref->lat, fix_ref->lon,
                                                 fix_ref->alt, fix_ref->fix_quality, fix_ref->sats);
                                        
                                        control_set_mode(TRACK_MODE_GPS);
                                        control_set_target_fix(fix_ref);
                                    }
                                } else {
                                     ESP_LOGW(TAG, "UART%d: NMEA sentence parse failed.", params->port);
                                }
                            }
                        } 
                        // Check for Custom sentence
                        else if (uartData[0] == '#') {
                            gps_fix_t custom_fix;
                            if (parse_custom_gps_fix((char*)uartData, &custom_fix)) { // Assuming this function exists
                                ESP_LOGI("GPS_CUSTOM", "UART%d CUSTOM: Lat=%.6f, Lon=%.6f, Alt=%.2f",
                                         params->port, custom_fix.lat, custom_fix.lon, custom_fix.alt);
                                
                                // Assume custom fixes are always for the target
                                control_set_mode(TRACK_MODE_GPS);
                                control_set_target_fix(&custom_fix);
                            } else {
                                ESP_LOGW(TAG, "UART%d: Failed to parse custom GPS fix", params->port);
                            }
                        }
                    }
                    break;
                }
                
                // Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART%d: HW FIFO Overflow", params->port);
                    uart_flush_input(params->port);
                    xQueueReset(params->event_queue);
                    break;
                
                // Event of Ring Buffer full detected
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART%d: Ring Buffer Full", params->port);
                    uart_flush_input(params->port);
                    xQueueReset(params->event_queue);
                    break;
                
                // Others
                default:
                    ESP_LOGD(TAG, "UART%d event type: %d", params->port, event.type);
                    break;
            }
        }
    }
    
    free(uartData);
    vTaskDelete(NULL);
}

// ========================= INITIALIZATION =========================

/**
 * @brief Helper to initialize a single UART port
 */
static esp_err_t uart_init_port(uart_port_t port, int tx_pin, int rx_pin, int baud_rate, QueueHandle_t *event_queue) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(port, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Initialized UART%d (TX=%d, RX=%d, Baud=%d)", port, tx_pin, rx_pin, baud_rate);
    return ESP_OK;
}

/**
 * @brief Main UART driver initialization
 * * Initializes UART0, UART1, and UART2 and spawns RX tasks for each.
 */
esp_err_t uart_driver_init(void) {
    
    // Initialize UART0, UART1, and UART2
    uart_init_port(UART_NUM_0, UART0_TX_PIN, UART0_RX_PIN, UART_BAUD_RATE, &uart0_queue);
    uart_init_port(UART_NUM_1, UART1_TX_PIN, UART1_RX_PIN, UART_BAUD_RATE, &uart1_queue);
    uart_init_port(UART_NUM_2, UART2_TX_PIN, UART2_RX_PIN, GPS_BAUD_RATE,  &uart2_queue); // Use GPS_BAUD_RATE

        // Configure GPS once at setup 
    ESP_ERROR_CHECK(gps_set_update_rate(3000)); // Set update rate to 3 second
    ESP_ERROR_CHECK(gps_enable_rmc_gga()); // Enable RMC and GGA sentences

    // Spawn RX tasks
    uart_task_param_t *param0 = malloc(sizeof(uart_task_param_t));
    *param0 = (uart_task_param_t){ .port = UART_NUM_0, .event_queue = uart0_queue };
    xTaskCreate(uart_rx_task, "uart0_rx_task", 4096, param0, 5, NULL);

    uart_task_param_t *param1 = malloc(sizeof(uart_task_param_t));
    *param1 = (uart_task_param_t){ .port = UART_NUM_1, .event_queue = uart1_queue };
    xTaskCreate(uart_rx_task, "uart1_rx_task", 4096, param1, 5, NULL);

    uart_task_param_t *param2 = malloc(sizeof(uart_task_param_t));
    *param2 = (uart_task_param_t){ .port = UART_NUM_2, .event_queue = uart2_queue };
    xTaskCreate(uart_rx_task, "uart2_rx_task", 4096, param2, 5, NULL); // Task the GPS

    return ESP_OK;
}

// ========================= TRANSMIT FUNCTION =========================
/**
 * @brief Transmit data over a specified UART port
 */
esp_err_t uart_driver_send(uart_port_t port, const char *data) {
    int len = strlen(data);
    int txBytes = uart_write_bytes(port, data, len);
    if (txBytes == len) {
        ESP_LOGD(TAG, "UART%d Wrote %d bytes: %s", port, txBytes, data);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "UART%d Error writing. Wrote %d of %d bytes.", port, txBytes, len);
        return ESP_FAIL;
    }
}
