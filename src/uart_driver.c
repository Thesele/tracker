#include "uart_driver.h"
#include "esp_log.h"
#include "nmea_parser.h"
#include <string.h>
#include "control_task.h" // to update myfix when a new fix is received over uart

gps_fix_t uart_fix; // to hold the received GPS fix. this is defined in the uart driver.c

static const char *TAG = "UART_DRIVER";
static QueueHandle_t uart_event_queue;

// RX task: waits on UART events
static void uart_rx_task(void *arg) {
    uart_event_t event;
    char uartData[100] = {0};

    for (;;) {
        // Wait indefinitely for UART events
        if (xQueueReceive(uart_event_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: // if the event is data received
                    memset(uartData, 0, sizeof(uartData)); // clear the buffer
                    int len = uart_read_bytes(UART_PORT_NUM, uartData, event.size, portMAX_DELAY); // reads received data
                    ESP_LOGI(TAG, "Received %d bytes: '%.*s'", event.size, event.size, uartData);
                    // char* sumChecker = malloc(len );
                    // strncpy(sumChecker, uartData, len);

                    if (len > 0) {
                        if(uartData[0] == '$') {
                            // Process the received data starting with '$' ( the nmea sentence )
                            ESP_LOGI(TAG, "Processing NMEA SENTENCE");

                            if (nmea_parser_parse_sentence(uartData, &uart_fix) == ESP_OK) {
                                ESP_LOGI("GPS", "Lat=%.6f, Lon=%.6f, Alt=%.2f, Fix=%d, Sats=%d",
                               uart_fix.lat,uart_fix.lon,uart_fix.alt,uart_fix.fix_quality,uart_fix.sats);

                                 control_set_my_fix(&uart_fix); // Update shared state with my location

                            }

                        } else if(uartData[0] == '#') {
                            ESP_LOGI(TAG, "Processing CUSTOM SENTENCE");
                        }
                    }
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "HW FIFO Overflow");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring Buffer Full");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                default:
                    ESP_LOGD(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

esp_err_t uart_driver_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

esp_err_t uart_driver_send(const char *data) {
    int len = strlen(data);
    int txBytes = uart_write_bytes(UART_PORT_NUM, data, len);
    return (txBytes == len) ? ESP_OK : ESP_FAIL;
}

void uart_driver_start_rx_task(void) {
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
}
