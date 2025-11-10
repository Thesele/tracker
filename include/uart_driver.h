#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "nmea_parser.h"

// ========================= CONFIGURATION =========================

// Default baud rate for both UARTs
#define UART_BAUD_RATE       115200
#define UART_BUF_SIZE        (1024)

// UART0 (Testing Interface)
#define UART0_TX_PIN         (GPIO_NUM_1)   // Default TX0
#define UART0_RX_PIN         (GPIO_NUM_3)   // Default RX0

// UART1 (Field Interface)
#define UART1_TX_PIN         (GPIO_NUM_17)
#define UART1_RX_PIN         (GPIO_NUM_16)

// ========================= GLOBAL FIX VARIABLES =========================
extern gps_fix_t uart_fix_0;  // Data received over UART0 (testing)
extern gps_fix_t uart_fix_1;  // Data received over UART1 (field use)

// ========================= FUNCTION PROTOTYPES =========================

// Initialize both UART0 and UART1, spawn RX tasks for each
esp_err_t uart_driver_init(void);

// Send data through a specified UART port (UART_NUM_0 or UART_NUM_1)
esp_err_t uart_driver_send(uart_port_t port, const char *data);

#endif // UART_DRIVER_H
