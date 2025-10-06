#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define UART_PORT_NUM      UART_NUM_0  //  this is for testing.... GO BACK TO UART1 BEFORE TESTING
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        17
#define UART_RX_PIN        16
#define UART_BUF_SIZE      (1024)
#define BAUD_RATE         115200

// Public function prototypes
esp_err_t uart_driver_init(void);
esp_err_t uart_driver_send(const char *data);
void uart_driver_start_rx_task(void);

#endif // UART_DRIVER_H