#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "cJSON.h"
#include "nmea_parser.h" // for gps_fix_t
#include "uart_driver.h"

#define TAG "WIFI_AP_TCP"

#define SERVER_PORT 5000
#define SERVER_BACKLOG 1
#define RECV_BUF_SZ 512
#define STATUS_INTERVAL_MS 1000

// Externals expected from your app (declare these in appropriate modules)
extern gps_fix_t myfix;
extern gps_fix_t uart_fix;
extern float current_az;
extern float current_el;
extern const char *device_id;

esp_err_t wifi_ap_start(const char *ssid, const char *password);
