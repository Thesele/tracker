#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "cJSON.h"
#include "nmea_parser.h" // for gps_fix_t

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

// Local state
static TaskHandle_t tcp_server_task_handle = NULL;

/* Forward declarations */
esp_err_t wifi_ap_start(const char *ssid, const char *password);
static void tcp_server_task(void *arg);

/* --- Helper: start Wi-Fi AP --- */
esp_err_t wifi_ap_start(const char *ssid, const char *password)
{
if (!ssid) return ESP_ERR_INVALID_ARG;
ESP_LOGI(TAG, "Initializing TCP server AP (ssid=%s)", ssid);

// Initialize TCP/IP stack and event loop if not already
esp_netif_init();
esp_event_loop_create_default();

// Create default WiFi AP netif
esp_netif_create_default_wifi_ap();

wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_wifi_init(&cfg));

wifi_config_t ap_config = { .ap = { 0 } };
strncpy((char *)ap_config.ap.ssid, ssid, sizeof(ap_config.ap.ssid)-1);
ap_config.ap.ssid_len = strlen(ssid);
if (password && strlen(password) > 0) {
    strncpy((char *)ap_config.ap.password, password, sizeof(ap_config.ap.password)-1);
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
} else {
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
}
ap_config.ap.max_connection = 1; // single GUI client
ap_config.ap.ssid_hidden = 0;

ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
ESP_ERROR_CHECK(esp_wifi_start());

ESP_LOGI(TAG, "WiFi AP started. Connect to SSID '%s' (pass='%s'). AP IP is 192.168.4.1 by default.", ssid,
         (password && strlen(password)) ? password : "(open)");

// start server task
if (tcp_server_task_handle == NULL) {
    xTaskCreate(tcp_server_task, "tcp_server_task", 8 * 1024, NULL, 5, &tcp_server_task_handle);
}

return ESP_OK;
}





/* --- Helper: send status JSON over socket (one line, '\n' terminated) --- */
static void send_status_json(int client_sock)
{
    if (client_sock < 0) return;
    cJSON *root = cJSON_CreateObject();
if (!root) return;

cJSON_AddStringToObject(root, "device_id", device_id ? device_id : "tracker");

cJSON *j_myfix = cJSON_CreateObject();
cJSON_AddBoolToObject(j_myfix, "valid", myfix.valid);
cJSON_AddNumberToObject(j_myfix, "lat", myfix.lat);
cJSON_AddNumberToObject(j_myfix, "lon", myfix.lon);
cJSON_AddNumberToObject(j_myfix, "alt", myfix.alt);
cJSON_AddNumberToObject(j_myfix, "sats", myfix.sats);
cJSON_AddItemToObject(root, "myfix", j_myfix);

cJSON *j_uart = cJSON_CreateObject();
cJSON_AddBoolToObject(j_uart, "valid", uart_fix.valid);
cJSON_AddNumberToObject(j_uart, "lat", uart_fix.lat);
cJSON_AddNumberToObject(j_uart, "lon", uart_fix.lon);
cJSON_AddNumberToObject(j_uart, "alt", uart_fix.alt);
cJSON_AddItemToObject(root, "uart_fix", j_uart);

cJSON_AddNumberToObject(root, "az", current_az);
cJSON_AddNumberToObject(root, "el", current_el);

cJSON_AddNumberToObject(root, "uptime_s", xTaskGetTickCount() / configTICK_RATE_HZ);

char *s = cJSON_PrintUnformatted(root);
if (s) {
    // send as a line
    size_t len = strlen(s);
    char *buf = malloc(len + 2);
    if (buf) {
        memcpy(buf, s, len);
        buf[len] = '\n';
        buf[len+1] = '\0';
        ssize_t sent = send(client_sock, buf, len + 1, 0);
        if (sent < 0) {
            ESP_LOGW(TAG, "Failed to send status: errno=%d", errno);
        }
        free(buf);
    }
    free(s);
}
cJSON_Delete(root);
}



/* --- Helper: send ack JSON for a command --- */
static void send_ack(int client_sock, const char *cmd, const char *status, const char *msg)
{
if (client_sock < 0) return;
cJSON *ack = cJSON_CreateObject();
cJSON_AddStringToObject(ack, "cmd", cmd ? cmd : "");
cJSON_AddStringToObject(ack, "status", status ? status : "ERROR");
if (msg) cJSON_AddStringToObject(ack, "msg", msg);
char *s = cJSON_PrintUnformatted(ack);
if (s) {
size_t len = strlen(s);
char *buf = malloc(len + 2);
if (buf) {
memcpy(buf, s, len);
buf[len] = '\n';
send(client_sock, buf, len + 1, 0);
free(buf);
}
free(s);
}
cJSON_Delete(ack);
}

/* --- Helper: handle command JSON (string length len) --- */
static void handle_command_json(int client_sock, const char *json_str, size_t len)
{
    if (!json_str || len == 0) return;
    cJSON *root = cJSON_ParseWithLength(json_str, len);
if (!root) {
    send_ack(client_sock, "", "ERROR", "invalid_json");
    return;
}

cJSON *cmd = cJSON_GetObjectItem(root, "command");
if (!cJSON_IsString(cmd)) {
    send_ack(client_sock, "", "ERROR", "missing_command");
    cJSON_Delete(root);
    return;
}

const char *cmdstr = cmd->valuestring;
if (strcmp(cmdstr, "set_manual_azel") == 0) {
    cJSON *jaz = cJSON_GetObjectItem(root, "az");
    cJSON *jel = cJSON_GetObjectItem(root, "el");
    if (cJSON_IsNumber(jaz) && cJSON_IsNumber(jel)) {
        float az = (float)jaz->valuedouble;
        float el = (float)jel->valuedouble;
        // validate ranges
        if (az < 0 || az >= 360 || el < -90 || el > 90) {
            send_ack(client_sock, cmdstr, "ERROR", "out_of_range");
        } else {
            // apply directly for now (recommended: enqueue to control task)
            current_az = az;
            current_el = el;
            send_ack(client_sock, cmdstr, "OK", "az_el_set");
        }
    } else {
        send_ack(client_sock, cmdstr, "ERROR", "Missing az/el");
    }
} else if (strcmp(cmdstr, "set_manual_latlon") == 0) {
    cJSON *jlat = cJSON_GetObjectItem(root, "lat");
    cJSON *jlon = cJSON_GetObjectItem(root, "lon");
    cJSON *jalt = cJSON_GetObjectItem(root, "alt");
    if (cJSON_IsNumber(jlat) && cJSON_IsNumber(jlon)) {
        double lat = jlat->valuedouble;
        double lon = jlon->valuedouble;
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            send_ack(client_sock, cmdstr, "ERROR", "latlon_out_of_range");
        } else {
            myfix.lat = lat;
            myfix.lon = lon;
            if (cJSON_IsNumber(jalt)) myfix.alt = jalt->valuedouble;
            send_ack(client_sock, cmdstr, "OK", "latlon_set");
        }
    } else {
        send_ack(client_sock, cmdstr, "ERROR", "missing lat/lon");
    }
} else if (strcmp(cmdstr, "get_status") == 0) {
    send_ack(client_sock, cmdstr, "OK", "status_sent");
    // status will be sent by main loop soon
} else {
    send_ack(client_sock, cmdstr, "ERROR", "unknown_command");
}

cJSON_Delete(root);
}

/* --- TCP server task --- */
/* --- TCP server task: accepts one client, handles lines --- */
static void tcp_server_task(void *arg)
{
    int server_sock = -1;
    struct sockaddr_in server_addr;
    server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
if (server_sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno=%d", errno);
    vTaskDelete(NULL);
    return;
}

int opt = 1;
setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

server_addr.sin_family = AF_INET;
server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 0.0.0.0
server_addr.sin_port = htons(SERVER_PORT);

if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Socket bind failed: errno=%d", errno);
    close(server_sock);
    vTaskDelete(NULL);
    return;
}

if (listen(server_sock, SERVER_BACKLOG) < 0) {
    ESP_LOGE(TAG, "Socket listen failed: errno=%d", errno);
    close(server_sock);
    vTaskDelete(NULL);
    return;
}

ESP_LOGI(TAG, "TCP server listening on port %d", SERVER_PORT);

while (1) {
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    ESP_LOGI(TAG, "Waiting for TCP client...");
    int client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);
    if (client_sock < 0) {
        ESP_LOGW(TAG, "Unable to accept connection: errno=%d", errno);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
    }

    ESP_LOGI(TAG, "Client connected: %s:%d", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    // Set recv timeout so recv returns periodically
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 }; // 200 ms
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Accumulator for partial incoming JSON lines
    char recv_accum[2048];
    size_t recv_accum_len = 0;

    TickType_t last_status_tick = xTaskGetTickCount();

    // Client loop
    while (1) {
        char rbuf[RECV_BUF_SZ];
        ssize_t r = recv(client_sock, rbuf, sizeof(rbuf) - 1, 0);
        if (r > 0) {
            // append to accumulator (be careful not to overflow)
            size_t to_copy = (size_t)r;
            if (recv_accum_len + to_copy >= sizeof(recv_accum)) {
                ESP_LOGW(TAG, "Command accumulator overflow; reset");
                recv_accum_len = 0;
            }
            memcpy(recv_accum + recv_accum_len, rbuf, to_copy);
            recv_accum_len += to_copy;

            // Look for newline-terminated lines
            size_t scan_pos = 0;
            while (scan_pos < recv_accum_len) {
                // find '\n'
                char *nl = memchr(recv_accum + scan_pos, '\n', recv_accum_len - scan_pos);
                if (!nl) break;
                size_t line_len = (nl - (recv_accum + scan_pos));
                if (line_len > 0) {
                    // process single JSON command line
                    // trim any '\r' at end
                    if (recv_accum[scan_pos + line_len - 1] == '\r') line_len--;
                    handle_command_json(client_sock, recv_accum + scan_pos, line_len);
                }
                scan_pos += (line_len + 1); // move past '\n'
            }

            // shift leftover
            if (scan_pos < recv_accum_len) {
                memmove(recv_accum, recv_accum + scan_pos, recv_accum_len - scan_pos);
                recv_accum_len -= scan_pos;
            } else {
                recv_accum_len = 0;
            }
        } else if (r == 0) {
            ESP_LOGI(TAG, "Client disconnected gracefully");
            break;
        } else {
            // r < 0: timeout or error
            int err = errno;
            if (err == EWOULDBLOCK || err == EAGAIN || err == EINTR) {
                // no data right now, continue to status sending
            } else {
                ESP_LOGW(TAG, "recv error: errno=%d", err);
                break;
            }
        }

        // Periodically send status JSON
        if ((xTaskGetTickCount() - last_status_tick) >= pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {
            send_status_json(client_sock);
            last_status_tick = xTaskGetTickCount();
        }

        // small yield
        vTaskDelay(pdMS_TO_TICKS(10));
    } // end client loop

    close(client_sock);
    ESP_LOGI(TAG, "Client handler finished, waiting for new client");
} // end accept loop

// cleanup (never reached here normally)
close(server_sock);
vTaskDelete(NULL);
}