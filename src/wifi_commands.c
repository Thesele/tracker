/* esp32_wifi_commands.c
 *
 * UPDATED:
 * - Adds new "imu_data" object to the periodic status JSON.
 * - This object contains heading, roll, pitch, and yaw.
 */

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
#include "nvs_flash.h"

#include "cJSON.h"

#include "control_task.h" // Use new header

#define TAG "WIFI_AP_TCP"

#define SERVER_PORT 5000
#define SERVER_BACKLOG 1
#define RECV_BUF_SZ 512
#define STATUS_INTERVAL_MS 500

/* Externals */
extern const char *device_id; // Assuming this is defined in main

// helper prototypes
static void tcp_server_task(void *arg);
static void send_status_json(int client_sock);
static void send_ack(int client_sock, const char *cmd, const char *status, const char *msg);

esp_err_t wifi_ap_start(const char *ssid, const char *password)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Starting Wi-Fi AP (ssid=%s)...", ssid);

    // Ensure NVS is initialized before wifi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = { .ap = { {0} } };
    strncpy((char *)ap_config.ap.ssid, ssid, sizeof(ap_config.ap.ssid)-1);
    ap_config.ap.ssid_len = strlen(ssid);
    if (password && strlen(password) > 0) {
        strncpy((char *)ap_config.ap.password, password, sizeof(ap_config.ap.password)-1);
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ap_config.ap.max_connection = 1;
    ap_config.ap.ssid_hidden = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Start server task
    xTaskCreate(tcp_server_task, "tcp_server_task", 8*1024, NULL, 5, NULL);

    ESP_LOGI(TAG, "AP started. Connect to SSID='%s' (pass='%s'). AP IP=192.168.4.1", ssid, (password?password:""));
    return ESP_OK;
}

/* --- send status JSON (line terminated) --- */
static void send_status_json(int client_sock)
{
    if (client_sock < 0) return;

    tracker_state_t st;
    if (control_get_state(&st) != ESP_OK) return;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", device_id ? device_id : "tracker");

    cJSON *j_myfix = cJSON_CreateObject();
    cJSON_AddBoolToObject(j_myfix, "valid", st.my_fix.valid);
    cJSON_AddNumberToObject(j_myfix, "lat", st.my_fix.lat);
    cJSON_AddNumberToObject(j_myfix, "lon", st.my_fix.lon);
    cJSON_AddNumberToObject(j_myfix, "alt", st.my_fix.alt);
    cJSON_AddNumberToObject(j_myfix, "sats", st.my_fix.sats);
    cJSON_AddItemToObject(root, "my_fix", j_myfix);

    cJSON *j_target = cJSON_CreateObject();
    cJSON_AddBoolToObject(j_target, "valid", st.target_fix.valid);
    cJSON_AddNumberToObject(j_target, "lat", st.target_fix.lat);
    cJSON_AddNumberToObject(j_target, "lon", st.target_fix.lon);
    cJSON_AddNumberToObject(j_target, "alt", st.target_fix.alt);
    cJSON_AddItemToObject(root, "target_fix", j_target);

    cJSON *j_azel = cJSON_CreateObject();
    cJSON_AddNumberToObject(j_azel, "az", st.current_azel.az);
    cJSON_AddNumberToObject(j_azel, "el", st.current_azel.el);
    cJSON_AddItemToObject(root, "azel", j_azel);

    // --- NEW IMU DATA ---
    cJSON *j_imu = cJSON_CreateObject();
    cJSON_AddBoolToObject(j_imu, "valid", st.imu_valid);
    cJSON_AddNumberToObject(j_imu, "heading", st.heading);
    cJSON_AddNumberToObject(j_imu, "roll", st.euler.roll);
    cJSON_AddNumberToObject(j_imu, "pitch", st.euler.pitch);
    cJSON_AddNumberToObject(j_imu, "yaw", st.euler.yaw);
    cJSON_AddItemToObject(root, "imu_data", j_imu);
    // --- END NEW ---

    cJSON_AddStringToObject(root, "mode", (st.mode == TRACK_MODE_MANUAL) ? "MANUAL" : "GPS");
    cJSON_AddNumberToObject(root, "uptime_s", xTaskGetTickCount() / configTICK_RATE_HZ);

    char *s = cJSON_PrintUnformatted(root);
    if (s) {
        size_t len = strlen(s);
        // send as single line
        char *buf = malloc(len + 2);
        if (buf) {
            memcpy(buf, s, len);
            buf[len] = '\n';
            send(client_sock, buf, len + 1, 0);
            free(buf);
        }
        free(s);
    }
    cJSON_Delete(root);
}

/* --- send ack JSON --- */
static void send_ack(int client_sock, const char *cmd, const char *status, const char *msg)
{
    if (!cmd) cmd = "";
    cJSON *ack = cJSON_CreateObject();
    cJSON_AddStringToObject(ack, "cmd", cmd);
    cJSON_AddStringToObject(ack, "status", status?status:"ERROR");
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

/* --- handle incoming JSON commands --- */
static void handle_command_json(int client_sock, const char *json_str, size_t len)
{
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

    if (strcmp(cmdstr, "SET_MODE") == 0) {
        cJSON *m = cJSON_GetObjectItem(root, "mode");
        if (cJSON_IsString(m)) {
            if (strcasecmp(m->valuestring, "MANUAL") == 0) {
                control_set_mode(TRACK_MODE_MANUAL);
                send_ack(client_sock, cmdstr, "OK", "mode_manual");
            } else {
                control_set_mode(TRACK_MODE_GPS);
                send_ack(client_sock, cmdstr, "OK", "mode_gps");
            }
        } else {
            send_ack(client_sock, cmdstr, "ERROR", "missing_mode");
        }
    }
    else if (strcmp(cmdstr, "SET_AZ_EL") == 0) {
        cJSON *jaz = cJSON_GetObjectItem(root, "az");
        cJSON *jel = cJSON_GetObjectItem(root, "el");
        if (cJSON_IsNumber(jaz) && cJSON_IsNumber(jel)) {
            double az = jaz->valuedouble;
            double el = jel->valuedouble;
            if (control_set_manual_azel(az, el) == ESP_OK) {
                send_ack(client_sock, cmdstr, "OK", "manual_azel_set");
            } else {
                send_ack(client_sock, cmdstr, "ERROR", "failed_set_azel");
            }
        } else {
            send_ack(client_sock, cmdstr, "ERROR", "missing_az_el");
        }
    }
    else if (strcmp(cmdstr, "SET_TARGET") == 0) {
        cJSON *jlat = cJSON_GetObjectItem(root, "lat");
        cJSON *jlon = cJSON_GetObjectItem(root, "lon");
        cJSON *jalt = cJSON_GetObjectItem(root, "alt");
        if (cJSON_IsNumber(jlat) && cJSON_IsNumber(jlon)) {
            gps_fix_t fix = {0};
            fix.lat = jlat->valuedouble;
            fix.lon = jlon->valuedouble;
            fix.alt = cJSON_IsNumber(jalt) ? jalt->valuedouble : 0.0;
            fix.valid = true;
            control_set_target_fix(&fix);
            // compute az/el to return to caller
            azel_t azel;
            tracker_state_t st;
            control_get_state(&st);
            if (st.my_fix.valid) {
                control_compute_azel(&st.my_fix, &fix, &azel);
                char msgbuf[64];
                snprintf(msgbuf, sizeof(msgbuf), "target_set az=%.2f el=%.2f", azel.az, azel.el);
                send_ack(client_sock, cmdstr, "OK", msgbuf);
            } else {
                send_ack(client_sock, cmdstr, "OK", "target_set_no_myfix");
            }
        } else {
            send_ack(client_sock, cmdstr, "ERROR", "missing_latlon");
        }
    }
    else if (strcmp(cmdstr, "GET_STATUS") == 0) {
        send_ack(client_sock, cmdstr, "OK", "status_soon");
        // status will be pushed by periodic sender
    } else {
        send_ack(client_sock, cmdstr, "ERROR", "unknown_command");
    }

    cJSON_Delete(root);
}

/* --- TCP server task --- */
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
            ESP_LOGW(TAG, "Accept failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        ESP_LOGI(TAG, "Client connected: %s:%d (sock=%d)", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), client_sock);

        // Set recv timeout
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 }; // 200 ms
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Accumulator for partial incoming JSON lines
        char recv_accum[2048];
        size_t recv_accum_len = 0;
        TickType_t last_status_tick = xTaskGetTickCount();

        while (1) {
            char rbuf[RECV_BUF_SZ];
            ssize_t r = recv(client_sock, rbuf, sizeof(rbuf), 0);
            if (r > 0) {
                // append
                if (recv_accum_len + (size_t)r >= sizeof(recv_accum)) {
                    ESP_LOGW(TAG, "recv accumulator overflow; reset");
                    recv_accum_len = 0;
                }
                memcpy(recv_accum + recv_accum_len, rbuf, (size_t)r);
                recv_accum_len += (size_t)r;

                // process lines
                size_t scan = 0;
                while (scan < recv_accum_len) {
                    void *pnl = memchr(recv_accum + scan, '\n', recv_accum_len - scan);
                    if (!pnl) break;
                    size_t line_len = (size_t)((char*)pnl - (recv_accum + scan));
                    // trim trailing \r
                    if (line_len > 0 && recv_accum[scan + line_len - 1] == '\r') line_len--;
                    if (line_len > 0) {
                        handle_command_json(client_sock, recv_accum + scan, line_len);
                    }
                    scan += (line_len + 1); // +1 for newline
                }
                // shift leftovers
                if (scan < recv_accum_len) {
                    memmove(recv_accum, recv_accum + scan, recv_accum_len - scan);
                    recv_accum_len -= scan;
                } else {
                    recv_accum_len = 0;
                }
            } else if (r == 0) {
                ESP_LOGI(TAG, "Client closed connection");
                break;
            } else {
                int err = errno;
                if (err == EWOULDBLOCK || err == EAGAIN || err == EINTR) {
                    // no data
                } else {
                    ESP_LOGW(TAG, "recv error: errno=%d", err);
                    break;
                }
            }

            // send status periodically
            if ((xTaskGetTickCount() - last_status_tick) >= pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {
                send_status_json(client_sock);
                last_status_tick = xTaskGetTickCount();
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        close(client_sock);
        ESP_LOGI(TAG, "Client handler finished");
    }
    // unreachable
    close(server_sock);
    vTaskDelete(NULL);
}
