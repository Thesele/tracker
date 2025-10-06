#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

#include "i2c_gps.h"       // provides: esp_err_t gps_i2c_read_sentence(char*,size_t,uint32_t);
#include "nmea_parser.h"   // provides: esp_err_t nmea_parser_parse_sentence(const char*, gps_fix_t*);

#define TAG "GPS_TASK"

#define GPS_SENTENCE_MAX_LEN 128   // max NMEA sentence we will accept
#define GPS_CHUNK_SIZE        32   // keep small to reduce padding problems
#define GPS_READ_TIMEOUT_MS   200  // timeout for i2c read

// Public/latest containers (thread-safe access via mutex)
static char last_gga[GPS_SENTENCE_MAX_LEN];
static char last_rmc[GPS_SENTENCE_MAX_LEN];
static SemaphoreHandle_t last_sentence_mutex = NULL;

// Local assembly buffer for current in-flight sentence
static char cur_sentence[GPS_SENTENCE_MAX_LEN];
static size_t cur_pos = 0;
static bool in_sentence = false; // true after we saw '$' and until we see '\n'

// last parsed fix (optional export)
gps_fix_t myfix;

// Optional helper getters
// Copies the latest GGA into user buffer (null-terminated). Returns ESP_OK on success.
esp_err_t gps_get_last_gga(char *out, size_t out_len)
{
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;
    if (!last_sentence_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(last_sentence_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;
    strncpy(out, last_gga, out_len - 1);
    out[out_len - 1] = '\0';
    xSemaphoreGive(last_sentence_mutex);
    return ESP_OK;
}

esp_err_t gps_get_last_rmc(char *out, size_t out_len)
{
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;
    if (!last_sentence_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(last_sentence_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;
    strncpy(out, last_rmc, out_len - 1);
    out[out_len - 1] = '\0';
    xSemaphoreGive(last_sentence_mutex);
    return ESP_OK;
}

// Helper: trim trailing CR/LF in-place
static void trim_crlf(char *s)
{
    if (!s) return;
    size_t L = strlen(s);
    while (L > 0 && (s[L-1] == '\r' || s[L-1] == '\n')) {
        s[L-1] = '\0';
        L--;
    }
}

// Helper: check if sentence type equals 3-char type at position 3 (NMEA standard)
static bool nmea_is_type(const char *s, const char *type3)
{
    if (!s || strlen(s) < 6) return false; // minimal: "$xxTYY"
    // type starts at index 3 (0 = '$', 1-2 = talker ID, 3..5 = type)
    return (strncmp(&s[3], type3, 3) == 0);
}

void gps_task(void *arg)
{
    ESP_LOGI(TAG, "GPS task started (assembling stream)...");

    // create mutex once
    if (last_sentence_mutex == NULL) {
        last_sentence_mutex = xSemaphoreCreateMutex();
        if (last_sentence_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            vTaskDelete(NULL);
            return;
        }
    }

    // Clear storage
    last_gga[0] = '\0';
    last_rmc[0] = '\0';
    cur_sentence[0] = '\0';
    cur_pos = 0;
    in_sentence = false;

    // temp chunk buffer used for reading from gps_i2c_read_sentence()
    char chunk[GPS_CHUNK_SIZE];

    while (1) {
        // Read a small chunk from I2C. Keep signature untouched.
        esp_err_t ret = gps_i2c_read_sentence(chunk, sizeof(chunk), GPS_READ_TIMEOUT_MS);

        if (ret == ESP_OK) {
            // gps_i2c_read_sentence fills chunk as a C-string (may include many '\n' padding)
            size_t chunk_len = strnlen(chunk, sizeof(chunk));
            if (chunk_len == 0) {
                // nothing useful returned
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // Process the chunk byte-by-byte (it may contain multiple sentences and padding)
            for (size_t i = 0; i < chunk_len; ++i) {
                char c = chunk[i];

                // If not currently assembling a sentence, ignore everything until '$'
                if (!in_sentence) {
                    if (c == '$') {
                        in_sentence = true;
                        cur_pos = 0;
                        cur_sentence[cur_pos++] = c;
                    } else {
                        // skip padding newlines and other bytes
                        continue;
                    }
                } else {
                    // already in a sentence; store char if space permits
                    if (cur_pos < (GPS_SENTENCE_MAX_LEN - 1)) {
                        cur_sentence[cur_pos++] = c;
                    } else {
                        // overflow: reset and drop the fragment
                        in_sentence = false;
                        cur_pos = 0;
                        cur_sentence[0] = '\0';
                        ESP_LOGW(TAG, "Sentence too long, discarding fragment");
                        continue;
                    }

                    // Sentence termination: NMEA lines end with '\r\n' (we look for '\n')
                    if (c == '\n') {
                        // terminate and process
                        cur_sentence[cur_pos] = '\0';
                        // trim CR/LF
                        trim_crlf(cur_sentence);

                        // Sanity: ensure it starts with $
                        if (cur_sentence[0] == '$') {
                            // Identify type (GGA or RMC)
                            if (nmea_is_type(cur_sentence, "GGA")) {
                                // store last_gga safely
                                if (xSemaphoreTake(last_sentence_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                                    strncpy(last_gga, cur_sentence, sizeof(last_gga) - 1);
                                    last_gga[sizeof(last_gga) - 1] = '\0';
                                    xSemaphoreGive(last_sentence_mutex);
                                }
                                // parse GGA into myfix
                                if (nmea_parser_parse_sentence(cur_sentence, &myfix) == ESP_OK) {
                                    if (myfix.valid) {
                                        ESP_LOGI(TAG, "Got fix: Lat=%.6f Lon=%.6f Alt=%.2f Sats=%d",
                                                myfix.lat, myfix.lon, myfix.alt, myfix.sats);
                                    } else {
                                        ESP_LOGI(TAG, "GGA received but no valid fix yet (quality=%d).", myfix.fix_quality);
                                    }
                                } else {
                                    ESP_LOGW(TAG, "GGA parse failed");
                                }
                            }
                            else if (nmea_is_type(cur_sentence, "RMC")) {
                                if (xSemaphoreTake(last_sentence_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                                    strncpy(last_rmc, cur_sentence, sizeof(last_rmc) - 1);
                                    last_rmc[sizeof(last_rmc) - 1] = '\0';
                                    xSemaphoreGive(last_sentence_mutex);
                                }
                            } else {
                                // other sentence types - ignore
                            }
                        }

                        // reset for next sentence
                        in_sentence = false;
                        cur_pos = 0;
                        cur_sentence[0] = '\0';
                    }
                } // end in_sentence
            } // end for chunk bytes
        } else if (ret == ESP_ERR_TIMEOUT) {
            // no data within timeout — fine, just loop
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            // some I2C error - log and wait a bit
            ESP_LOGW(TAG, "gps_i2c_read_sentence error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // small yield — tune as needed
        taskYIELD();
    } // end while
}