#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "i2c_gps.h"    // your I2C GPS functions
#include "nmea_parser.h"
#include "gps_task.h"

#define TAG "GPS_TASK"
#define ACCUM_BUF_SIZE 512

static char accum_buf[ACCUM_BUF_SIZE];
static size_t accum_len = 0;

static char last_gga[GPS_SENTENCE_MAX_LEN] = {0};
static char last_rmc[GPS_SENTENCE_MAX_LEN] = {0};

extern gps_fix_t myfix;

static void process_sentence(const char *sentence)
{
    if (strncmp(sentence, "$GNGGA", 6) == 0 || strncmp(sentence, "$GPGGA", 6) == 0) {
        strncpy(last_gga, sentence, GPS_SENTENCE_MAX_LEN-1);
        last_gga[GPS_SENTENCE_MAX_LEN-1] = '\0';

        if (nmea_parser_parse_sentence(sentence, &myfix) == ESP_OK && myfix.valid) {
            ESP_LOGI(TAG, "GGA parsed successfully: NMEA %s", sentence);
            ESP_LOGI(TAG, "Got fix: Lat=%.6f, Lon=%.6f, Alt=%.2f, Sats=%d",
                     myfix.lat, myfix.lon, myfix.alt, myfix.sats);
            control_set_my_fix(&myfix); // Update shared state with my location
            vTaskDelete(NULL); // Stop task after getting a valid fix
        } else {
            ESP_LOGW(TAG, "GGA parse failed or no fix yet");
        }
    } else if (strncmp(sentence, "$GNRMC", 6) == 0 || strncmp(sentence, "$GPRMC", 6) == 0) {
        strncpy(last_rmc, sentence, GPS_SENTENCE_MAX_LEN-1);
        last_rmc[GPS_SENTENCE_MAX_LEN-1] = '\0';
        // Keep RMC for future use if needed
    }
}

static void extract_sentences_from_buffer(void)
{
    size_t i = 0;

    while (i < accum_len) {
        // Find start of sentence
        if (accum_buf[i] != '$') {
            i++;
            continue;
        }

        // Find end of sentence: either \n or * + 2 hex digits
        size_t start = i;
        size_t end = start;
        int found_end = 0;

        for (; end < accum_len; end++) {
            if (accum_buf[end] == '\n' || accum_buf[end] == '\r') {
                found_end = 1;
                break;
            }
            if (accum_buf[end] == '*') {
                if (end + 2 < accum_len) {
                    found_end = 1;
                    end += 2; // include checksum hex digits
                    break;
                }
            }
        }

        if (!found_end) break; // Wait for more data

        // Copy sentence
        size_t len = end - start + 1;
        if (len >= GPS_SENTENCE_MAX_LEN) len = GPS_SENTENCE_MAX_LEN - 1;

        char sentence[GPS_SENTENCE_MAX_LEN] = {0};
        memcpy(sentence, &accum_buf[start], len);
        sentence[len] = '\0';

        // Trim trailing \r or \n
        size_t slen = strlen(sentence);
        while (slen > 0 && (sentence[slen-1] == '\r' || sentence[slen-1] == '\n')) {
            sentence[slen-1] = '\0';
            slen--;
        }

        // Process
        process_sentence(sentence);

        // Move buffer forward
        i = end + 1;
    }

    // Shift remaining data to start of buffer
    if (i < accum_len) {
        memmove(accum_buf, &accum_buf[i], accum_len - i);
        accum_len -= i;
    } else {
        accum_len = 0;
    }
}

void gps_task(void *arg)
{
    ESP_LOGI(TAG, "GPS task started, waiting for fix...");

    while (1) {
        char read_buf[128] = {0};
        esp_err_t ret = gps_i2c_read_sentence(read_buf, sizeof(read_buf), 500);

        if (ret == ESP_OK) {
            // Append to accumulation buffer
            size_t read_len = strnlen(read_buf, sizeof(read_buf));
            if (read_len + accum_len >= ACCUM_BUF_SIZE) {
                ESP_LOGW(TAG, "Accumulation buffer overflow, discarding old data");
                accum_len = 0;
            }
            memcpy(&accum_buf[accum_len], read_buf, read_len);
            accum_len += read_len;

            // Extract complete sentences and process
            extract_sentences_from_buffer();
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Error reading GPS: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // 200ms polling
    }
}
