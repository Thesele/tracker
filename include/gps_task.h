#pragma once
#include "nmea_parser.h"
#include "esp_err.h"
#include "control_task.h"

#define GPS_SENTENCE_MAX_LEN 128
#define GPS_READ_TIMEOUT_MS 50

typedef struct {
    char last_gga[GPS_SENTENCE_MAX_LEN];
    char last_rmc[GPS_SENTENCE_MAX_LEN];
} gps_sentences_t;

extern gps_sentences_t gps_buf;

// GPS task entry point
void gps_task(void *arg);
