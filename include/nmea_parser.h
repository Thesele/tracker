#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * Lightweight, robust NMEA parser for ESP-IDF.
 *
 * Features:
 *  - streaming input (feed bytes or buffers)
 *  - checksum verification (if present)
 *  - GGA, RMC, GLL sentence parsing for lat/lon/alt/status
 *  - callback API invoked when a sentence is parsed
 *  - retains last parsed fix (read-only access)
 *
 * Threading model: parser is not internally synchronized. Call from a single task
 * (e.g. your UART RX task) or add external synchronization.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Parsed GPS fix structure. */
typedef struct {
    double     lat;          /**< degrees, +N, -S */
    double     lon;          /**< degrees, +E, -W */
    double     alt;          /**< meters (when available, else 0) */
    int        fix_quality;  /**< GGA fix quality (0 = invalid, 1=GPS fix, 2=DGPS, ...). -1 if unknown */
    int        sats;         /**< number of satellites (if available), -1 if unknown */
    bool       valid;        /**< true if parser reports a valid fix (quality/status) */
    char       utc_time[16]; /**< optional UTC hhmmss.sss when present */
    char       src[8];       /**< sentence source id, e.g. "GGA","RMC","GLL" */
} gps_fix_t;

/** Callback invoked when a sentence is parsed (validity inside gps_fix_t). */
typedef void (*nmea_fix_cb_t)(const gps_fix_t *fix, void *user_ctx);

/**
 * Initialize the parser. Must be called before consume functions.
 * Returns ESP_OK or an esp_err_t on failure.
 */
esp_err_t nmea_parser_init(void);

/**
 * Reset internal parser state (flush buffers, last-fix cleared).
 */
esp_err_t nmea_parser_reset(void);

/**
 * Register a callback. The callback will be called synchronously from the
 * thread that feeds the parser (i.e. the caller of consume functions).
 * user_ctx is passed back to the callback.
 */
esp_err_t nmea_parser_register_callback(nmea_fix_cb_t cb, void *user_ctx);

/**
 * Feed a single byte into the parser. Useful if you handle bytes one-by-one.
 */
esp_err_t nmea_parser_consume_byte(uint8_t b);

/**
 * Feed a buffer of bytes into the parser. Parser will assemble full NMEA
 * sentences and call the registered callback when a sentence is parsed.
 */
esp_err_t nmea_parser_consume_buffer(const uint8_t *buf, size_t len);

/**
 * Parse a single, null-terminated NMEA sentence (for unit tests). It will
 * validate checksum (when present) and fill out_fix if recognized.
 *
 * Returns:
 *   ESP_OK           : sentence parsed (out_fix filled; valid flag indicates fix)
 *   ESP_ERR_NOT_FOUND: sentence type not supported
 *   ESP_FAIL         : checksum failed or irrecoverable parse error
 *   ESP_ERR_INVALID_ARG : null pointer args
 */
esp_err_t nmea_parser_parse_sentence(const char *sentence, gps_fix_t *out_fix);

/**
 * Return pointer to the last parsed fix (read-only). May be NULL if none parsed.
 */
const gps_fix_t *nmea_parser_get_last_fix(void);

#ifdef __cplusplus
}
#endif

#endif // NMEA_PARSER_H

