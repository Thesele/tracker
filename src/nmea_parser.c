/*
 * nmea_parser.c
 *
 * Robust streaming NMEA parser for ESP-IDF.
 * Implements GGA, RMC, GLL parsing and a small streaming assembler.
 *
 * Notes:
 *  - Parser expects ASCII bytes (CR/LF terminated lines typical).
 *  - Caller should feed uart_read_bytes() output into nmea_parser_consume_buffer().
 */

#include "nmea_parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "esp_log.h"

static const char *TAG = "NMEA_PARSER";

/* Internal accumulation buffer for streaming input */
#define NMEA_BUF_SIZE 1024
static char s_buf[NMEA_BUF_SIZE];
static size_t s_len = 0;

/* Last parsed fix (persisted) */
static gps_fix_t s_last_fix;
static bool s_has_last = false;

/* Callback */
static nmea_fix_cb_t s_cb = NULL;
static void *s_cb_ctx = NULL;

/* Helper: convert hex character to value or -1 */
static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
}

/* Verify checksum if '*' present. Returns:
 *  1 = checksum ok
 *  0 = checksum present but mismatch
 * -1 = no checksum present
 */
static int nmea_verify_checksum(const char *s)
{
    if (!s || *s != '$') return 0;
    const char *p = s + 1;
    uint8_t cs = 0;
    while (*p && *p != '*') {
        cs ^= (uint8_t)(*p++);
    }
    ESP_LOGI(TAG, "Computed checksum: %02X", cs);
    if (*p == '*') {
        /* parse two hex chars after '*' */
        p++;
        int hi = hexval(p[0]);
        int lo = hexval(p[1]);
        if (hi < 0 || lo < 0) return 0;
        uint8_t given = (hi << 4) | lo;
        return (cs == given) ? 1 : 0;
    }
    return -1; /* no checksum present */
}

/* Convert NMEA coordinate "ddmm.mmmm" or "dddmm.mmmm" + hemisphere to degrees */
static double nmea_coord_to_deg(const char *coord, char hemi)
{
    if (!coord || coord[0] == '\0') return 0.0;
    double raw = atof(coord); /* safe for typical NMEA strings */
    int deg = (int)(raw / 100.0);
    double minutes = raw - (deg * 100.0);
    double value = (double)deg + (minutes / 60.0);
    if (hemi == 'S' || hemi == 's' || hemi == 'W' || hemi == 'w') value = -value;
    return value;
}

/* Zero a gps_fix_t */
static void gps_fix_clear(gps_fix_t *f)
{
    if (!f) return;
    memset(f, 0, sizeof(*f));
    f->fix_quality = -1;
    f->sats = -1;
    f->valid = false;
    f->utc_time[0] = '\0';
    f->src[0] = '\0';
}

/* Copy local parsed fix to last and call callback if registered */
static void publish_fix(const gps_fix_t *fix)
{
    if (!fix) return;
    s_last_fix = *fix;
    s_has_last = true;
    if (s_cb) s_cb(&s_last_fix, s_cb_ctx);
}

/* Parse GGA sentence. Expects sentence starting with "$GxGGA" and null-terminated.
 * Fills out_fix (if non-null) and returns ESP_OK on parse (even if fix invalid).
 */
static esp_err_t parse_gga(const char *sentence, gps_fix_t *out_fix)
{
    if (!sentence) return ESP_ERR_INVALID_ARG;

    char buf[256];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf)-1] = '\0';

    /* cut off checksum if present */
    char *star = strchr(buf, '*');
    if (star) *star = '\0';

    gps_fix_t f;
    gps_fix_clear(&f);
    strncpy(f.src, "GGA", sizeof(f.src)-1);

    char *saveptr = NULL;
    char *tok = strtok_r(buf, ",", &saveptr);
    int field = 0;
    while (tok) {
        switch (field) {
            case 1: /* UTC time hhmmss.sss */
                strncpy(f.utc_time, tok, sizeof(f.utc_time)-1);
                break;
            case 2: /* lat */
                if (tok[0] != '\0') strncpy(buf, tok, sizeof(buf)-1); /* temporarily reuse */
                break;
            default:
                break;
        }
        tok = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    /* We must re-tokenize because the above only grabbed time improperly (kept for pattern).
     * Instead perform explicit field extraction so positions are correct.
     */
    /* Re-scan fields more carefully */
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf)-1] = '\0';
    if (star) {
        char *s = strchr(buf, '*');
        if (s) *s = '\0';
    }
    char *fields[16];
    int nf = 0;
    char *p = buf;
    while (nf < (int)(sizeof(fields)/sizeof(fields[0])) && p) {
        char *comma = strchr(p, ',');
        if (comma) {
            *comma = '\0';
            fields[nf++] = p;
            p = comma + 1;
        } else {
            fields[nf++] = p;
            break;
        }
    }

    /* Field indices for GGA:
     * 0 = $GxxGGA
     * 1 = UTC time
     * 2 = lat ddmm.mmmm
     * 3 = N/S
     * 4 = lon dddmm.mmmm
     * 5 = E/W
     * 6 = fix quality
     * 7 = num satellites
     * 8 = HDOP
     * 9 = altitude
     */
    if (nf >= 10) {
        /* time */
        if (fields[1] && fields[1][0] != '\0') {
            strncpy(f.utc_time, fields[1], sizeof(f.utc_time)-1);
        }
        /* lat/lon */
        if (fields[2] && fields[3] && fields[2][0] != '\0' && fields[3][0] != '\0') {
            f.lat = nmea_coord_to_deg(fields[2], fields[3][0]);
        }
        if (fields[4] && fields[5] && fields[4][0] != '\0' && fields[5][0] != '\0') {
            f.lon = nmea_coord_to_deg(fields[4], fields[5][0]);
        }
        /* fix quality */
        if (fields[6] && fields[6][0] != '\0') f.fix_quality = atoi(fields[6]); else f.fix_quality = -1;
        /* sats */
        if (fields[7] && fields[7][0] != '\0') f.sats = atoi(fields[7]); else f.sats = -1;
        /* altitude */
        if (fields[9] && fields[9][0] != '\0') f.alt = atof(fields[9]); else f.alt = 0.0;
        /* valid if fix_quality > 0 */
        f.valid = (f.fix_quality > 0);
        if (out_fix) *out_fix = f;
        /* publish only if fix field parsed; even if invalid, we return OK */
        publish_fix(&f);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/* Parse RMC sentence. */
static esp_err_t parse_rmc(const char *sentence, gps_fix_t *out_fix)
{
    if (!sentence) return ESP_ERR_INVALID_ARG;

    char buf[256];
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    char *star = strchr(buf, '*');
    if (star) *star = '\0';

    gps_fix_t f;
    gps_fix_clear(&f);
    strncpy(f.src, "RMC", sizeof(f.src)-1);

    /* Tokenize */
    char *saveptr = NULL;
    char *tok = strtok_r(buf, ",", &saveptr);
    int field = 0;
    while (tok) {
        switch (field) {
            case 1: /* UTC time */
                if (tok && tok[0] != '\0') strncpy(f.utc_time, tok, sizeof(f.utc_time)-1);
                break;
            case 2: /* status A/V */
                /* handled below */
                break;
            case 3: /* lat */
                if (tok && tok[0] != '\0') {
                    /* need hemisphere from next field */
                    char *lat = tok;
                    char *hemi = strtok_r(NULL, ",", &saveptr); /* field 4 */
                    field++; /* account for extra token consumed */
                    if (hemi && hemi[0] != '\0') {
                        f.lat = nmea_coord_to_deg(lat, hemi[0]);
                    }
                }
                break;
            case 5: /* lon (if not consumed earlier) */
                if (tok && tok[0] != '\0') {
                    char *lon = tok;
                    char *hemi = strtok_r(NULL, ",", &saveptr); /* field 6 */
                    field++;
                    if (hemi && hemi[0] != '\0') {
                        f.lon = nmea_coord_to_deg(lon, hemi[0]);
                    }
                }
                break;
            default:
                break;
        }
        tok = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    /* Determine validity: look for "A" status token in original sentence. Simpler: search for ",A," near beginning. */
    if (strstr(sentence, ",A,") != NULL || strstr(sentence, ",A*") != NULL) {
        f.valid = true;
    } else {
        f.valid = false;
    }

    if (out_fix) *out_fix = f;
    publish_fix(&f);
    return ESP_OK;
}

/* Parse GLL sentence (lat/lon + status). */
static esp_err_t parse_gll(const char *sentence, gps_fix_t *out_fix)
{
    if (!sentence) return ESP_ERR_INVALID_ARG;

    char buf[256];
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    char *star = strchr(buf, '*');
    if (star) *star = '\0';

    gps_fix_t f;
    gps_fix_clear(&f);
    strncpy(f.src, "GLL", sizeof(f.src)-1);

    char *saveptr = NULL;
    char *tok = strtok_r(buf, ",", &saveptr);
    int field = 0;
    while (tok) {
        switch (field) {
            case 1:
                if (tok[0] != '\0') strncpy(buf, tok, sizeof(buf)-1);
                break;
            default:
                break;
        }
        tok = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    /* Re-scan into fields[] for stable indices */
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    if (star) {
        char *s = strchr(buf, '*');
        if (s) *s = '\0';
    }
    char *fields[8];
    int nf = 0;
    char *p = buf;
    while (nf < (int)(sizeof(fields)/sizeof(fields[0])) && p) {
        char *comma = strchr(p, ',');
        if (comma) {
            *comma = '\0';
            fields[nf++] = p;
            p = comma + 1;
        } else {
            fields[nf++] = p;
            break;
        }
    }
    /* indices:
     * 0 = $GxGLL
     * 1 = lat
     * 2 = N/S
     * 3 = lon
     * 4 = E/W
     * 5 = UTC time
     * 6 = status (A/V)
     */
    if (nf >= 6) {
        if (fields[1] && fields[1][0] != '\0' && fields[2] && fields[2][0] != '\0') {
            f.lat = nmea_coord_to_deg(fields[1], fields[2][0]);
        }
        if (fields[3] && fields[3][0] != '\0' && fields[4] && fields[4][0] != '\0') {
            f.lon = nmea_coord_to_deg(fields[3], fields[4][0]);
        }
        if (fields[5]) strncpy(f.utc_time, fields[5], sizeof(f.utc_time)-1);
        if (fields[6] && fields[6][0] == 'A') f.valid = true;
        else f.valid = false;

        if (out_fix) *out_fix = f;
        publish_fix(&f);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/* Recognize sentence type and dispatch */
esp_err_t nmea_parser_parse_sentence(const char *sentence, gps_fix_t *out_fix)
{
    if (!sentence) return ESP_ERR_INVALID_ARG;

    /* ensure sentence starts with $ */
    const char *s = sentence;
    while (*s && isspace((unsigned char)*s)) s++;
    if (*s != '$') return ESP_ERR_INVALID_ARG;

    int chk = nmea_verify_checksum(s);
    if (chk == 0) {
        ESP_LOGW(TAG, "NMEA checksum mismatch: %s", s);
        return ESP_FAIL;
    } else if (chk == -1) {
        ESP_LOGD(TAG, "NMEA sentence has no checksum: %s", s);
        /* allow parsing without checksum */
    }

    /* identify type e.g. $GPGGA or $GNGGA or $GLGGA etc. */
    const char *type = s + 1; /* skip '$' */
    /* type runs until first comma or null */
    char t[8] = {0};
    const char *c = strchr(type, ',');
    size_t tlen = (c) ? (size_t)(c - type) : strlen(type);
    if (tlen >= sizeof(t)) tlen = sizeof(t)-1;
    strncpy(t, type, tlen);
    t[tlen] = '\0';

    /* Accept both "GPGGA" and "GNGGA" and "GLGGA" etc.; focus on last 3 chars */
    size_t tl = strlen(t);
    if (tl >= 3) {
        const char *suffix = &t[tl - 3]; /* e.g., "GGA" */
        if (strcmp(suffix, "GGA") == 0) {
            return parse_gga(s, out_fix);
        } else if (strcmp(suffix, "RMC") == 0) {
            return parse_rmc(s, out_fix);
        } else if (strcmp(suffix, "GLL") == 0) {
            return parse_gll(s, out_fix);
        } else {
            /* unsupported sentence type */
            return ESP_ERR_NOT_FOUND;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

/* Public API: init/reset/register callback */
esp_err_t nmea_parser_init(void)
{
    s_len = 0;
    s_buf[0] = '\0';
    s_cb = NULL;
    s_cb_ctx = NULL;
    s_has_last = false;
    gps_fix_clear(&s_last_fix);
    return ESP_OK;
}

esp_err_t nmea_parser_reset(void)
{
    return nmea_parser_init();
}

esp_err_t nmea_parser_register_callback(nmea_fix_cb_t cb, void *user_ctx)
{
    s_cb = cb;
    s_cb_ctx = user_ctx;
    return ESP_OK;
}

const gps_fix_t *nmea_parser_get_last_fix(void)
{
    if (!s_has_last) return NULL;
    return &s_last_fix;
}

/* Consume a single byte: append to buffer and attempt to extract lines */
esp_err_t nmea_parser_consume_byte(uint8_t b)
{
    if (s_len + 1 >= NMEA_BUF_SIZE) {
        /* Buffer overflow: drop everything until next '$' */
        ESP_LOGW(TAG, "NMEA stream buffer overflow; resynchronizing");
        /* find next '$' */
        char *p = memchr(s_buf, '$', NMEA_BUF_SIZE);
        if (p) {
            size_t off = p - s_buf;
            size_t rem = s_len - off;
            memmove(s_buf, &s_buf[off], rem);
            s_len = rem;
        } else {
            s_len = 0;
        }
    }
    s_buf[s_len++] = (char)b;
    s_buf[s_len] = '\0';

    /* check for newline */
    for (;;) {
        char *nl = memchr(s_buf, '\n', s_len);
        if (!nl) break;
        size_t line_len = nl - s_buf + 1; /* include '\n' */
        /* find start of sentence ($) */
        char *start = memchr(s_buf, '$', line_len);
        if (!start) {
            /* no start symbol: drop up to newline */
            size_t newlen = s_len - line_len;
            memmove(s_buf, s_buf + line_len, newlen);
            s_len = newlen;
            s_buf[s_len] = '\0';
            continue;
        }
        size_t start_idx = start - s_buf;
        /* extract sentence from start to newline */
        char sentence[512];
        size_t slen = (nl - start) + 1;
        if (slen >= sizeof(sentence)) {
            ESP_LOGW(TAG, "NMEA sentence too long; dropping");
            /* drop up to newline and continue */
            size_t newlen = s_len - line_len;
            memmove(s_buf, s_buf + line_len, newlen);
            s_len = newlen;
            s_buf[s_len] = '\0';
            continue;
        }
        memcpy(sentence, start, slen);
        sentence[slen] = '\0';

        /* attempt parse */
        gps_fix_t fix;
        esp_err_t ret = nmea_parser_parse_sentence(sentence, &fix);
        if (ret == ESP_OK) {
            /* parse succeeded; publish_fix already called by parse_* functions */
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGD(TAG, "NMEA sentence type not supported: %s", sentence);
        } else {
            ESP_LOGW(TAG, "NMEA parse error (%d) for: %s", ret, sentence);
        }

        /* remove processed bytes up to newline */
        size_t newlen = s_len - (start_idx + slen);
        if (newlen > 0) memmove(s_buf, s_buf + start_idx + slen, newlen);
        s_len = newlen;
        s_buf[s_len] = '\0';
    }
    return ESP_OK;
}

/* Consume buffer: feed each byte into consume_byte (optimized loop) */
esp_err_t nmea_parser_consume_buffer(const uint8_t *buf, size_t len)
{
    if (!buf && len > 0) return ESP_ERR_INVALID_ARG;
    for (size_t i = 0; i < len; ++i) {
        nmea_parser_consume_byte(buf[i]);
    }
    return ESP_OK;
}
