// control_task.c
// Implements the control task and thread-safe state manipulation.
// Computes az/el from my_fix -> target_fix (ECEF conversion).

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "control_task.h"
#include "servo_control.h"

static const char *TAG = "CONTROL";

static tracker_state_t g_state;
static SemaphoreHandle_t g_state_mux = NULL;
static TaskHandle_t g_control_task_handle = NULL;

// Utility: degrees <-> radians
static inline double deg2rad(double d){ return d * M_PI / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

// ECEF conversion (WGS84)
static void geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m,
                             double *x, double *y, double *z)
{
    // WGS84
    const double a = 6378137.0;
    const double e2 = 6.69437999014e-3;

    double lat = deg2rad(lat_deg);
    double lon = deg2rad(lon_deg);

    double N = a / sqrt(1.0 - e2 * sin(lat)*sin(lat));
    *x = (N + alt_m) * cos(lat) * cos(lon);
    *y = (N + alt_m) * cos(lat) * sin(lon);
    *z = (N * (1.0 - e2) + alt_m) * sin(lat);
}

// Compute azimuth and elevation from observer (myfix) to target
// Azimuth: degrees from north (0° = north) clockwise toward east
// Elevation: degrees above horizon
static void  compute_azel_from_fix(const gps_fix_t *myf, const gps_fix_t *tgt, azel_t *out)
{
    // Convert to ECEF
    double x1, y1, z1, x2, y2, z2;
    geodetic_to_ecef(myf->lat, myf->lon, myf->alt, &x1, &y1, &z1);
    geodetic_to_ecef(tgt->lat, tgt->lon, tgt->alt, &x2, &y2, &z2);

    // Vector from observer to target in ECEF
    double vx = x2 - x1;
    double vy = y2 - y1;
    double vz = z2 - z1;

    // Convert vector to local ENU coordinates at observer
    double lat = deg2rad(myf->lat);
    double lon = deg2rad(myf->lon);
    // ECEF -> ENU rotation
    double sin_lat = sin(lat), cos_lat = cos(lat);
    double sin_lon = sin(lon), cos_lon = cos(lon);

    double east  = -sin_lon * vx + cos_lon * vy;
    double north = -cos_lon * sin_lat * vx - sin_lat * sin_lon * vy + cos_lat * vz;
    double up    = cos_lat * cos_lon * vx + cos_lat * sin_lon * vy + sin_lat * vz;

    // Azimuth (0=north, clockwise)
    double az = atan2(east, north); // radians
    if (az < 0) az += 2.0*M_PI;
    // Elevation
    double horizontal_range = sqrt(east*east + north*north);
    double el = atan2(up, horizontal_range);

    out->az = rad2deg(az);
    out->el = rad2deg(el);
}

esp_err_t control_init(void)
{
    if (g_state_mux) return ESP_ERR_INVALID_STATE;
    g_state_mux = xSemaphoreCreateMutex();
    if (!g_state_mux) return ESP_ERR_NO_MEM;

    // Initialize default state
    memset(&g_state, 0, sizeof(g_state));
    g_state.mode = TRACK_MODE_GPS;
    g_state.current_azel.az = 0.0;
    g_state.current_azel.el = 0.0;
    g_state.manual_azel.az = 0.0;
    g_state.manual_azel.el = 0.0;
    return ESP_OK;
}

static void control_task_fn(void *arg)
{
    const TickType_t delay = pdMS_TO_TICKS(100); // 10 Hz control loop
    tracker_state_t local;

    ESP_LOGI(TAG, "Control task started");

    for (;;) {
        // Copy state atomically
        if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(&local, &g_state, sizeof(local));
            xSemaphoreGive(g_state_mux);
        } else {
            // Couldn't acquire mutex; skip this cycle
            vTaskDelay(delay);
            continue;
        }

        // Behavior by mode
        if (local.mode == TRACK_MODE_GPS) {
            // Only compute az/el if both fixes are valid
            if (local.my_fix.valid && local.target_fix.valid) {
                azel_t computed;
                compute_azel_from_fix(&local.my_fix, &local.target_fix, &computed);

                // Set servo angles (convert az to servo angle)
                float az_angle = servo_Az_to_angle((float)computed.az);
                servo_set_angle(SERVO_AZ, az_angle); // Azimuth servo
                servo_set_angle(SERVO_EL, (float)computed.el); // Elevation servo

                // update global current_azel
                if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_state.current_azel = computed;
                    xSemaphoreGive(g_state_mux);
                }
            }
            // if no valid target, do nothing
        } else { // MANUAL
            // Ensure the current_azel equals manual_azel
            if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_state.current_azel = g_state.manual_azel;

                // Clear target_fix to reflect manual control
                g_state.target_fix.valid = false;
                xSemaphoreGive(g_state_mux);
                
                // Set servo angles (convert az to servo angle)
                float az_angle = servo_Az_to_angle((float)local.current_azel.az);
                servo_set_angle(SERVO_AZ, az_angle); // Azimuth servo
                servo_set_angle(SERVO_EL, (float)local.current_azel.el); // Elevation servo
            }
        }

        // Sleep until next cycle
        vTaskDelay(delay);
    }
}

// Public API functions
esp_err_t control_start(void)
{
    if (!g_state_mux) return ESP_ERR_INVALID_STATE;
    if (g_control_task_handle) return ESP_ERR_INVALID_STATE;
    if (xTaskCreate(control_task_fn, "control_task", 5*1024, NULL, 6, &g_control_task_handle) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t control_set_mode(track_mode_t mode)
{
    if (!g_state_mux) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    g_state.mode = mode;
    if (mode == TRACK_MODE_MANUAL) {
        // clear target
        g_state.target_fix.valid = false;
    }
    xSemaphoreGive(g_state_mux);
    return ESP_OK;
}

esp_err_t control_set_manual_azel(double az_deg, double el_deg)
{
    if (!g_state_mux) return ESP_ERR_INVALID_STATE;
    if (az_deg < 0.0) {
        // normalize
        while (az_deg < 0.0) az_deg += 360.0;
    }
    if (az_deg >= 360.0) az_deg = fmod(az_deg, 360.0);

    if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    g_state.manual_azel.az = az_deg;
    g_state.manual_azel.el = el_deg;
    g_state.mode = TRACK_MODE_MANUAL;
    // Clear target to reflect manual taking precedence
    g_state.target_fix.valid = false;
    // Also set current_azel immediately
    g_state.current_azel = g_state.manual_azel;
    xSemaphoreGive(g_state_mux);
    return ESP_OK;
}

esp_err_t control_set_target_fix(const gps_fix_t *fix)
{
    if (!g_state_mux || !fix) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    memcpy(&g_state.target_fix, fix, sizeof(gps_fix_t));
    // keep mode unchanged; if in GPS mode, control task will compute az/el
    xSemaphoreGive(g_state_mux);
    ESP_LOGI("CONTROL","TARGET FIX SET: ");
    return ESP_OK;
}
esp_err_t control_set_my_fix(const gps_fix_t *fix)
{
    if (!g_state_mux || !fix) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    memcpy(&g_state.my_fix, fix, sizeof(gps_fix_t));
    // keep mode unchanged; if in GPS mode, control task will compute az/el
    xSemaphoreGive(g_state_mux);
    return ESP_OK;
}

esp_err_t control_clear_target_fix(void)
{
    if (!g_state_mux) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    g_state.target_fix.valid = false;
    xSemaphoreGive(g_state_mux);
    return ESP_OK;
}

esp_err_t control_get_state(tracker_state_t *out_state)
{
    if (!g_state_mux || !out_state) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(g_state_mux, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    memcpy(out_state, &g_state, sizeof(*out_state));
    xSemaphoreGive(g_state_mux);
    return ESP_OK;
}

esp_err_t control_compute_azel(const gps_fix_t *myfix, const gps_fix_t *target, azel_t *out)
{
    if (!myfix || !target || !out) return ESP_ERR_INVALID_ARG;
    compute_azel_from_fix(myfix, target, out);
    return ESP_OK;
}
