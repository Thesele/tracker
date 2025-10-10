#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "nmea_parser.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mode selection
typedef enum {
    TRACK_MODE_GPS = 0,
    TRACK_MODE_MANUAL = 1
} track_mode_t;

// Simple AZ/EL container
typedef struct {
    double az; // degrees 0..360
    double el; // degrees -90..90
} azel_t;

// Public shared state (use provided getters/setters)
typedef struct {
    gps_fix_t my_fix;     // station coordinates (I2C GPS)
    gps_fix_t target_fix; // target coordinates (from UART or GUI)
    azel_t current_azel;  // current commanded az/el
    azel_t manual_azel;   // last manual set az/el
    track_mode_t mode;
} tracker_state_t;

// Initialize control subsystem (call once from app_main)
esp_err_t control_init(void);

// Start the control task (creates FreeRTOS task)
esp_err_t control_start(void);

// Helpers used by TCP task to modify state safely
esp_err_t control_set_mode(track_mode_t mode);
esp_err_t control_set_my_fix(const gps_fix_t *fix);          // sets my_fix (I2C GPS)
esp_err_t control_set_manual_azel(double az_deg, double el_deg); // sets manual and switches mode to MANUAL
esp_err_t control_set_target_fix(const gps_fix_t *fix);          // sets target (and mode stays unchanged)
esp_err_t control_clear_target_fix(void);
esp_err_t control_get_state(tracker_state_t *out_state);        // copies current state

// Compute az/el given my_fix and target_fix (thread-safe read)
esp_err_t control_compute_azel(const gps_fix_t *myfix, const gps_fix_t *target, azel_t *out);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_TASK_H

