#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SERVO_AZ = 0,
    SERVO_EL = 1,
    SERVO_COUNT = 2
} servo_id_t;

typedef struct {
    // Pulse range (microseconds)
    uint32_t min_pulse_us;    // typically 500
    uint32_t neutral_pulse_us;// typically 1500
    uint32_t max_pulse_us;    // typically 2500

    // Angle range (degrees) that maps linearly to pulse range.
    // E.g. min_deg=-135, max_deg=+135 and neutral_deg=0 -> neutral_pulse_us=1500
    double min_deg;
    double max_deg;
} servo_config_t;

typedef struct {
    double kp;
    double ki;
    double kd;
    // safety limits
    double max_integrator; // to limit integral windup (in deg*sec or deg units depending on dt usage)
    double max_rate_deg_s; // maximum command slew rate (deg per second)
} servo_pid_cfg_t;

// Initialize servo control module and start the control task
esp_err_t servo_control_init(const servo_config_t configs[SERVO_COUNT],
                             const servo_pid_cfg_t pid_cfgs[SERVO_COUNT]);

// Stop servo control task and cleanup
void servo_control_deinit(void);

// Set a commanded angle (degrees) for a servo (thread-safe). The controller will move the servo to this angle.
// If mode_manual true, this is a manual az/el; if false it's produced by control_task (tracking target).
esp_err_t servo_set_target_angle(servo_id_t id, double angle_deg);

// Query last commanded angle (not necessarily reached)
esp_err_t servo_get_target_angle(servo_id_t id, double *out_angle_deg);

// Read last measured angle from feedback (if implemented). Returns ESP_ERR_NOT_SUPPORTED if feedback is not implemented.
esp_err_t servo_get_measured_angle(servo_id_t id, double *out_angle_deg);

// Force output pulse (microseconds). For advanced users; will be clamped to servo_config range.
esp_err_t servo_force_pulse_us(servo_id_t id, uint32_t pulse_us);

// Optional: set PID gains dynamically
esp_err_t servo_set_pid(servo_id_t id, servo_pid_cfg_t cfg);

#ifdef __cplusplus
}
#endif

#endif // SERVO_CONTROL_H
