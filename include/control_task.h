/* control_task.h
 *
 * Header for the tracker control subsystem.
 * Centralises all shared state (GPS positions, tracking mode,
 * manual AZ/EL, and IMU orientation) and provides a thread-safe
 * public API for other FreeRTOS tasks.
 *
 * The control task runs the main pointing algorithm, while
 * GPS, IMU, and UI tasks update the shared state via the
 * functions declared here.
 */

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h> // <-- Added for M_PI and math functions
#include "esp_err.h"
#include "nmea_parser.h"        // gps_fix_t definition

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Tracking mode enumeration                                                  */
/* -------------------------------------------------------------------------- */
typedef enum {
    TRACK_MODE_GPS    = 0,   /**< Automatic pointing toward a GPS target      */
    TRACK_MODE_MANUAL = 1    /**< User-defined azimuth/elevation              */
} track_mode_t;

/* -------------------------------------------------------------------------- */
/* Azimuth / Elevation container                                              */
/* -------------------------------------------------------------------------- */
typedef struct {
    double az;               /**< Azimuth in degrees, 0 = North, 90 = East, [0,360) */
    double el;               /**< Elevation in degrees, 0 = horizon, 90 = zenith   */
} azel_t;

/* -------------------------------------------------------------------------- */
/* Math Utilities                                                             */
/* -------------------------------------------------------------------------- */
// Utility: degrees <-> radians
static inline double deg2rad(double d){ return d * M_PI / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

/* -------------------------------------------------------------------------- */
/* Quaternion (unit quaternion from IMU fusion)                               */
/* -------------------------------------------------------------------------- */
typedef struct {
    double w, x, y, z;       /**< w = real part, x/y/z = imaginary parts         */
} quat_t;

/* -------------------------------------------------------------------------- */
/* Euler angles (derived from quaternion, degrees)                            */
/* -------------------------------------------------------------------------- */
typedef struct {
    double roll;             /**< Rotation about X-axis (left/right tilt)        */
    double pitch;            /**< Rotation about Y-axis (forward/back tilt)      */
    double yaw;              /**< Rotation about Z-axis (heading, 0 = North)     */
} euler_t;

/* -------------------------------------------------------------------------- */
/* Central shared tracker state                                               */
/* -------------------------------------------------------------------------- */
typedef struct {
    /* ----- GPS / Location --------------------------------------------------- */
    gps_fix_t my_fix;        /**< This station's position (I2C GPS)              */
    gps_fix_t target_fix;    /**< Target position (UART, GUI, etc.)              */

    /* ----- Control & Pointing ---------------------------------------------- */
    track_mode_t mode;       /**< Current operating mode                         */
    azel_t manual_azel;      /**< Last user-set AZ/EL (used in MANUAL mode)      */
    azel_t current_azel;     /**< AZ/EL currently commanded to the actuator      */

    /* ----- IMU / Orientation ----------------------------------------------- */
    bool      imu_valid;     /**< true when the latest IMU data is fresh         */
    quat_t    quat;          /**< Raw quaternion from sensor-fusion              */
    euler_t   euler;         /**< Converted Euler angles for readability/logging */
    double    heading;       /**< Compass heading (degrees, 0 = North)           */
} tracker_state_t;

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* All functions return ESP_OK on success or an esp_err_t error code.         */
/* Designed to be called from any FreeRTOS task; internal mutexes guarantee   */
/* atomic updates where required.                                             */
/* ========================================================================== */

/**
 * @brief Initialise the control subsystem
 *
 * Creates internal mutexes, clears state, and sets defaults.
 * Must be called once from `app_main()` before any other API call.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t control_init(void);

/**
 * @brief Spawn the main control loop task
 *
 * Starts the FreeRTOS task that continuously computes the required
 * AZ/EL and drives the actuators.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t control_start(void);

/**
 * @brief Change the tracking mode
 *
 * @param mode Desired mode (TRACK_MODE_GPS or TRACK_MODE_MANUAL)
 * @return ESP_OK on success
 */
esp_err_t control_set_mode(track_mode_t mode);

/**
 * @brief Update this station's GPS fix
 *
 * Called by the I2C GPS task whenever a new fix arrives.
 *
 * @param fix Pointer to a valid gps_fix_t (copied internally)
 * @return ESP_OK on success
 */
esp_err_t control_set_my_fix(const gps_fix_t *fix);

/**
 * @brief Set manual AZ/EL and automatically switch to MANUAL mode
 *
 * Convenience for UI or remote-control input.
 *
 * @param az_deg Desired azimuth  [0, 360)
 * @param el_deg Desired elevation (typically [0, 90])
 * @return ESP_OK on success
 */
esp_err_t control_set_manual_azel(double az_deg, double el_deg);

/**
 * @brief Store a new target GPS fix (mode unchanged)
 *
 * Used by UART, TCP, or GUI tasks to provide a target location.
 *
 * @param fix Pointer to a valid gps_fix_t
 * @return ESP_OK on success
 */
esp_err_t control_set_target_fix(const gps_fix_t *fix);

/**
 * @brief Remove the current target fix
 *
 * Useful when the target is lost or tracking should stop.
 *
 * @return ESP_OK on success
 */
esp_err_t control_clear_target_fix(void);

/**
 * @brief Update IMU orientation data
 *
 * Called by the dedicated IMU task after sensor-fusion.
 *
 * @param q         Pointer to normalised quaternion
 * @param e         Pointer to derived Euler angles (degrees)
 * @param heading   Compass heading (degrees, 0 = North)
 * @return ESP_OK on success
 */
esp_err_t control_set_imu_data(const quat_t *q,
                               const euler_t *e,
                               double heading);

/**
 * @brief Obtain a thread-safe snapshot of the whole tracker state
 *
 * Copies the internal state atomically into the user-provided struct.
 *
 * @param out_state Pointer to a caller-allocated tracker_state_t
 * @return ESP_OK on success
 */
esp_err_t control_get_state(tracker_state_t *out_state);

/**
 * @brief Compute required AZ/EL from two GPS fixes
 *
 * Uses spherical geometry; thread-safe (reads only the supplied fixes).
 *
 * @param myfix  Pointer to own position
 * @param target Pointer to target position
 * @param out    Pointer to azel_t that receives the result
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if inputs are invalid
 */
esp_err_t control_compute_azel(const gps_fix_t *myfix,
                               const gps_fix_t *target,
                               azel_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_TASK_H */

