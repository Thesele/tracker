// servo_control.c
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "servo_control.h"

static const char *TAG = "SERVO_CTRL";

/* Configuration defaults (can be overridden via servo_control_init args) */
#define SERVO_PWM_FREQ_HZ    50U   // 50 Hz -> 20 ms period (typical for servos)
#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_BIT       LEDC_TIMER_16_BIT

// Map servo IDs to LEDC channels and GPIOs (EDIT THESE to match wiring)
static const int ledc_channel_gpio[SERVO_COUNT] = {
    [SERVO_AZ] = 18,   //  AZ servo PWM pin 
    [SERVO_EL] = 19    //  EL servo PWM pin 
};

static const ledc_channel_t ledc_channel[SERVO_COUNT] = {
    [SERVO_AZ] = LEDC_CHANNEL_0,
    [SERVO_EL] = LEDC_CHANNEL_1
};

typedef struct {
    servo_config_t cfg;
    servo_pid_cfg_t pid;
    double target_deg;
    double last_target_deg;
    double measured_deg; // cached (from feedback). If no feedback: we assume equals last output.
    double integrator;
    double last_error;
    double last_command_deg; // last output angle used for rate limiting
} servo_state_t;

static servo_state_t s_state[SERVO_COUNT];
static SemaphoreHandle_t s_mutex = NULL;
static TaskHandle_t s_task = NULL;
static bool s_initialized = false;

/* ---------- Utility: mapping functions ---------- */

// clamp helper
static inline double clamp_double(double v, double a, double b) {
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

// Convert angle (deg) -> pulse (us) using linear mapping based on servo_config
static uint32_t angle_to_pulse_us(const servo_config_t *c, double angle_deg)
{
    // Linear mapping: min_deg -> min_pulse_us ; max_deg -> max_pulse_us
    // handle deg outside range by clamping
    double a = clamp_double(angle_deg, c->min_deg, c->max_deg);
    double frac = 0.0;
    if (c->max_deg == c->min_deg) frac = 0.0;
    else frac = (a - c->min_deg) / (c->max_deg - c->min_deg);
    double pulse = (double)c->min_pulse_us + frac * (double)(c->max_pulse_us - c->min_pulse_us);
    return (uint32_t) (pulse + 0.5);
}

// Convert pulse (us) -> angle (deg) inverse of above
static double pulse_us_to_angle(const servo_config_t *c, uint32_t pulse_us)
{
    double p = (double)clamp_double(pulse_us, c->min_pulse_us, c->max_pulse_us);
    double frac = (p - (double)c->min_pulse_us) / (double)(c->max_pulse_us - c->min_pulse_us);
    double angle = c->min_deg + frac * (c->max_deg - c->min_deg);
    return angle;
}

/* ---------- LEDC helpers ---------- */

// Period in microseconds
static const double pwm_period_us = 1000000.0 / (double)SERVO_PWM_FREQ_HZ;

// duty resolution (2^n - 1)
static inline uint32_t ledc_duty_max() {
    return (1u << LEDC_TIMER_BIT) - 1u;
}

// Set LEDC duty as a pulse width in microseconds
static esp_err_t ledc_set_pulse_us(ledc_channel_t ch, uint32_t pulse_us)
{
    // Convert pulse_us to duty count
    // duty = pulse_us / period_us * (2^res - 1)
    double duty_ratio = (double)pulse_us / pwm_period_us;
    if (duty_ratio < 0.0) duty_ratio = 0.0;
    if (duty_ratio > 1.0) duty_ratio = 1.0;
    uint32_t duty = (uint32_t) round(duty_ratio * (double)ledc_duty_max());

    esp_err_t err = ledc_set_duty(LEDC_MODE, ch, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_MODE, ch);
}

/* ---------- Feedback reading: MUST IMPLEMENT ---------- */

/*
 * The servo provides feedback. You must implement this function
 * for your hardware.
 *
 * It should return the current servo pulse width in microseconds
 * (500..2500 range), or a negative value (e.g. -1) if feedback is
 * unavailable or cannot be read.
 *
 * Possible implementations:
 *  - Measure feedback PWM pulse width on a GPIO using RMT or pulsein-like capture.
 *  - Read a serial/analog position sensor provided by the servo.
 *
 * For now, we provide a stub that returns -1 (indicating no feedback).
 */
__attribute__((weak))
int servo_feedback_read_us(servo_id_t id)
{
    // TODO: implement for  hardware. remember to recheck ADC parameter and find a good voltage divider circuit
    // return (int) measured_pulse_in_microseconds;
    (void)id;
    return -1;
}

/* ---------- PID control loop ---------- */

static void servo_task_fn(void *arg)
{
    const TickType_t loop_ms = 20; // 50 Hz control loop
    const double dt = (double)loop_ms / 1000.0;

    ESP_LOGI(TAG, "Servo control task started (%.0f Hz)", 1.0 / dt);

    while (1) {
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // copy states locally to minimize locked time
            servo_state_t local[SERVO_COUNT];
            memcpy(local, s_state, sizeof(local));
            xSemaphoreGive(s_mutex);

            for (int i = 0; i < SERVO_COUNT; ++i) {
                servo_state_t *st = &local[i];
                const servo_config_t *cfg = &st->cfg;

                // read feedback if available
                int fb_us = servo_feedback_read_us((servo_id_t)i);
                bool have_feedback = (fb_us > 0);
                double measured_deg = st->measured_deg;
                if (have_feedback) {
                    measured_deg = pulse_us_to_angle(cfg, (uint32_t)fb_us);
                }

                // desired target and last command
                double target_deg = st->target_deg;
                double last_cmd = st->last_command_deg;

                // Error
                double error = target_deg - measured_deg;

                // PID
                double P = st->pid.kp * error;

                // Integral with anti-windup
                st->integrator += error * dt;
                double max_int = st->pid.max_integrator;
                if (st->integrator > max_int) st->integrator = max_int;
                if (st->integrator < -max_int) st->integrator = -max_int;
                double I = st->pid.ki * st->integrator;

                // Derivative (band-limited)
                double derivative = (error - st->last_error) / dt;
                double D = st->pid.kd * derivative;

                double output_deg = P + I + D + measured_deg; // commanding absolute deg (measured_deg + correction)
                st->last_error = error;

                // Rate limiting (slew limit) based on pid.max_rate_deg_s
                double max_step = st->pid.max_rate_deg_s * dt;
                double clamped_deg = output_deg;
                if (output_deg - last_cmd > max_step) clamped_deg = last_cmd + max_step;
                if (output_deg - last_cmd < -max_step) clamped_deg = last_cmd - max_step;

                // Convert to pulse and clamp
                uint32_t pulse_us = angle_to_pulse_us(cfg, clamped_deg);
                if (pulse_us < cfg->min_pulse_us) pulse_us = cfg->min_pulse_us;
                if (pulse_us > cfg->max_pulse_us) pulse_us = cfg->max_pulse_us;

                // Write to LEDC
                esp_err_t r = ledc_set_pulse_us(ledc_channel[i], pulse_us);
                if (r != ESP_OK) {
                    ESP_LOGW(TAG, "LEDC set pulse failed ch=%d err=%d", i, (int)r);
                }

                // update local state copies (we will write back below)
                local[i].measured_deg = measured_deg;
                local[i].last_command_deg = clamped_deg;
                local[i].integrator = st->integrator;
                local[i].last_error = st->last_error;
            }

            // write back local states to shared s_state under mutex
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                for (int i = 0; i < SERVO_COUNT; ++i) {
                    s_state[i].measured_deg = local[i].measured_deg;
                    s_state[i].last_command_deg = local[i].last_command_deg;
                    s_state[i].integrator = local[i].integrator;
                    s_state[i].last_error = local[i].last_error;
                }
                xSemaphoreGive(s_mutex);
            }
        } else {
            ESP_LOGW(TAG, "servo_task: failed to acquire mutex");
        }

        vTaskDelay(pdMS_TO_TICKS(loop_ms));
    }
}

/* ---------- Public API ---------- */

esp_err_t servo_control_init(const servo_config_t configs[SERVO_COUNT],
                             const servo_pid_cfg_t pid_cfgs[SERVO_COUNT])
{
    if (s_initialized) return ESP_ERR_INVALID_STATE;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    // Setup LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_BIT,
        .timer_num = LEDC_TIMER,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", err);
        return err;
    }

    // configure each channel
    for (int i = 0; i < SERVO_COUNT; ++i) {
        // record configs
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50));
        s_state[i].cfg = configs[i];
        s_state[i].pid = pid_cfgs[i];
        s_state[i].target_deg = 0.0;
        s_state[i].last_target_deg = 0.0;
        s_state[i].measured_deg = 0.0;
        s_state[i].integrator = 0.0;
        s_state[i].last_error = 0.0;
        s_state[i].last_command_deg = 0.0;
        xSemaphoreGive(s_mutex);

        ledc_channel_config_t ch_conf = {
            .gpio_num = ledc_channel_gpio[i],
            .speed_mode = LEDC_MODE,
            .channel    = ledc_channel[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER,
            .duty       = 0
        };
        err = ledc_channel_config(&ch_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed ch=%d err=%d", i, err);
            return err;
        }

        // set initial neutral pulse
        uint32_t init_pulse = angle_to_pulse_us(&s_state[i].cfg, 0.0);
        err = ledc_set_pulse_us(ledc_channel[i], init_pulse);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "initial ledc set failed ch=%d err=%d", i, err);
        }
    }

    // start control task
    if (xTaskCreate(servo_task_fn, "servo_task", 6*1024, NULL, 6, &s_task) != pdPASS) {
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Servo control initialized");
    return ESP_OK;
}

void servo_control_deinit(void)
{
    if (!s_initialized) return;
    // Not fully implemented: ideally signal task to exit and join.
    vTaskDelete(s_task);
    s_task = NULL;
    if (s_mutex) vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    s_initialized = false;
}

esp_err_t servo_set_target_angle(servo_id_t id, double angle_deg)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (id < 0 || id >= SERVO_COUNT) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    // clamp to allowed range
    double clamped = clamp_double(angle_deg, s_state[id].cfg.min_deg, s_state[id].cfg.max_deg);
    s_state[id].target_deg = clamped;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t servo_get_target_angle(servo_id_t id, double *out_angle_deg)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!out_angle_deg) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    *out_angle_deg = s_state[id].target_deg;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t servo_get_measured_angle(servo_id_t id, double *out_angle_deg)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!out_angle_deg) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    // If feedback present, measured_deg updated by task; otherwise it equals last_command_deg
    *out_angle_deg = s_state[id].measured_deg;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t servo_force_pulse_us(servo_id_t id, uint32_t pulse_us)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (id < 0 || id >= SERVO_COUNT) return ESP_ERR_INVALID_ARG;
    // clamp
    uint32_t p = (uint32_t)clamp_double((double)pulse_us, s_state[id].cfg.min_pulse_us, s_state[id].cfg.max_pulse_us);
    esp_err_t r = ledc_set_pulse_us(ledc_channel[id], p);
    return r;
}

esp_err_t servo_set_pid(servo_id_t id, servo_pid_cfg_t cfg)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (id < 0 || id >= SERVO_COUNT) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    s_state[id].pid = cfg;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}
