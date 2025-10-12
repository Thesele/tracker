#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdbool.h>

#define SERVO_COUNT 2
#define SERVO_AZ 0
#define SERVO_EL 1

typedef struct {
    uint32_t min_pulse_us;
    uint32_t neutral_pulse_us;
    uint32_t max_pulse_us;
    float min_deg;
    float max_deg;
    gpio_num_t pwm_pin;
    adc_channel_t fb_channel;   // ADC channel for feedback (if used)
} servo_config_t;

typedef struct {
    servo_config_t cfg;
    float current_angle;     // Last measured (feedback) angle
    float target_angle;      // Desired angle
    bool feedback_enabled;   // Whether feedback is being used
    float fb_min_v;          // Calibration: ADC voltage at max angle
    float fb_max_v;          // Calibration: ADC voltage at min angle
} servo_state_t;

esp_err_t servo_control_init(const servo_config_t *configs);
esp_err_t servo_set_angle(int id, float angle_deg);
float servo_get_feedback_angle(int id);
void servo_enable_feedback(int id, bool enable);
esp_err_t servo_calibrate_feedback(int id, float fb_min_v, float fb_max_v);
float servo_Az_to_angle(float az);
void servo_control_task(void *pvParameters);

#endif // SERVO_CONTROL_H
