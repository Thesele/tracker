#include "servo_control.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "SERVO_CTRL";
static servo_state_t servos[SERVO_COUNT];
static esp_adc_cal_characteristics_t adc_chars;

// ========== Initialization ==========
esp_err_t servo_control_init(const servo_config_t *configs)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,  // 20 ms period
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (int i = 0; i < SERVO_COUNT; i++) {
        servos[i].cfg = configs[i];
        servos[i].feedback_enabled = false;
        servos[i].current_angle = 0;
        servos[i].target_angle = 0;

        ledc_channel_config_t ch_cfg = {
            .gpio_num = configs[i].pwm_pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

    // ADC setup for feedback (shared calibration)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // default, update per servo
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);

    ESP_LOGI(TAG, "Servo control initialized");
    return ESP_OK;
}

// ========== PWM Control ==========
float servo_Az_to_angle(float az){
    float wrapped = fmodf(az, 360.0f);  // Ensure az is within 0–360
    if (wrapped > 180.0f)
        return wrapped - 360.0f;        // Map to negative range
    else
        return -wrapped;                // Clockwise is negative
}
esp_err_t servo_set_angle(int id, float angle_deg)
{
    if (id < 0 || id >= SERVO_COUNT)
        return ESP_ERR_INVALID_ARG;

    servo_config_t *cfg = &servos[id].cfg;
    float clamped = angle_deg;
    if (clamped > cfg->max_deg) clamped = cfg->max_deg;
    if (clamped < cfg->min_deg) clamped = cfg->min_deg;

    // Map angle → pulse width
    float ratio = (clamped - cfg->min_deg) / (cfg->max_deg - cfg->min_deg);
    uint32_t pulse_width = cfg->min_pulse_us +
                           ratio * (cfg->max_pulse_us - cfg->min_pulse_us);

    // Convert to LEDC duty (assuming 50 Hz)
    uint32_t duty = (pulse_width * (1 << 16)) / 20000;  // 20ms period
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, id, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, id));

    servos[id].target_angle = clamped;
    return ESP_OK;
}

// ========== Feedback Reading ==========
float servo_get_feedback_angle(int id)
{
    if (!servos[id].feedback_enabled)
        return servos[id].target_angle;

    uint32_t adc_read = adc1_get_raw(servos[id].cfg.fb_channel);
    float voltage = esp_adc_cal_raw_to_voltage(adc_read, &adc_chars) / 1000.0f;

    // Clamp within calibration range
    float min_v = servos[id].fb_min_v;
    float max_v = servos[id].fb_max_v;
    if (voltage < min_v) voltage = min_v;
    if (voltage > max_v) voltage = max_v;

    // Map voltage → angle (inverse relation)
    float angle = servos[id].cfg.max_deg -
                  (voltage - min_v) * (servos[id].cfg.max_deg - servos[id].cfg.min_deg) / (max_v - min_v);

    servos[id].current_angle = angle;
    return angle;
}

// ========== Enable / Calibrate Feedback ==========
void servo_enable_feedback(int id, bool enable)
{
    if (id < 0 || id >= SERVO_COUNT) return;
    servos[id].feedback_enabled = enable;
    ESP_LOGI(TAG, "Feedback %s for servo %d", enable ? "enabled" : "disabled", id);
}

esp_err_t servo_calibrate_feedback(int id, float fb_min_v, float fb_max_v)
{
    if (id < 0 || id >= SERVO_COUNT) return ESP_ERR_INVALID_ARG;
    servos[id].fb_min_v = fb_min_v;
    servos[id].fb_max_v = fb_max_v;
    ESP_LOGI(TAG, "Calibrated servo %d feedback range: %.2fV - %.2fV", id, fb_min_v, fb_max_v);
    return ESP_OK;
}

// ========== Control Task ==========
void servo_control_task(void *pvParameters)
{
    const float kp = 0.1f;
    const float tol = 1.5f;  // allowable error in degrees

    while (1) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (!servos[i].feedback_enabled) continue;

            float current = servo_get_feedback_angle(i);
            float error = servos[i].target_angle - current;

            if (fabsf(error) > tol) {
                float correction = kp * error;
                servo_set_angle(i, current + correction);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // control loop every 100 ms
    }
}
