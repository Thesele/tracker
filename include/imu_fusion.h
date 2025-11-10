// #ifndef IMU_FUSION_H
// #define IMU_FUSION_H

// #include "driver/i2c_master.h"
// #include "esp_err.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// // I2C addresses
// #define ICM20948_I2C_ADDR     0x68     // AD0 = low
// #define AK09916_I2C_ADDR      0x0C     // Internal mag

// // Public handles (created in init)
// extern i2c_master_bus_handle_t imu_bus_handle;
// extern i2c_master_dev_handle_t imu_dev;

// // Initialization
// esp_err_t imu_i2c_init_full(i2c_port_t i2c_port, int sda_gpio, int scl_gpio, uint32_t freq_hz);
// esp_err_t icm20948_init(void);     // config accel/gyro + enable I2C master + setup mag
// esp_err_t ak09916_init(void);      // places AK09916 in continuous mode via ICM’s I2C master

// // Raw reads
// esp_err_t imu_read_accel(float *ax_g, float *ay_g, float *az_g);
// esp_err_t imu_read_gyro(float *gx_dps, float *gy_dps, float *gz_dps);
// esp_err_t imu_read_mag(float *mx_uT, float *my_uT, float *mz_uT);

// // Math utilities
// void imu_compute_tilt_from_accel(float ax_g, float ay_g, float az_g, float *pitch_deg, float *roll_deg);
// float imu_compute_heading_deg(float mx_uT, float my_uT, float mz_uT, float pitch_deg, float roll_deg);

// // Filters and fusion
// void imu_update_ma_filter(float pitch_deg, float roll_deg, float *f_pitch_deg, float *f_roll_deg);
// void imu_complementary_fusion(float ax_g, float ay_g, float az_g,
//                               float gx_dps, float gy_dps, float gz_dps,
//                               float dt_s,
//                               float *fused_pitch_deg, float *fused_roll_deg, float *fused_yaw_deg);

// // Az/El correction
// void imu_correct_az_el(float raw_az_deg, float raw_el_deg,
//                        float fused_pitch_deg, float fused_roll_deg,
//                        float *az_out_deg, float *el_out_deg);

// // Task (1 Hz GUI updates)
// void imu_fusion_task(void *arg);

// #endif // IMU_FUSION_H