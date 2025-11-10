// #include "imu_fusion.h"
// #include "esp_log.h"
// #include <math.h>
// #include <string.h>

// static const char *TAG = "IMU_FUSION";

// // ===== ESP-IDF I2C handles =====
// i2c_master_bus_handle_t imu_bus_handle;
// i2c_master_dev_handle_t imu_dev;

// // ===== Filters =====
// #define MA_FILTER_SIZE 10
// static float pitch_buf[MA_FILTER_SIZE] = {0};
// static float roll_buf[MA_FILTER_SIZE]  = {0};
// static int   ma_index = 0;

// // ===== ICM-20948 register/banks =====
// #define REG_BANK_SEL          0x7F
// #define BANK0                 0x00
// #define BANK1                 0x10
// #define BANK2                 0x20
// #define BANK3                 0x30

// // Bank 0
// #define WHO_AM_I              0x00
// #define PWR_MGMT_1            0x06
// #define USER_CTRL             0x03
// #define INT_PIN_CFG           0x0F
// #define I2C_MST_STATUS        0x17
// #define EXT_SENS_DATA_00      0x3B
// #define ACCEL_XOUT_H          0x2D
// #define GYRO_XOUT_H           0x33

// // Bank 2 (Accel/Gyro config)
// #define GYRO_SMPLRT_DIV      0x00
// #define GYRO_CONFIG_1        0x01
// #define GYRO_CONFIG_2        0x02
// #define ACCEL_SMPLRT_DIV_1   0x10
// #define ACCEL_SMPLRT_DIV_2   0x11
// #define ACCEL_INTEL_CTRL     0x12
// #define ACCEL_CONFIG         0x14
// #define ACCEL_CONFIG_2       0x15

// // Bank 3 (I2C master)
// #define I2C_MST_CTRL          0x01
// #define I2C_MST_ODR_CFG       0x00
// #define I2C_SLV0_ADDR         0x03
// #define I2C_SLV0_REG          0x04
// #define I2C_SLV0_CTRL         0x05
// #define I2C_SLV1_ADDR         0x07
// #define I2C_SLV1_REG          0x08
// #define I2C_SLV1_CTRL         0x09
// #define I2C_SLV1_DO           0x06

// // AK09916 registers
// #define AK09916_WHO_AM_I      0x01
// #define AK09916_ST1           0x10
// #define AK09916_HXL           0x11
// #define AK09916_CNTL2         0x31
// #define AK09916_CNTL3         0x32

// // Scales (default config set below)
// static float accel_scale_g = 1.0f / 16384.0f;   // ±2g
// static float gyro_scale_dps = 1.0f / 131.0f;    // ±250 dps
// static float mag_scale_uT = 0.15f;              // AK09916 ~0.15 uT/LSB

// // Complementary fusion gains
// static const float CF_ALPHA = 0.02f; // accel trust (tilt slow)
// static const float CF_BETA  = 0.98f; // gyro trust (tilt fast)

// // ===== Low-level I2C helpers =====
// static inline esp_err_t write_reg(uint8_t reg, uint8_t val) {
//     uint8_t tx[2] = {reg, val};
//     return i2c_master_transmit(imu_dev, tx, sizeof(tx), -1);
// }

// static inline esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
//     return i2c_master_transmit_receive(imu_dev, &reg, 1, buf, len, -1);
// }

// static esp_err_t set_bank(uint8_t bank) {
//     return write_reg(REG_BANK_SEL, bank);
// }

// // ===== Init sequence =====
// esp_err_t imu_i2c_init_full(i2c_port_t i2c_port, int sda_gpio, int scl_gpio, uint32_t freq_hz)
// {
//     i2c_master_bus_config_t bus_cfg = {
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .i2c_port = i2c_port,
//         .sda_io_num = sda_gpio,
//         .scl_io_num = scl_gpio,
//         .glitch_ignore_cnt = 7,
//         .flags = { .enable_internal_pullup = 1 }
//     };
//     ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &imu_bus_handle), TAG, "bus create failed");

//     i2c_device_config_t dev_cfg = {
//         .dev_addr_length = I2C_ADDR_BIT_7,
//         .device_address  = ICM20948_I2C_ADDR,
//         .scl_speed_hz    = freq_hz
//     };
//     ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(imu_bus_handle, &dev_cfg, &imu_dev), TAG, "add dev failed");

//     ESP_LOGI(TAG, "I2C master ready for ICM-20948 at 0x%02X", ICM20948_I2C_ADDR);
//     return ESP_OK;
// }

// esp_err_t icm20948_init(void)
// {
//     // Reset & wake
//     ESP_RETURN_ON_ERROR(set_bank(BANK0), TAG, "bank0 fail");
//     ESP_RETURN_ON_ERROR(write_reg(PWR_MGMT_1, 0x80), TAG, "reset fail"); // DEVICE_RESET
//     vTaskDelay(pdMS_TO_TICKS(50));
//     ESP_RETURN_ON_ERROR(write_reg(PWR_MGMT_1, 0x01), TAG, "clock select fail"); // AUTO clock
//     vTaskDelay(pdMS_TO_TICKS(10));

//     // Enable I2C master, disable FIFO
//     ESP_RETURN_ON_ERROR(write_reg(USER_CTRL, 0x20), TAG, "I2C_MST_EN fail");

//     // Bypass disabled (we use internal I2C master route)
//     ESP_RETURN_ON_ERROR(write_reg(INT_PIN_CFG, 0x30), TAG, "INT pin cfg fail"); // latch, etc.

//     // Configure gyro/accel (Bank 2): 250 dps, ±2g, DLPF on, sample rate dividers
//     ESP_RETURN_ON_ERROR(set_bank(BANK2), TAG, "bank2 fail");

//     // Gyro: DLPF enable, DLPF=4 (~21.2 Hz), FS=±250dps
//     ESP_RETURN_ON_ERROR(write_reg(GYRO_CONFIG_1, 0x11), TAG, "gyro cfg1 fail"); // [GYRO_DLPF=1][GYRO_FS_SEL=0][DLPF_CFG=1]
//     ESP_RETURN_ON_ERROR(write_reg(GYRO_SMPLRT_DIV, 0x04), TAG, "gyro smpl div fail"); // SR = base/(1+div). Choose ~225 Hz base => ~45 Hz

//     // Accel: DLPF enable, DLPF=4 (~23.9 Hz), FS=±2g
//     ESP_RETURN_ON_ERROR(write_reg(ACCEL_CONFIG, 0x11), TAG, "accel cfg fail"); // [ACCEL_DLPF=1][ACCEL_FS_SEL=0][DLPF_CFG=1]
//     ESP_RETURN_ON_ERROR(write_reg(ACCEL_SMPLRT_DIV_1, 0x00), TAG, "accel div1 fail");
//     ESP_RETURN_ON_ERROR(write_reg(ACCEL_SMPLRT_DIV_2, 0x04), TAG, "accel div2 fail"); // ~45 Hz

//     // Back to Bank 0
//     ESP_RETURN_ON_ERROR(set_bank(BANK0), TAG, "bank0 fail 2");

//     ESP_LOGI(TAG, "ICM-20948 accel/gyro configured");
//     return ESP_OK;
// }

// // ===== AK09916 via ICM-20948 internal I2C master =====
// // We use SLV1 to write control registers, and SLV0 to continuously read 8 bytes starting at ST1.
// static esp_err_t i2c_master_write_mag_reg(uint8_t reg, uint8_t val)
// {
//     // Bank3: SLV1_ADDR (write), SLV1_REG, SLV1_DO, SLV1_CTRL (length=1, enable)
//     ESP_RETURN_ON_ERROR(set_bank(BANK3), TAG, "bank3 fail");
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV1_ADDR, (AK09916_I2C_ADDR << 1) | 0x00), TAG, "slv1 addr fail");
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV1_REG, reg), TAG, "slv1 reg fail");
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV1_DO, val), TAG, "slv1 do fail");
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV1_CTRL, 0x81), TAG, "slv1 ctrl fail"); // enable, length=1
//     // Small delay for write to complete
//     vTaskDelay(pdMS_TO_TICKS(10));
//     // Disable SLV1 transaction to avoid continuous writes
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV1_CTRL, 0x00), TAG, "slv1 disable fail");
//     // Back to Bank0
//     ESP_RETURN_ON_ERROR(set_bank(BANK0), TAG, "bank0 fail");
//     return ESP_OK;
// }

// esp_err_t ak09916_init(void)
// {
//     // Enable ICM’s I2C master controller
//     ESP_RETURN_ON_ERROR(set_bank(BANK3), TAG, "bank3 fail");
//     ESP_RETURN_ON_ERROR(write_reg(I2C_MST_CTRL, 0x17), TAG, "mst ctrl fail"); // I2C master clk (~345 kHz), multi-master off
//     ESP_RETURN_ON_ERROR(write_reg(I2C_MST_ODR_CFG, 0x03), TAG, "mst odr fail"); // Update EXT_SENS at ~55 Hz

//     // Configure SLV0 for continuous read: start at ST1, length = 8 bytes (ST1 + HXL..HZH + ST2)
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV0_ADDR, (AK09916_I2C_ADDR << 1) | 0x01), TAG, "slv0 addr fail"); // read
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV0_REG, AK09916_ST1), TAG, "slv0 reg fail");
//     ESP_RETURN_ON_ERROR(write_reg(I2C_SLV0_CTRL, 0x88), TAG, "slv0 ctrl fail"); // enable, length=8

//     // Back to Bank0
//     ESP_RETURN_ON_ERROR(set_bank(BANK0), TAG, "bank0 fail");

//     // Place AK09916 into continuous mode 4 (100 Hz) or 2 (20 Hz). We’ll use mode 2 for stability.
//     ESP_RETURN_ON_ERROR(i2c_master_write_mag_reg(AK09916_CNTL3, 0x01), TAG, "soft reset fail");
//     vTaskDelay(pdMS_TO_TICKS(10));
//     ESP_RETURN_ON_ERROR(i2c_master_write_mag_reg(AK09916_CNTL2, 0x08), TAG, "set mode fail"); // 0x08 => CONTINUOUS MODE 2 (~20Hz)

//     ESP_LOGI(TAG, "AK09916 magnetometer configured via ICM internal I2C master");
//     return ESP_OK;
// }

// // ===== Raw reads =====
// esp_err_t imu_read_accel(float *ax_g, float *ay_g, float *az_g)
// {
//     uint8_t raw[6];
//     ESP_RETURN_ON_ERROR(read_regs(ACCEL_XOUT_H, raw, sizeof(raw)), TAG, "accel read fail");
//     int16_t ax = (raw[0] << 8) | raw[1];
//     int16_t ay = (raw[2] << 8) | raw[3];
//     int16_t az = (raw[4] << 8) | raw[5];
//     *ax_g = ax * accel_scale_g;
//     *ay_g = ay * accel_scale_g;
//     *az_g = az * accel_scale_g;
//     return ESP_OK;
// }

// esp_err_t imu_read_gyro(float *gx_dps, float *gy_dps, float *gz_dps)
// {
//     uint8_t raw[6];
//     ESP_RETURN_ON_ERROR(read_regs(GYRO_XOUT_H, raw, sizeof(raw)), TAG, "gyro read fail");
//     int16_t gx = (raw[0] << 8) | raw[1];
//     int16_t gy = (raw[2] << 8) | raw[3];
//     int16_t gz = (raw[4] << 8) | raw[5];
//     *gx_dps = gx * gyro_scale_dps;
//     *gy_dps = gy * gyro_scale_dps;
//     *gz_dps = gz * gyro_scale_dps;
//     return ESP_OK;
// }

// esp_err_t imu_read_mag(float *mx_uT, float *my_uT, float *mz_uT)
// {
//     // Data appears in EXT_SENS_DATA_00.. after SLV0 read setup
//     uint8_t buf[8];
//     ESP_RETURN_ON_ERROR(read_regs(EXT_SENS_DATA_00, buf, sizeof(buf)), TAG, "mag ext read fail");

//     // buf[0] = ST1, ensure data ready bit
//     if ((buf[0] & 0x01) == 0) {
//         // Not ready; keep last or return ESP_ERR_INVALID_STATE
//         return ESP_ERR_INVALID_STATE;
//     }

//     int16_t mx = (int16_t)((buf[2] << 8) | buf[1]); // HXL, HXH are little-endian in AK09916
//     int16_t my = (int16_t)((buf[4] << 8) | buf[3]);
//     int16_t mz = (int16_t)((buf[6] << 8) | buf[5]);
//     // buf[7] = ST2 (overflow flag bit 3). You may check & handle overflows.

//     *mx_uT = mx * mag_scale_uT;
//     *my_uT = my * mag_scale_uT;
//     *mz_uT = mz * mag_scale_uT;
//     return ESP_OK;
// }

// // ===== Math utilities =====
// void imu_compute_tilt_from_accel(float ax_g, float ay_g, float az_g, float *pitch_deg, float *roll_deg)
// {
//     // Pitch: rotation about X; Roll: rotation about Y (choose consistent convention)
//     *pitch_deg = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;
//     *roll_deg  = atan2f(ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 180.0f / M_PI;
// }

// float imu_compute_heading_deg(float mx_uT, float my_uT, float mz_uT, float pitch_deg, float roll_deg)
// {
//     float pitch = pitch_deg * (M_PI / 180.0f);
//     float roll  = roll_deg  * (M_PI / 180.0f);

//     // Tilt compensation (NED-style). Adjust if your axis mapping differs.
//     float mx_comp = mx_uT * cosf(pitch) + mz_uT * sinf(pitch);
//     float my_comp = mx_uT * sinf(roll) * sinf(pitch) + my_uT * cosf(roll) - mz_uT * sinf(roll) * cosf(pitch);

//     float heading = atan2f(-my_comp, mx_comp) * 180.0f / M_PI; // negative sign aligns to compass convention
//     if (heading < 0) heading += 360.0f;
//     return heading;
// }

// // ===== Filters and fusion =====
// void imu_update_ma_filter(float pitch_deg, float roll_deg, float *f_pitch_deg, float *f_roll_deg)
// {
//     pitch_buf[ma_index] = pitch_deg;
//     roll_buf[ma_index]  = roll_deg;
//     ma_index = (ma_index + 1) % MA_FILTER_SIZE;

//     float sp = 0, sr = 0;
//     for (int i = 0; i < MA_FILTER_SIZE; i++) { sp += pitch_buf[i]; sr += roll_buf[i]; }
//     *f_pitch_deg = sp / MA_FILTER_SIZE;
//     *f_roll_deg  = sr / MA_FILTER_SIZE;
// }

// // Lightweight complementary filter for tilt and yaw
// void imu_complementary_fusion(float ax_g, float ay_g, float az_g,
//                               float gx_dps, float gy_dps, float gz_dps,
//                               float dt_s,
//                               float *fused_pitch_deg, float *fused_roll_deg, float *fused_yaw_deg)
// {
//     float pitch_acc, roll_acc;
//     imu_compute_tilt_from_accel(ax_g, ay_g, az_g, &pitch_acc, &roll_acc);

//     // Integrate gyro
//     float pitch_rate = gx_dps; // depending on axis mapping; verify orientation
//     float roll_rate  = gy_dps;
//     float yaw_rate   = gz_dps;

//     // Complementary fusion for tilt
//     float pitch_est = (*fused_pitch_deg) + pitch_rate * dt_s;
//     float roll_est  = (*fused_roll_deg)  + roll_rate  * dt_s;

//     *fused_pitch_deg = CF_BETA * pitch_est + CF_ALPHA * pitch_acc;
//     *fused_roll_deg  = CF_BETA * roll_est  + CF_ALPHA * roll_acc;
//     *fused_yaw_deg   = fmodf((*fused_yaw_deg) + yaw_rate * dt_s + 360.0f, 360.0f);
// }

// // ===== Az/El correction =====
// void imu_correct_az_el(float raw_az_deg, float raw_el_deg,
//                        float fused_pitch_deg, float fused_roll_deg,
//                        float *az_out_deg, float *el_out_deg)
// {
//     // Simple plane-based correction: roll biases azimuth; pitch biases elevation.
//     *az_out_deg = fmodf(raw_az_deg + fused_roll_deg + 360.0f, 360.0f);
//     float el = raw_el_deg - fused_pitch_deg;
//     if (el < 0) el = 0;
//     if (el > 90) el = 90;
//     *el_out_deg = el;
// }

// // ===== Task =====
// void imu_fusion_task(void *arg)
// {
//     // Initial fused states
//     float fused_pitch = 0.0f, fused_roll = 0.0f, fused_yaw = 0.0f;

//     // Example raw az/el from your target solver — replace with real values per update
//     float raw_az = 180.0f, raw_el = 45.0f;

//     // Init sequence
//     if (icm20948_init() != ESP_OK) {
//         ESP_LOGE(TAG, "ICM-20948 init failed");
//         vTaskDelete(NULL);
//     }
//     if (ak09916_init() != ESP_OK) {
//         ESP_LOGE(TAG, "AK09916 init failed");
//         vTaskDelete(NULL);
//     }

//     TickType_t last = xTaskGetTickCount();
//     while (1) {
//         float ax, ay, az, gx, gy, gz, mx, my, mz;

//         // Read sensors
//         if (imu_read_accel(&ax, &ay, &az) == ESP_OK &&
//             imu_read_gyro(&gx, &gy, &gz) == ESP_OK) {

//             // dt based on 1 Hz loop (but compute precisely)
//             TickType_t now = xTaskGetTickCount();
//             float dt = (float)(now - last) / 1000.0f * portTICK_PERIOD_MS;
//             last = now;

//             // Fusion (complementary)
//             imu_complementary_fusion(ax, ay, az, gx, gy, gz, dt, &fused_pitch, &fused_roll, &fused_yaw);

//             // Optional moving average smoothing for GUI
//             float f_pitch_ma, f_roll_ma;
//             imu_update_ma_filter(fused_pitch, fused_roll, &f_pitch_ma, &f_roll_ma);

//             // Magnetometer read (non-fatal if not ready)
//             if (imu_read_mag(&mx, &my, &mz) == ESP_OK) {
//                 float heading = imu_compute_heading_deg(mx, my, mz, f_pitch_ma, f_roll_ma);
//                 fused_yaw = heading; // Bias yaw to mag heading occasionally; for stronger stability, low-pass this assignment.
//             }

//             float az_corr, el_corr;
//             imu_correct_az_el(raw_az, raw_el, f_pitch_ma, f_roll_ma, &az_corr, &el_corr);

//             ESP_LOGI(TAG, "Tilt P/R: %.2f / %.2f | Yaw: %.2f | Az/El: %.2f / %.2f",
//                      f_pitch_ma, f_roll_ma, fused_yaw, az_corr, el_corr);

//             // TODO: send to Python GUI (UART/Wi-Fi)
//             // gui_send_tilt(f_pitch_ma, f_roll_ma);
//             // gui_send_heading(fused_yaw);
//             // gui_send_az_el(az_corr, el_corr);
//         }

//         vTaskDelay(pdMS_TO_TICKS(1000)); // Match GUI 1 Hz
//     }
// }