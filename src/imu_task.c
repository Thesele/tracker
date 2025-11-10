#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "imu_task.h"
#include "control_task.h"

#include "icm20948.h"
#include "icm20948_i2c.h"
#include "icm20948_registers.h"
#include "icm20948_enumerations.h"
#include "ak09916_registers.h"
#include "ak09916_enumerations.h"

static const char *TAG = "IMU_TASK";



/* ========= Fusion / calibration constants ========= */
#define DT_SEC                 0.02f     /* 50 Hz loop */
#define ALPHA_RP               0.98f     /* roll/pitch complementary blend */
#define ALPHA_YAW              0.78f     /* yaw complementary blend (more stable) */
#define MAG_LPF_ALPHA          0.90f     /* 0=no smoothing, 0.9 strong smoothing */

#define ACCEL_SENS             16384.0f  /* ±2 g */
#define GYRO_SENS              131.0f    /* ±250 dps */

#define DEG2RAD(x)             ((x) * (float)M_PI / 180.0f)
#define RAD2DEG(x)             ((x) * 180.0f / (float)M_PI)

/* Set your magnetic declination (degrees). Cape Town ~ +24° E. */
#ifndef DECLINATION_DEG
#define DECLINATION_DEG        24.0f
#endif

/* ========= Device config ========= */
static icm20948_device_t g_icm;
static icm0948_config_i2c_t g_icm_cfg = {
    .i2c_port = I2C_MASTER_NUM,
    .i2c_addr = ICM_20948_I2C_ADDR_AD1   /* 0x69 */
};

/* ========= Bias / calibration store ========= */
typedef struct {
    /* Gyro biases [deg/s] */
    float gx_b;
    float gy_b;
    float gz_b;

    /* Magnetometer calibration (hard-iron offsets + simple axis scales) */
    float mx_off;
    float my_off;
    float mz_off;
    float mx_s;
    float my_s;
    float mz_s;
} imu_bias_t;

static imu_bias_t g_bias = {
    .gx_b = 0.0f, .gy_b = 0.0f, .gz_b = 0.0f,
    .mx_off = 0.0f, .my_off = 0.0f, .mz_off = 0.0f,
    .mx_s = 1.0f, .my_s = 1.0f, .mz_s = 1.0f
};

/* ========= I2C driver ========= */
static esp_err_t i2c_driver_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %d", err);
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "I2C driver ready: SDA=%d SCL=%d freq=%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

/* ========= Accel/Gyro bring-up (±2g, ±250 dps, wide BW) ========= */
static bool bringup_accel_gyro(icm20948_device_t *dev)
{
    icm20948_status_e st;

    st = icm20948_sw_reset(dev);
    vTaskDelay(pdMS_TO_TICKS(250));

    st |= icm20948_sleep(dev, false);
    st |= icm20948_low_power(dev, false);

    icm20948_internal_sensor_id_bm sns =
        (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

    st |= icm20948_set_sample_mode(dev, sns, SAMPLE_MODE_CONTINUOUS);

    icm20948_fss_t fss;
    fss.a = GPM_2;
    fss.g = DPS_250;
    st |= icm20948_set_full_scale(dev, sns, fss);

    icm20948_dlpcfg_t dlpf;
    dlpf.a = ACC_D473BW_N499BW;          /* effectively “off” */
    dlpf.g = GYR_D361BW4_N376BW5;        /* effectively “off” */
    st |= icm20948_set_dlpf_cfg(dev, sns, dlpf);

    st |= icm20948_enable_dlpf(dev, ICM_20948_INTERNAL_ACC, false);
    st |= icm20948_enable_dlpf(dev, ICM_20948_INTERNAL_GYR, false);

    if (st != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Accel/Gyro setup failed");
        return false;
    }

    ESP_LOGI(TAG, "ICM-20948 awake. Full scale: ±2 g, ±250 dps.");
    return true;
}

/* ========= Internal I2C master + AK09916 @ 100 Hz ========= */
static bool bringup_master_and_mag(icm20948_device_t *dev)
{
    icm20948_status_e st;

    st = icm20948_i2c_master_enable(dev, true);
    if (st != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "icm20948_i2c_master_enable(true) failed");
        return false;
    }
    ESP_LOGI(TAG, "Internal I2C master enabled.");

    /* AK09916 soft reset */
    uint8_t rst = 0x01;
    st = icm20948_i2c_master_single_w(dev, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, &rst);
    if (st != ICM_20948_STAT_OK) {
        ESP_LOGW(TAG, "AK09916 CNTL3 reset write failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* WHO_AM_I check */
    uint8_t wia1 = 0, wia2 = 0;
    st = icm20948_i2c_master_single_r(dev, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA1, &wia1);
    if (st != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "AK09916 WIA1 read failed");
        return false;
    }
    st = icm20948_i2c_master_single_r(dev, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &wia2);
    if (st != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "AK09916 WIA2 read failed");
        return false;
    }
    ESP_LOGI(TAG, "AK09916 ID: 0x%02X 0x%02X (expect 0x48 0x09)", wia1, wia2);

    if (!((wia1 == 0x48) && (wia2 == 0x09))) {
        ESP_LOGE(TAG, "Magnetometer WHO_AM_I mismatch");
        return false;
    }

    /* Continuous 100 Hz */
    uint8_t mode = (uint8_t)AK09916_MODE_CONT_100_HZ;
    st = icm20948_i2c_master_single_w(dev, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, &mode);
    if (st != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "AK09916 CNTL2 write failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* SLV0: read 9 bytes from ST1 each internal-master cycle */
    st = icm20948_i2c_controller_configure_peripheral(
            dev,
            0,                               /* SLV0 */
            MAG_AK09916_I2C_ADDR,            /* 0x0C */
            AK09916_REG_ST1,                 /* start at ST1 */
            9,                               /* ST1, HxL, HxH, HyL, HyH, HzL, HzH, dummy, ST2 */
            true,                            /* read */
            true,                            /* enable */
            false,                           /* data_only (REG_DIS)=0 */
            false,                           /* GRP=0 */
            false,                           /* BYTE_SW=0 */
            0                                /* dataOut ignored for read */
    );
    if (st != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "SLV0 configure (ST1,9) failed");
        return false;
    }

    ESP_LOGI(TAG, "AK09916 ready (CONT_100_HZ, SLV0 ST1..9).");
    return true;
}

/* ========= Gyro bias calibration (2 s still) ========= */
static void calibrate_gyro_bias(icm20948_device_t *dev)
{
    ESP_LOGI(TAG, "Calibrating gyro… keep IMU still (2 s)");
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(2000);

    double gx = 0.0, gy = 0.0, gz = 0.0;
    uint32_t n = 0;

    while (xTaskGetTickCount() < end) {
        icm20948_agmt_t agmt;
        if (icm20948_get_agmt(dev, &agmt) == ICM_20948_STAT_OK) {
            gx += agmt.gyr.axes.x;
            gy += agmt.gyr.axes.y;
            gz += agmt.gyr.axes.z;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (n == 0) {
        n = 1;
    }

    g_bias.gx_b = (float)(gx / n) / GYRO_SENS;
    g_bias.gy_b = (float)(gy / n) / GYRO_SENS;
    g_bias.gz_b = (float)(gz / n) / GYRO_SENS;

    ESP_LOGI(TAG, "Gyro bias [dps]: %.3f %.3f %.3f",
             g_bias.gx_b, g_bias.gy_b, g_bias.gz_b);
}

/* ========= Magnetometer min/max (6 s sweep) ========= */
static void calibrate_mag_minmax(icm20948_device_t *dev)
{
    ESP_LOGI(TAG, "Mag calibration: rotate slowly in all directions (6 s)");
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(6000);

    /* Track min/max on the **aligned** mag axes (after remap) */
    int16_t minx =  32767, miny =  32767, minz =  32767;
    int16_t maxx = -32768, maxy = -32768, maxz = -32768;

    while (xTaskGetTickCount() < end) {
        icm20948_agmt_t agmt;
        if (icm20948_get_agmt(dev, &agmt) == ICM_20948_STAT_OK) {
            /* AK09916 → body remap:
               mx_body = +My_raw, my_body = +Mx_raw, mz_body = −Mz_raw */
            int16_t mx_r = agmt.mag.axes.y;
            int16_t my_r = agmt.mag.axes.x;
            int16_t mz_r = (int16_t)(-agmt.mag.axes.z);

            if (mx_r < minx) { minx = mx_r; }
            if (mx_r > maxx) { maxx = mx_r; }
            if (my_r < miny) { miny = my_r; }
            if (my_r > maxy) { maxy = my_r; }
            if (mz_r < minz) { minz = mz_r; }
            if (mz_r > maxz) { maxz = mz_r; }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* Hard-iron offsets */
    g_bias.mx_off = 0.5f * (float)(maxx + minx);
    g_bias.my_off = 0.5f * (float)(maxy + miny);
    g_bias.mz_off = 0.5f * (float)(maxz + minz);

    /* Simple soft-iron scaling (equalize radii) */
    float rx = 0.5f * (float)(maxx - minx);
    float ry = 0.5f * (float)(maxy - miny);
    float rz = 0.5f * (float)(maxz - minz);
    float ravg = (rx + ry + rz) / 3.0f;

    if (rx > 1.0f) { g_bias.mx_s = ravg / rx; }
    else           { g_bias.mx_s = 1.0f; }

    if (ry > 1.0f) { g_bias.my_s = ravg / ry; }
    else           { g_bias.my_s = 1.0f; }

    if (rz > 1.0f) { g_bias.mz_s = ravg / rz; }
    else           { g_bias.mz_s = 1.0f; }

    ESP_LOGI(TAG, "Mag off: %.1f %.1f %.1f | scale: %.3f %.3f %.3f",
             g_bias.mx_off, g_bias.my_off, g_bias.mz_off,
             g_bias.mx_s,  g_bias.my_s,  g_bias.mz_s);
}

/* ========= The IMU task ========= */
static void imu_task_fn(void *arg)
{
    ESP_LOGI(TAG, "IMU task starting…");

    if (i2c_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        vTaskDelete(NULL);
        return;
    }

    icm20948_init_i2c(&g_icm, &g_icm_cfg);

    if (icm20948_check_id(&g_icm) != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "ICM20948 ID check failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "ICM20948 ID OK.");

    if (bringup_accel_gyro(&g_icm) == false) {
        ESP_LOGE(TAG, "Accel/Gyro bring-up failed");
        vTaskDelete(NULL);
        return;
    }

    if (bringup_master_and_mag(&g_icm) == false) {
        ESP_LOGE(TAG, "Magnetometer bring-up failed");
        vTaskDelete(NULL);
        return;
    }

    /* One-time calibrations */
    calibrate_gyro_bias(&g_icm);
    calibrate_mag_minmax(&g_icm);

    /* Complementary filter state (radians) */
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;

    /* Declination (radians) */
    const float decl = DEG2RAD(DECLINATION_DEG);

    /* Magnetometer LPF state */
    float mx_lpf = 0.0f, my_lpf = 0.0f, mz_lpf = 0.0f;
    bool  lpf_init = false;

    const TickType_t period = pdMS_TO_TICKS(20);
    uint32_t log_div = 0;

    ESP_LOGI(TAG, "Entering run loop…");
    while (true) {
        vTaskDelay(period);

        icm20948_agmt_t agmt;
        icm20948_status_e st = icm20948_get_agmt(&g_icm, &agmt);
        if (st != ICM_20948_STAT_OK) {
            ESP_LOGW(TAG, "icm20948_get_agmt failed: %d", st);
            continue;
        }

        /* Convert accel & gyro to physical units */
        float ax = (float)agmt.acc.axes.x / ACCEL_SENS;
        float ay = (float)agmt.acc.axes.y / ACCEL_SENS;
        float az = (float)agmt.acc.axes.z / ACCEL_SENS;

        float gx = ((float)agmt.gyr.axes.x / GYRO_SENS) - g_bias.gx_b;
        float gy = ((float)agmt.gyr.axes.y / GYRO_SENS) - g_bias.gy_b;
        float gz = ((float)agmt.gyr.axes.z / GYRO_SENS) - g_bias.gz_b;

        /* ==== Magnetometer axis remap (AK09916 → body) ==== 
           mx = +My_raw;  my = +Mx_raw;  mz = −Mz_raw   */
        float mx_raw = (float)agmt.mag.axes.y;
        float my_raw = (float)agmt.mag.axes.x;
        float mz_raw = (float)(-agmt.mag.axes.z);

        /* Apply hard-iron & simple soft-iron */
        float mx_c = (mx_raw - g_bias.mx_off) * g_bias.mx_s;
        float my_c = (my_raw - g_bias.my_off) * g_bias.my_s;
        float mz_c = (mz_raw - g_bias.mz_off) * g_bias.mz_s;

        /* LPF the magnetometer a bit to stabilize heading */
        if (lpf_init == false) {
            mx_lpf = mx_c;
            my_lpf = my_c;
            mz_lpf = mz_c;
            lpf_init = true;
        } else {
            mx_lpf = MAG_LPF_ALPHA * mx_lpf + (1.0f - MAG_LPF_ALPHA) * mx_c;
            my_lpf = MAG_LPF_ALPHA * my_lpf + (1.0f - MAG_LPF_ALPHA) * my_c;
            mz_lpf = MAG_LPF_ALPHA * mz_lpf + (1.0f - MAG_LPF_ALPHA) * mz_c;
        }

        /* Normalize accel to stabilize atan2 when not exactly 1 g */
        float anorm = sqrtf(ax*ax + ay*ay + az*az);
        if ((anorm > 0.3f) && (anorm < 1.8f)) {
            ax /= anorm;
            ay /= anorm;
            az /= anorm;
        }

        /* Accel-derived roll & pitch (radians) */
        float roll_acc  = atan2f( ay, az );
        float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

        /* Integrate gyro (deg/s → rad/s) */
        roll  += DEG2RAD(gx) * DT_SEC;
        pitch += DEG2RAD(gy) * DT_SEC;
        yaw   += DEG2RAD(gz) * DT_SEC;

        /* Complementary filter for roll/pitch */
        roll  = ALPHA_RP * roll  + (1.0f - ALPHA_RP) * roll_acc;
        pitch = ALPHA_RP * pitch + (1.0f - ALPHA_RP) * pitch_acc;

        /* Tilt-compensated heading from mag (radians) */
        float cr = cosf(roll);
        float sr = sinf(roll);
        float cp = cosf(pitch);
        float sp = sinf(pitch);

        float Xh =  mx_lpf * cp + my_lpf * sr * sp + mz_lpf * cr * sp;
        float Yh =  my_lpf * cr - mz_lpf * sr;

        float yaw_mag = atan2f(-Yh, Xh) + decl;
        if (yaw_mag < 0.0f) {
            yaw_mag += 2.0f * (float)M_PI;
        } else if (yaw_mag >= 2.0f * (float)M_PI) {
            yaw_mag -= 2.0f * (float)M_PI;
        }

        /* Complementary yaw: shortest-path correction */
        float err = yaw_mag - yaw;
        if (err > (float)M_PI)  { err -= 2.0f * (float)M_PI; }
        if (err < -(float)M_PI) { err += 2.0f * (float)M_PI; }
        yaw += (1.0f - ALPHA_YAW) * err;

        /* Wrap yaw */
        if (yaw < 0.0f) {
            yaw += 2.0f * (float)M_PI;
        } else if (yaw >= 2.0f * (float)M_PI) {
            yaw -= 2.0f * (float)M_PI;
        }

        /* Output in degrees */
        euler_t e;
        e.roll  = RAD2DEG(roll);
        e.pitch = RAD2DEG(pitch);
        e.yaw   = RAD2DEG(yaw);

        control_set_imu_data(NULL, &e, e.yaw);

        /* Logs ~5 Hz */
        log_div += 1u;
        if (log_div >= 10u) {
            ESP_LOGI(TAG,
                     "R:%.1f P:%.1f Y:%.1f | "
                     "A:(%.2f %.2f %.2f) | "
                     "G:(%.1f %.1f %.1f) | "
                     "M:(%.0f %.0f %.0f)",
                     e.roll, e.pitch, e.yaw,
                     ax, ay, az,
                     gx, gy, gz,
                     mx_lpf, my_lpf, mz_lpf);
            log_div = 0u;
        }
    }
}

/* ========= Public API ========= */
esp_err_t imu_task_start(void)
{
    BaseType_t ok = xTaskCreate(imu_task_fn, "imu_task", 6*1024, NULL, 7, NULL);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
