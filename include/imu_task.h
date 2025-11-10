/* imu_task.h
 *
 * Public API for the ICM20948 IMU task.
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "esp_err.h"

// I2C pins
#define I2C_MASTER_SCL_IO           GPIO_NUM_32
#define I2C_MASTER_SDA_IO           GPIO_NUM_33
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// IMU interrupt pin
#define IMU_INT_PIN                 GPIO_NUM_39
#define IMU_INT_PIN_SEL             (1ULL << IMU_INT_PIN)


/**
 * @brief Starts the IMU task.
 * This task will initialize I2C, the ICM20948, and its DMP.
 * It will then handle interrupts and update the central control state.
 */
esp_err_t imu_task_start(void);


#endif // IMU_TASK_H
