/**
 * @file imu_comm.h
 * @brief Virtual API Hardware Interface for MPU6050 I2C Communication
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category C hardware boundary seam. Isolates physical I2C transactions
 * from the high-level imu_driver using the Virtual API pattern. Allows 100%
 * host-based unit testing via test doubles without physical target hardware.
 */

#ifndef IMU_COMM_H
#define IMU_COMM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* MPU6050 REGISTER DEFINITIONS                                               */
/* ========================================================================== */

#define MPU6050_I2C_ADDR_DEFAULT         (0x68U)

#define MPU6050_REG_SMPLRT_DIV           (0x19U)
#define MPU6050_REG_CONFIG               (0x1AU)
#define MPU6050_REG_GYRO_CONFIG          (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG         (0x1CU)
#define MPU6050_REG_ACCEL_XOUT_H         (0x3BU)
#define MPU6050_REG_PWR_MGMT_1           (0x6BU)
#define MPU6050_REG_WHO_AM_I             (0x75U)

#define MPU6050_WHO_AM_I_DEFAULT_VAL     (0x68U)
#define MPU6050_BURST_DATA_LEN           (14U) /**< 6 Accel + 2 Temp + 6 Gyro */

/* ========================================================================== */
/* HARDWARE INTERFACE FUNCTION PROTOTYPES                                     */
/* ========================================================================== */

/**
 * @brief Initializes the physical or simulated I2C bus peripheral.
 * 
 * @return bool true on success, false on bus initialization failure
 */
bool imu_comm_init(void);

/**
 * @brief Reads the WHO_AM_I register and verifies device identity.
 * 
 * @param expected_id Expected device identifier (typically 0x68)
 * @return bool true if read succeeded and matched expected_id, false otherwise
 */
bool imu_comm_check_whoami(uint8_t expected_id);

/**
 * @brief Performs a burst read transaction from sequential registers.
 * 
 * Reads 14 contiguous bytes starting from ACCEL_XOUT_H (0x3B) through
 * GYRO_ZOUT_L (0x48).
 * 
 * @param[in]  start_reg Starting register address (e.g. MPU6050_REG_ACCEL_XOUT_H)
 * @param[out] p_buf     Destination buffer for received bytes
 * @param[in]  len       Number of bytes to read
 * @return bool true on transaction success, false on I2C NACK or bus timeout
 */
bool imu_comm_read_burst(uint8_t start_reg, uint8_t *p_buf, uint8_t len);

/**
 * @brief Writes a single byte to a device register.
 * 
 * @param reg Register address to write
 * @param val Value byte to write
 * @return bool true on success, false on I2C bus error
 */
bool imu_comm_write_reg(uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* IMU_COMM_H */
