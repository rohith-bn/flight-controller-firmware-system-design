/**
 * @file imu_driver.h
 * @brief Inertial Sensor Subsystem Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category B module. Coordinates sensor initialization, WHO_AM_I device verification,
 * boot-time stationary zero-rate calibration, 14-byte burst reading via imu_comm,
 * scaling via imu_converter, and real-time gyro bias cancellation.
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "qfc_types.h"
#include "imu_comm.h"
#include "imu_converter.h"
#include "gyro_calibrator.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IMU driver subsystem state structure.
 */
typedef struct {
    gyro_calibrator_t calibrator;    /**< Embedded startup bias accumulator */
    gyro_bias_t       stored_bias;   /**< Cached zero-rate bias offsets */
    imu_scaled_data_t last_sample;   /**< Most recent calibrated measurement */
    bool              b_initialized; /**< Hardware verification flag */
    bool              b_calibrated;  /**< Calibration completion flag */
} imu_driver_t;

/**
 * @brief Initializes the IMU subsystem and verifies hardware presence.
 * 
 * Invokes imu_comm_init(), checks WHO_AM_I (0x68), initializes power management
 * and sample rate divider registers, and initializes the zero-rate calibrator.
 * 
 * @param[out] p_driver Pointer to driver instance
 * @return bool true if device verified on bus, false on bus error or ID mismatch
 */
bool imu_driver_init(imu_driver_t *p_driver);

/**
 * @brief Executes one sample step of the startup gyro bias calibration sequence.
 * 
 * Reads a burst frame from the bus, scales gyro rates, and pushes the sample into
 * the calibrator. Sets *p_b_complete = true when all 512 samples are gathered.
 * 
 * @param[in,out] p_driver     Pointer to driver instance
 * @param[out]    p_b_complete Destination receiving completion status
 * @return bool true on success, false on I2C fault or excessive bench motion
 */
bool imu_driver_calibrate_step(imu_driver_t *p_driver, bool *p_b_complete);

/**
 * @brief Reads a 14-byte burst from hardware and returns bias-corrected physical units.
 * 
 * Decodes accelerometer, temperature, and gyroscope measurements. Subtracts stored
 * zero-rate gyroscope bias from angular velocities before returning.
 * 
 * @param[in,out] p_driver Pointer to driver instance
 * @param[out]    p_scaled Destination structure for calibrated physical measurements
 * @return bool true on successful read and conversion, false on I2C failure or NULL
 */
bool imu_driver_read_calibrated(imu_driver_t *p_driver, imu_scaled_data_t *p_scaled);

/**
 * @brief Retrieves stored zero-rate gyroscope bias offsets.
 * 
 * @param[in]  p_driver Pointer to driver instance
 * @param[out] p_bias   Destination structure for retrieved offsets
 * @return bool true if calibrated and copied, false otherwise
 */
bool imu_driver_get_bias(const imu_driver_t *p_driver, gyro_bias_t *p_bias);

/**
 * @brief Queries whether the sensor subsystem has completed startup calibration.
 * 
 * @param[in] p_driver Pointer to driver instance
 * @return bool true if calibrated and ready for flight
 */
bool imu_driver_is_calibrated(const imu_driver_t *p_driver);

#ifdef __cplusplus
}
#endif

#endif /* IMU_DRIVER_H */
