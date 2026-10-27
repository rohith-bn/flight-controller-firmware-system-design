/**
 * @file gyro_calibrator.h
 * @brief Startup Zero-Rate Gyroscope Bias Characterization
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category A module. Collects N = 512 resting angular velocity samples during boot,
 * accumulates high-precision 64-bit sums to prevent numerical overflow, verifies that
 * the vehicle is stationary (motion detection threshold), and calculates the zero-rate
 * offset bias for subtraction in the IMU driver.
 */

#ifndef GYRO_CALIBRATOR_H
#define GYRO_CALIBRATOR_H

#include "qfc_types.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum tolerable angular velocity while calibrating stationary on bench (deg/s) */
#define GYRO_CALIBRATOR_MAX_MOTION_DPS   (10.0f)

/**
 * @brief Gyroscope calibrator state accumulator instance.
 */
typedef struct {
    uint16_t    sample_count;    /**< Number of samples gathered so far [0, 512] */
    double      sum_x;           /**< 64-bit floating accumulator for X axis */
    double      sum_y;           /**< 64-bit floating accumulator for Y axis */
    double      sum_z;           /**< 64-bit floating accumulator for Z axis */
    gyro_bias_t calculated_bias; /**< Stored calculated zero-rate bias offsets */
    bool        b_complete;      /**< True once all 512 samples have been processed */
    bool        b_motion_fault;  /**< True if vehicle motion was detected during calibration */
} gyro_calibrator_t;

/**
 * @brief Initializes the calibrator state and zeroes accumulators.
 * 
 * @param[out] p_cal Pointer to calibrator instance
 * @return bool true on success, false if p_cal is NULL
 */
bool gyro_calibrator_init(gyro_calibrator_t *p_cal);

/**
 * @brief Resets sample count and accumulators to begin a new calibration run.
 * 
 * @param[in,out] p_cal Pointer to calibrator instance
 * @return bool true on success, false if p_cal is NULL
 */
bool gyro_calibrator_reset(gyro_calibrator_t *p_cal);

/**
 * @brief Pushes a new resting angular velocity sample into the calibrator.
 * 
 * If sample rate exceeds GYRO_CALIBRATOR_MAX_MOTION_DPS, a motion fault is latched.
 * When sample_count reaches QFC_GYRO_CALIBRATION_SAMPLES (512), mean bias is computed.
 * 
 * @param[in,out] p_cal          Pointer to calibrator instance
 * @param[in]     p_sample_dps   Pointer to single-axis angular rate sample in deg/s
 * @return bool true if sample accepted, false on NULL or motion fault
 */
bool gyro_calibrator_push_sample(gyro_calibrator_t *p_cal, const vector3f_t *p_sample_dps);

/**
 * @brief Queries whether calibration is finished.
 * 
 * @param[in] p_cal Pointer to calibrator instance
 * @return bool true if 512 valid samples have been processed without fault
 */
bool gyro_calibrator_is_complete(const gyro_calibrator_t *p_cal);

/**
 * @brief Retrieves the computed zero-rate gyroscope bias offsets.
 * 
 * @param[in]  p_cal  Pointer to calibrator instance
 * @param[out] p_bias Destination structure for computed bias offsets
 * @return bool true if calibration was complete and bias copied, false otherwise
 */
bool gyro_calibrator_get_bias(const gyro_calibrator_t *p_cal, gyro_bias_t *p_bias);

#ifdef __cplusplus
}
#endif

#endif /* GYRO_CALIBRATOR_H */
