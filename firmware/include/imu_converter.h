/**
 * @file imu_converter.h
 * @brief Stateless Raw Count to Physical Engineering Unit Scaler
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category A module (Pure algorithmic logic). Zero internal state, zero dynamic
 * memory allocation, zero HAL dependencies.
 */

#ifndef IMU_CONVERTER_H
#define IMU_CONVERTER_H

#include "qfc_types.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decodes two big-endian register bytes into a signed 16-bit integer.
 * 
 * @param msb Most significant byte (from higher register address, e.g. ACCEL_XOUT_H)
 * @param lsb Least significant byte (from lower register address, e.g. ACCEL_XOUT_L)
 * @return int16_t Reconstructed signed 16-bit two's complement value
 */
int16_t imu_converter_bytes_to_i16(uint8_t msb, uint8_t lsb);

/**
 * @brief Scales raw accelerometer counts to acceleration in m/s^2.
 * 
 * Scaling formula: a = (raw_counts / 16384.0) * 9.80665 m/s^2
 * 
 * @param raw_counts Signed 16-bit accelerometer register value
 * @return float Acceleration in m/s^2
 */
float imu_converter_raw_to_mps2(int16_t raw_counts);

/**
 * @brief Scales raw gyroscope counts to angular velocity in deg/s.
 * 
 * Scaling formula: omega = raw_counts / 131.0 deg/s
 * 
 * @param raw_counts Signed 16-bit gyroscope register value
 * @return float Angular velocity in deg/s
 */
float imu_converter_raw_to_dps(int16_t raw_counts);

/**
 * @brief Scales raw temperature counts to degrees Celsius per MPU6050 formula.
 * 
 * Temperature in degrees C = (raw_counts / 340.0) + 36.53
 * 
 * @param raw_counts Signed 16-bit temperature register value
 * @return float Temperature in degrees Celsius
 */
float imu_converter_raw_to_celsius(int16_t raw_counts);

/**
 * @brief Scales an entire raw measurement frame to physical engineering units.
 * 
 * @param[in]  p_raw    Pointer to raw integer measurements
 * @param[out] p_scaled Pointer to destination scaled physical measurements
 * @return bool true if conversion succeeded, false if any pointer was NULL
 */
bool imu_converter_process_frame(const imu_raw_data_t *p_raw, imu_scaled_data_t *p_scaled);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CONVERTER_H */
