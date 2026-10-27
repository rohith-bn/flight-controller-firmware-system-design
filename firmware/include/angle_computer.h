/**
 * @file angle_computer.h
 * @brief Stateless Accelerometer Geometric Orientation Calculator
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category A module. Computes Euler roll and pitch angles from static
 * gravity acceleration vectors using standard aeronautical trigonometry.
 */

#ifndef ANGLE_COMPUTER_H
#define ANGLE_COMPUTER_H

#include "qfc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Computes static roll angle (degrees) from accelerometer measurements.
 * 
 * Formula: phi = atan2f(ay, az) * (180.0 / PI)
 * 
 * @param ay Acceleration along body Y axis (m/s^2)
 * @param az Acceleration along body Z axis (m/s^2)
 * @return float Roll angle in degrees [-180.0, +180.0]
 */
float angle_computer_compute_roll(float ay, float az);

/**
 * @brief Computes static pitch angle (degrees) from accelerometer measurements.
 * 
 * Formula: theta = atan2f(-ax, sqrt(ay^2 + az^2)) * (180.0 / PI)
 * 
 * @param ax Acceleration along body X axis (m/s^2)
 * @param ay Acceleration along body Y axis (m/s^2)
 * @param az Acceleration along body Z axis (m/s^2)
 * @return float Pitch angle in degrees [-90.0, +90.0]
 */
float angle_computer_compute_pitch(float ax, float ay, float az);

/**
 * @brief Computes both roll and pitch angles from an acceleration vector.
 * 
 * Performs defensive sanity checks on vector norm. If total acceleration is
 * near zero (< 0.1 m/s^2, e.g. free fall), returns false and defaults to (0, 0).
 * 
 * @param[in]  p_accel    Pointer to 3-axis acceleration vector (m/s^2)
 * @param[out] p_attitude Pointer to attitude output structure
 * @return bool true if angles computed successfully, false if pointer is NULL or in free fall
 */
bool angle_computer_process(const vector3f_t *p_accel, attitude_deg_t *p_attitude);

#ifdef __cplusplus
}
#endif

#endif /* ANGLE_COMPUTER_H */
