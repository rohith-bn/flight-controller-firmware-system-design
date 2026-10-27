/**
 * @file attitude_estimator.h
 * @brief Sensor Fusion Subsystem Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category B module. Coordinates orientation estimation on every 1 ms tick.
 * Composes stateless angle_computer trigonometry with discrete complementary_filter
 * sensor fusion. Monitors roll and pitch angles for unrecoverable divergence (> +/- 90 deg).
 */

#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "qfc_types.h"
#include "angle_computer.h"
#include "complementary_filter.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief State structure for attitude estimator subsystem.
 */
typedef struct {
    complementary_filter_t filter;           /**< Embedded discrete complementary filter instance */
    attitude_deg_t         current_attitude; /**< Latest filtered orientation estimate */
    bool                   b_initialized;    /**< Subsystem initialization flag */
} attitude_estimator_t;

/**
 * @brief Initializes the attitude estimator subsystem and inner filter.
 * 
 * @param[out] p_est Pointer to attitude estimator instance
 * @param[in]  alpha Filter weighting coefficient [0.0, 1.0] (typical: 0.98f)
 * @param[in]  dt_s  Sample period in seconds (typical: 0.001f)
 * @return bool true on success, false on invalid arguments
 */
bool attitude_estimator_init(attitude_estimator_t *p_est, float alpha, float dt_s);

/**
 * @brief Resets the attitude estimate back to level hover (0.0, 0.0 deg).
 * 
 * @param[in,out] p_est Pointer to attitude estimator instance
 * @return bool true on success, false if p_est is NULL or uninitialized
 */
bool attitude_estimator_reset(attitude_estimator_t *p_est);

/**
 * @brief Executes one discrete orientation estimation cycle.
 * 
 * Pipeline:
 * 1. Computes static angles via angle_computer from accelerometer vector.
 * 2. Fuses with calibrated gyro angular rates via complementary_filter.
 * 3. Checks for attitude divergence (> +/- 90 degrees). If diverged, returns false.
 * 
 * @param[in,out] p_est         Pointer to attitude estimator instance
 * @param[in]     p_accel_mps2  Pointer to 3-axis calibrated acceleration vector
 * @param[in]     p_gyro_dps    Pointer to 3-axis calibrated angular rate vector
 * @param[out]    p_att_out     Destination structure for filtered Euler angles
 * @return bool true if valid attitude computed, false on NULL, free fall, or divergence (> 90 deg)
 */
bool attitude_estimator_update(attitude_estimator_t *p_est,
                               const vector3f_t     *p_accel_mps2,
                               const vector3f_t     *p_gyro_dps,
                               attitude_deg_t       *p_att_out);

/**
 * @brief Retrieves the latest valid attitude estimate.
 * 
 * @param[in]  p_est     Pointer to attitude estimator instance
 * @param[out] p_att_out Destination structure for Euler angles
 * @return bool true on success, false if pointers are NULL or uninitialized
 */
bool attitude_estimator_get_attitude(const attitude_estimator_t *p_est, attitude_deg_t *p_att_out);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_ESTIMATOR_H */
