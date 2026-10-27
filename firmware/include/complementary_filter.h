/**
 * @file complementary_filter.h
 * @brief Discrete-Time Complementary Orientation Filter
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category A module. Fuses high-frequency angular rate integration (gyroscope)
 * with low-frequency absolute gravitational reference (accelerometer) using
 * a first-order complementary filter:
 * 
 * theta = alpha * (theta_prev + omega * dt) + (1.0 - alpha) * theta_accel
 */

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "qfc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Default filter weighting constant (98% gyro trust, 2% accel trust) */
#define COMPLEMENTARY_FILTER_DEFAULT_ALPHA   (0.98f)

/**
 * @brief Internal state structure for the complementary filter.
 * 
 * Fully static memory footprint. Must be initialized via complementary_filter_init().
 */
typedef struct {
    float roll_deg;      /**< Current filtered roll angle estimate in degrees */
    float pitch_deg;     /**< Current filtered pitch angle estimate in degrees */
    float alpha;         /**< Dynamic weighting parameter [0.0, 1.0] */
    float dt_s;          /**< Time step in seconds (nominal: 0.001 s) */
    bool  b_initialized; /**< Initialization status flag */
} complementary_filter_t;

/**
 * @brief Initializes the complementary filter structure with tuning parameters.
 * 
 * @param[out] p_filter Pointer to filter instance to initialize
 * @param[in]  alpha    Filter coefficient (0.0 to 1.0, typical: 0.98f)
 * @param[in]  dt_s     Sample period in seconds (typical: 0.001f)
 * @return bool true if initialized successfully, false on invalid arguments
 */
bool complementary_filter_init(complementary_filter_t *p_filter, float alpha, float dt_s);

/**
 * @brief Resets or pre-loads the filter angles (e.g. from static accelerometer at boot).
 * 
 * @param[in,out] p_filter      Pointer to filter instance
 * @param[in]     initial_roll  Starting roll angle in degrees
 * @param[in]     initial_pitch Starting pitch angle in degrees
 * @return bool true if reset successfully, false if p_filter is NULL
 */
bool complementary_filter_reset(complementary_filter_t *p_filter, float initial_roll, float initial_pitch);

/**
 * @brief Executes one discrete filter update step.
 * 
 * @param[in,out] p_filter     Pointer to filter instance
 * @param[in]     p_accel_att  Pointer to static accelerometer roll/pitch angles
 * @param[in]     p_gyro_dps   Pointer to calibrated angular velocity (deg/s)
 * @param[out]    p_att_out    Pointer to structure receiving filtered output attitude
 * @return bool true on success, false if any pointer is NULL or filter uninitialized
 */
bool complementary_filter_update(complementary_filter_t *p_filter,
                                const attitude_deg_t   *p_accel_att,
                                const vector3f_t       *p_gyro_dps,
                                attitude_deg_t         *p_att_out);

/**
 * @brief Retrieves the current estimated attitude without running an update.
 * 
 * @param[in]  p_filter  Pointer to filter instance
 * @param[out] p_att_out Pointer to structure receiving current attitude
 * @return bool true on success, false if pointer is NULL
 */
bool complementary_filter_get_attitude(const complementary_filter_t *p_filter,
                                      attitude_deg_t               *p_att_out);

#ifdef __cplusplus
}
#endif

#endif /* COMPLEMENTARY_FILTER_H */
