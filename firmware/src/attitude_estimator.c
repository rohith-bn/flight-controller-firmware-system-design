/**
 * @file attitude_estimator.c
 * @brief Sensor Fusion Subsystem Coordinator Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "attitude_estimator.h"
#include <math.h>
#include <stddef.h>

bool attitude_estimator_init(attitude_estimator_t *p_est, float alpha, float dt_s) {
    if (p_est == NULL) {
        return false;
    }

    if (!complementary_filter_init(&p_est->filter, alpha, dt_s)) {
        return false;
    }

    p_est->current_attitude.roll_deg  = 0.0f;
    p_est->current_attitude.pitch_deg = 0.0f;
    p_est->b_initialized              = true;

    return true;
}

bool attitude_estimator_reset(attitude_estimator_t *p_est) {
    if ((p_est == NULL) || (!p_est->b_initialized)) {
        return false;
    }

    if (!complementary_filter_reset(&p_est->filter, 0.0f, 0.0f)) {
        return false;
    }

    p_est->current_attitude.roll_deg  = 0.0f;
    p_est->current_attitude.pitch_deg = 0.0f;

    return true;
}

bool attitude_estimator_update(attitude_estimator_t *p_est,
                               const vector3f_t     *p_accel_mps2,
                               const vector3f_t     *p_gyro_dps,
                               attitude_deg_t       *p_att_out) {
    if ((p_est == NULL) || (p_accel_mps2 == NULL) || (p_gyro_dps == NULL) || (p_att_out == NULL)) {
        return false;
    }

    if (!p_est->b_initialized) {
        return false;
    }

    /* 1. Compute static accelerometer geometric angles */
    attitude_deg_t accel_att;
    if (!angle_computer_process(p_accel_mps2, &accel_att)) {
        return false;
    }

    /* 2. Execute complementary filter fusion */
    if (!complementary_filter_update(&p_est->filter, &accel_att, p_gyro_dps, &p_est->current_attitude)) {
        return false;
    }

    /* 3. Safety divergence threshold monitoring (+/- 90 degrees) */
    if ((fabsf(p_est->current_attitude.roll_deg) > QFC_DIVERGENCE_THRESHOLD_DEG) ||
        (fabsf(p_est->current_attitude.pitch_deg) > QFC_DIVERGENCE_THRESHOLD_DEG)) {
        return false;
    }

    /* 4. Output filtered estimate */
    *p_att_out = p_est->current_attitude;
    return true;
}

bool attitude_estimator_get_attitude(const attitude_estimator_t *p_est, attitude_deg_t *p_att_out) {
    if ((p_est == NULL) || (p_att_out == NULL) || (!p_est->b_initialized)) {
        return false;
    }

    *p_att_out = p_est->current_attitude;
    return true;
}
