/**
 * @file complementary_filter.c
 * @brief Discrete-Time Complementary Orientation Filter Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "complementary_filter.h"
#include <stddef.h>

bool complementary_filter_init(complementary_filter_t *p_filter, float alpha, float dt_s) {
    if (p_filter == NULL) {
        return false;
    }

    if ((alpha < 0.0f) || (alpha > 1.0f) || (dt_s <= 0.0f)) {
        return false;
    }

    p_filter->roll_deg      = 0.0f;
    p_filter->pitch_deg     = 0.0f;
    p_filter->alpha         = alpha;
    p_filter->dt_s          = dt_s;
    p_filter->b_initialized = true;

    return true;
}

bool complementary_filter_reset(complementary_filter_t *p_filter, float initial_roll, float initial_pitch) {
    if ((p_filter == NULL) || (!p_filter->b_initialized)) {
        return false;
    }

    p_filter->roll_deg  = initial_roll;
    p_filter->pitch_deg = initial_pitch;

    return true;
}

bool complementary_filter_update(complementary_filter_t *p_filter,
                                const attitude_deg_t   *p_accel_att,
                                const vector3f_t       *p_gyro_dps,
                                attitude_deg_t         *p_att_out) {
    if ((p_filter == NULL) || (p_accel_att == NULL) || (p_gyro_dps == NULL) || (p_att_out == NULL)) {
        return false;
    }

    if (!p_filter->b_initialized) {
        return false;
    }

    /* 1. Integrate angular rate over sample period dt */
    float roll_gyro  = p_filter->roll_deg  + (p_gyro_dps->x * p_filter->dt_s);
    float pitch_gyro = p_filter->pitch_deg + (p_gyro_dps->y * p_filter->dt_s);

    /* 2. Fuse with static gravitational reference angle */
    float alpha = p_filter->alpha;
    float one_minus_alpha = 1.0f - alpha;

    p_filter->roll_deg  = (alpha * roll_gyro)  + (one_minus_alpha * p_accel_att->roll_deg);
    p_filter->pitch_deg = (alpha * pitch_gyro) + (one_minus_alpha * p_accel_att->pitch_deg);

    /* 3. Output filtered attitude */
    p_att_out->roll_deg  = p_filter->roll_deg;
    p_att_out->pitch_deg = p_filter->pitch_deg;

    return true;
}

bool complementary_filter_get_attitude(const complementary_filter_t *p_filter,
                                      attitude_deg_t               *p_att_out) {
    if ((p_filter == NULL) || (p_att_out == NULL) || (!p_filter->b_initialized)) {
        return false;
    }

    p_att_out->roll_deg  = p_filter->roll_deg;
    p_att_out->pitch_deg = p_filter->pitch_deg;

    return true;
}
