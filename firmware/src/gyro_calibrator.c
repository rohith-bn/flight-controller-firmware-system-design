/**
 * @file gyro_calibrator.c
 * @brief Startup Zero-Rate Gyroscope Bias Characterization Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "gyro_calibrator.h"
#include <math.h>
#include <stddef.h>

bool gyro_calibrator_init(gyro_calibrator_t *p_cal) {
    if (p_cal == NULL) {
        return false;
    }

    p_cal->sample_count               = 0U;
    p_cal->sum_x                      = 0.0;
    p_cal->sum_y                      = 0.0;
    p_cal->sum_z                      = 0.0;
    p_cal->calculated_bias.x_dps      = 0.0f;
    p_cal->calculated_bias.y_dps      = 0.0f;
    p_cal->calculated_bias.z_dps      = 0.0f;
    p_cal->b_complete                 = false;
    p_cal->b_motion_fault             = false;

    return true;
}

bool gyro_calibrator_reset(gyro_calibrator_t *p_cal) {
    return gyro_calibrator_init(p_cal);
}

bool gyro_calibrator_push_sample(gyro_calibrator_t *p_cal, const vector3f_t *p_sample_dps) {
    if ((p_cal == NULL) || (p_sample_dps == NULL)) {
        return false;
    }

    /* Reject any sample if motion fault has been latched */
    if (p_cal->b_motion_fault) {
        return false;
    }

    /* Already complete: retain computed bias */
    if (p_cal->b_complete) {
        return true;
    }

    /* Stationary bench motion verification */
    if ((fabsf(p_sample_dps->x) > GYRO_CALIBRATOR_MAX_MOTION_DPS) ||
        (fabsf(p_sample_dps->y) > GYRO_CALIBRATOR_MAX_MOTION_DPS) ||
        (fabsf(p_sample_dps->z) > GYRO_CALIBRATOR_MAX_MOTION_DPS)) {
        p_cal->b_motion_fault = true;
        return false;
    }

    /* Accumulate sample in 64-bit double precision */
    p_cal->sum_x += (double)p_sample_dps->x;
    p_cal->sum_y += (double)p_sample_dps->y;
    p_cal->sum_z += (double)p_sample_dps->z;
    p_cal->sample_count++;

    /* Once required sample count is reached, compute zero-rate offset averages */
    if (p_cal->sample_count >= QFC_GYRO_CALIBRATION_SAMPLES) {
        double count_d = (double)QFC_GYRO_CALIBRATION_SAMPLES;
        p_cal->calculated_bias.x_dps = (float)(p_cal->sum_x / count_d);
        p_cal->calculated_bias.y_dps = (float)(p_cal->sum_y / count_d);
        p_cal->calculated_bias.z_dps = (float)(p_cal->sum_z / count_d);
        p_cal->b_complete            = true;
    }

    return true;
}

bool gyro_calibrator_is_complete(const gyro_calibrator_t *p_cal) {
    if ((p_cal == NULL) || (p_cal->b_motion_fault)) {
        return false;
    }

    return p_cal->b_complete;
}

bool gyro_calibrator_get_bias(const gyro_calibrator_t *p_cal, gyro_bias_t *p_bias) {
    if ((p_cal == NULL) || (p_bias == NULL)) {
        return false;
    }

    if (!p_cal->b_complete || p_cal->b_motion_fault) {
        return false;
    }

    *p_bias = p_cal->calculated_bias;
    return true;
}
