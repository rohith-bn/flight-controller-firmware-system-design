/**
 * @file angle_computer.c
 * @brief Stateless Accelerometer Geometric Orientation Calculator Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "angle_computer.h"
#include <math.h>
#include <stddef.h>

/* Minimum squared magnitude to avoid free-fall / zero-G division anomaly (0.1 m/s^2)^2 */
#define MIN_ACCEL_MAG_SQ  (0.01f)

float angle_computer_compute_roll(float ay, float az) {
    /* phi = atan2f(ay, az) * (180.0 / PI) */
    return atan2f(ay, az) * QFC_RAD_TO_DEG;
}

float angle_computer_compute_pitch(float ax, float ay, float az) {
    /* theta = atan2f(-ax, sqrt(ay^2 + az^2)) * (180.0 / PI) */
    float denom = sqrtf((ay * ay) + (az * az));
    return atan2f(-ax, denom) * QFC_RAD_TO_DEG;
}

bool angle_computer_process(const vector3f_t *p_accel, attitude_deg_t *p_attitude) {
    if ((p_accel == NULL) || (p_attitude == NULL)) {
        return false;
    }

    /* Compute total vector acceleration magnitude squared */
    float mag_sq = (p_accel->x * p_accel->x) +
                   (p_accel->y * p_accel->y) +
                   (p_accel->z * p_accel->z);

    if (mag_sq < MIN_ACCEL_MAG_SQ) {
        /* Reject free-fall or near-zero gravity singularity */
        p_attitude->roll_deg = 0.0f;
        p_attitude->pitch_deg = 0.0f;
        return false;
    }

    p_attitude->roll_deg = angle_computer_compute_roll(p_accel->y, p_accel->z);
    p_attitude->pitch_deg = angle_computer_compute_pitch(p_accel->x, p_accel->y, p_accel->z);

    return true;
}
