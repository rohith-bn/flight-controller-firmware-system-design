/**
 * @file quad_physics.c
 * @brief 6-DOF Rigid-Body Quadrotor Flight Dynamics Simulation Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 Compliance
 */

#include "quad_physics.h"
#include <math.h>
#include <stddef.h>

#define MATH_SQRT2_INV     (0.70710678118654752440f)

bool quad_physics_init(quad_physics_t *p_quad, float init_roll_deg, float init_pitch_deg, float dt_s) {
    if ((p_quad == NULL) || (dt_s <= 0.0f)) {
        return false;
    }

    p_quad->roll_deg  = init_roll_deg;
    p_quad->pitch_deg = init_pitch_deg;
    p_quad->yaw_deg   = 0.0f;

    p_quad->p_dps = 0.0f;
    p_quad->q_dps = 0.0f;
    p_quad->r_dps = 0.0f;

    p_quad->dist_torque_roll_nm  = 0.0f;
    p_quad->dist_torque_pitch_nm = 0.0f;

    p_quad->time_s = 0.0f;
    p_quad->dt_s   = dt_s;

    /* Compute initial gravity reaction vector */
    float phi   = init_roll_deg * QFC_DEG_TO_RAD;
    float theta = init_pitch_deg * QFC_DEG_TO_RAD;

    p_quad->accel_body_mps2.x = -9.80665f * sinf(theta);
    p_quad->accel_body_mps2.y =  9.80665f * sinf(phi) * cosf(theta);
    p_quad->accel_body_mps2.z =  9.80665f * cosf(phi) * cosf(theta);

    return true;
}

void quad_physics_apply_disturbance(quad_physics_t *p_quad, float roll_torque_nm, float pitch_torque_nm) {
    if (p_quad != NULL) {
        p_quad->dist_torque_roll_nm  = roll_torque_nm;
        p_quad->dist_torque_pitch_nm = pitch_torque_nm;
    }
}

bool quad_physics_step(quad_physics_t *p_quad, const motor_commands_t *p_motors) {
    if ((p_quad == NULL) || (p_motors == NULL)) {
        return false;
    }

    /* 1. Calculate actuator torque generation across Quad-X arm geometry */
    float arm_effective_m = QUAD_ARM_LENGTH_M * MATH_SQRT2_INV;
    float k_tau = arm_effective_m * QUAD_MAX_MOTOR_THRUST_N;

    /* Quad-X torque mapping consistent with motor_mixer negative feedback */
    float m_diff_roll  = (p_motors->m1 + p_motors->m3) - (p_motors->m2 + p_motors->m4);
    float m_diff_pitch = (p_motors->m3 + p_motors->m4) - (p_motors->m1 + p_motors->m2);

    float tau_roll  = (k_tau * m_diff_roll) + p_quad->dist_torque_roll_nm;
    float tau_pitch = (k_tau * m_diff_pitch) + p_quad->dist_torque_pitch_nm;

    /* 2. Incorporate aerodynamic angular rate damping */
    float p_rad = p_quad->p_dps * QFC_DEG_TO_RAD;
    float q_rad = p_quad->q_dps * QFC_DEG_TO_RAD;

    tau_roll  -= (QUAD_AERO_DAMPING_COEFF * p_rad);
    tau_pitch -= (QUAD_AERO_DAMPING_COEFF * q_rad);

    /* 3. Angular accelerations (Euler's rotational dynamics: alpha = tau / I) */
    float alpha_p_rad_s2 = tau_roll / QUAD_INERTIA_IXX;
    float alpha_q_rad_s2 = tau_pitch / QUAD_INERTIA_IYY;

    /* 4. Integrate angular velocities */
    p_quad->p_dps += (alpha_p_rad_s2 * QFC_RAD_TO_DEG * p_quad->dt_s);
    p_quad->q_dps += (alpha_q_rad_s2 * QFC_RAD_TO_DEG * p_quad->dt_s);

    /* 5. Integrate kinematic Euler angle rates */
    float phi   = p_quad->roll_deg * QFC_DEG_TO_RAD;
    float theta = p_quad->pitch_deg * QFC_DEG_TO_RAD;

    /* Avoid singularity at exact 90 deg pitch */
    float cos_theta = cosf(theta);
    if (fabsf(cos_theta) < 0.001f) {
        cos_theta = 0.001f;
    }

    float phi_dot   = p_quad->p_dps + (p_quad->q_dps * sinf(phi) * tanf(theta));
    float theta_dot = p_quad->q_dps * cosf(phi);

    p_quad->roll_deg  += (phi_dot * p_quad->dt_s);
    p_quad->pitch_deg += (theta_dot * p_quad->dt_s);

    /* 6. Synthesize body accelerometer reaction to gravity */
    phi   = p_quad->roll_deg * QFC_DEG_TO_RAD;
    theta = p_quad->pitch_deg * QFC_DEG_TO_RAD;

    p_quad->accel_body_mps2.x = -9.80665f * sinf(theta);
    p_quad->accel_body_mps2.y =  9.80665f * sinf(phi) * cosf(theta);
    p_quad->accel_body_mps2.z =  9.80665f * cosf(phi) * cosf(theta);

    /* 7. Advance time */
    p_quad->time_s += p_quad->dt_s;

    return true;
}

void quad_physics_get_raw_imu_counts(const quad_physics_t *p_quad,
                                     int16_t *p_ax_raw, int16_t *p_ay_raw, int16_t *p_az_raw,
                                     int16_t *p_gx_raw, int16_t *p_gy_raw, int16_t *p_gz_raw) {
    if ((p_quad == NULL) || (p_ax_raw == NULL) || (p_ay_raw == NULL) || (p_az_raw == NULL) ||
        (p_gx_raw == NULL) || (p_gy_raw == NULL) || (p_gz_raw == NULL)) {
        return;
    }

    /* Convert m/s^2 to 16-bit counts: count = (accel_mps2 / 9.80665) * 16384 */
    float ax_counts = (p_quad->accel_body_mps2.x / 9.80665f) * QFC_MPU6050_ACCEL_LSB_PER_G;
    float ay_counts = (p_quad->accel_body_mps2.y / 9.80665f) * QFC_MPU6050_ACCEL_LSB_PER_G;
    float az_counts = (p_quad->accel_body_mps2.z / 9.80665f) * QFC_MPU6050_ACCEL_LSB_PER_G;

    /* Convert deg/s to 16-bit counts: count = dps * 131 */
    float gx_counts = p_quad->p_dps * QFC_MPU6050_GYRO_LSB_PER_DPS;
    float gy_counts = p_quad->q_dps * QFC_MPU6050_GYRO_LSB_PER_DPS;
    float gz_counts = p_quad->r_dps * QFC_MPU6050_GYRO_LSB_PER_DPS;

    *p_ax_raw = (int16_t)roundf(ax_counts);
    *p_ay_raw = (int16_t)roundf(ay_counts);
    *p_az_raw = (int16_t)roundf(az_counts);

    *p_gx_raw = (int16_t)roundf(gx_counts);
    *p_gy_raw = (int16_t)roundf(gy_counts);
    *p_gz_raw = (int16_t)roundf(gz_counts);
}
