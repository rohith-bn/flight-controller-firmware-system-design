/**
 * @file quad_physics.h
 * @brief 6-DOF Rigid-Body Quadrotor Flight Dynamics Simulation Engine
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 Compliance
 */

#ifndef QUAD_PHYSICS_H
#define QUAD_PHYSICS_H

#include "qfc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Quadrotor physical constants */
#define QUAD_MASS_KG                 (0.45f)      /**< Total airframe mass with battery */
#define QUAD_ARM_LENGTH_M            (0.125f)     /**< Center-to-motor arm distance (250mm diagonal) */
#define QUAD_INERTIA_IXX             (0.0040f)    /**< Roll axis moment of inertia (kg*m^2) */
#define QUAD_INERTIA_IYY             (0.0040f)    /**< Pitch axis moment of inertia (kg*m^2) */
#define QUAD_INERTIA_IZZ             (0.0075f)    /**< Yaw axis moment of inertia (kg*m^2) */
#define QUAD_MAX_MOTOR_THRUST_N      (2.40f)      /**< Max static thrust per motor (9.6N total = ~2.1:1 TWR) */
#define QUAD_AERO_DAMPING_COEFF      (0.0025f)    /**< Propeller wash aerodynamic damping (N*m / (rad/s)) */

/**
 * @brief 6-DOF Quadrotor Physical State Structure
 */
typedef struct {
    /* Attitude (Euler angles in degrees) */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    /* Body angular rates (degrees / second) */
    float p_dps;  /**< Roll rate */
    float q_dps;  /**< Pitch rate */
    float r_dps;  /**< Yaw rate */

    /* Linear acceleration in body frame (m/s^2) */
    vector3f_t accel_body_mps2;

    /* External disturbance torques (N*m) */
    float dist_torque_roll_nm;
    float dist_torque_pitch_nm;

    /* Simulation time */
    float time_s;
    float dt_s;
} quad_physics_t;

/**
 * @brief Initializes quadrotor physics state to level hover or custom attitude.
 * 
 * @param p_quad Pointer to physics state instance
 * @param init_roll_deg Initial roll angle in degrees
 * @param init_pitch_deg Initial pitch angle in degrees
 * @param dt_s Numerical integration time step (typically 0.001 s)
 * @return bool true on success, false on invalid pointer/parameters
 */
bool quad_physics_init(quad_physics_t *p_quad, float init_roll_deg, float init_pitch_deg, float dt_s);

/**
 * @brief Applies external disturbance torque (wind gust, ground strike, push).
 * 
 * @param p_quad Pointer to physics state instance
 * @param roll_torque_nm External torque around roll axis in N*m
 * @param pitch_torque_nm External torque around pitch axis in N*m
 */
void quad_physics_apply_disturbance(quad_physics_t *p_quad, float roll_torque_nm, float pitch_torque_nm);

/**
 * @brief Integrates rigid-body flight dynamics one time step forward based on motor commands.
 * 
 * @param p_quad Pointer to physics state instance
 * @param p_motors Commanded motor outputs (normalized 0.0 to 1.0)
 * @return bool true on success, false on NULL pointers
 */
bool quad_physics_step(quad_physics_t *p_quad, const motor_commands_t *p_motors);

/**
 * @brief Synthesizes 16-bit big-endian ADC raw sensor counts from true body physics.
 * 
 * Maps acceleration and angular rates into simulated MPU6050 register values:
 * Accel: 16384 LSB/g, Gyro: 131 LSB/(deg/s).
 * 
 * @param p_quad Pointer to physics state instance
 * @param p_ax_raw Output raw Accel X count
 * @param p_ay_raw Output raw Accel Y count
 * @param p_az_raw Output raw Accel Z count
 * @param p_gx_raw Output raw Gyro X count
 * @param p_gy_raw Output raw Gyro Y count
 * @param p_gz_raw Output raw Gyro Z count
 */
void quad_physics_get_raw_imu_counts(const quad_physics_t *p_quad,
                                     int16_t *p_ax_raw, int16_t *p_ay_raw, int16_t *p_az_raw,
                                     int16_t *p_gx_raw, int16_t *p_gy_raw, int16_t *p_gz_raw);

#ifdef __cplusplus
}
#endif

#endif /* QUAD_PHYSICS_H */
