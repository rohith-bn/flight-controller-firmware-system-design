/**
 * @file sil_harness.h
 * @brief Software-in-the-Loop Simulation Harness Interfacing Firmware and Physics
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 Compliance
 */

#ifndef SIL_HARNESS_H
#define SIL_HARNESS_H

#include "quad_physics.h"
#include "flight_controller.h"
#include "mock_imu_comm.h"
#include "mock_pwm_driver.h"
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Default PID gains tuned for quadrotor dynamics */
#define SIL_DEFAULT_KP             (0.0030f)   /**< Proportional gain (deg error to motor thrust command) */
#define SIL_DEFAULT_KI             (0.0002f)   /**< Integral gain for steady-state trim */
#define SIL_DEFAULT_KD             (0.0005f)   /**< Derivative gain on measurement for rate damping */
#define SIL_DEFAULT_WINDUP_LIMIT   (0.15f)     /**< Symmetric integrator clamp */

/**
 * @brief Telemetry snapshot recorded at each 1 ms simulation step.
 */
typedef struct {
    float time_s;
    float true_roll_deg;
    float true_pitch_deg;
    float est_roll_deg;
    float est_pitch_deg;
    float target_roll_deg;
    float target_pitch_deg;
    float m1_thrust;
    float m2_thrust;
    float m3_thrust;
    float m4_thrust;
    system_state_t sys_state;
    fault_code_t fault;
} sil_telemetry_t;

/**
 * @brief SIL closed-loop simulation container.
 */
typedef struct {
    quad_physics_t physics;
    flight_controller_t fc;
    sil_telemetry_t last_telemetry;
    float hover_throttle;
    bool b_armed;
} sil_harness_t;

/**
 * @brief Initializes closed-loop simulation, completes 512-sample boot calibration, and arms.
 * 
 * @param p_harness Pointer to SIL harness
 * @param init_roll_deg Initial aircraft roll angle in degrees
 * @param init_pitch_deg Initial aircraft pitch angle in degrees
 * @param hover_throttle Base hover throttle command (0.0 to 1.0, typically 0.45)
 * @param p_pid_cfg PID gain parameters (or NULL for SIL_DEFAULT gains)
 * @return bool true on success, false on failure
 */
bool sil_harness_init(sil_harness_t           *p_harness,
                      float                    init_roll_deg,
                      float                    init_pitch_deg,
                      float                    hover_throttle,
                      const pid_axis_config_t *p_pid_cfg);

/**
 * @brief Executes 1 closed-loop simulation step (1 ms = 1000 Hz).
 * 
 * @param p_harness Pointer to SIL harness
 * @return bool true if super loop succeeded, false if fault tripped
 */
bool sil_harness_step(sil_harness_t *p_harness);

/**
 * @brief Retrieves the latest telemetry snapshot.
 * 
 * @param p_harness Pointer to SIL harness
 * @param p_telem Output telemetry structure
 */
void sil_harness_get_telemetry(const sil_harness_t *p_harness, sil_telemetry_t *p_telem);

/**
 * @brief Sets a new target attitude setpoint in the flight controller.
 * 
 * @param p_harness Pointer to SIL harness
 * @param roll_deg Target roll angle in degrees (+/- 45 deg max)
 * @param pitch_deg Target pitch angle in degrees (+/- 45 deg max)
 */
void sil_harness_set_setpoint(sil_harness_t *p_harness, float roll_deg, float pitch_deg);

/**
 * @brief Injects external disturbance torque into physics engine.
 * 
 * @param p_harness Pointer to SIL harness
 * @param roll_torque_nm External roll torque in N*m
 * @param pitch_torque_nm External pitch torque in N*m
 */
void sil_harness_apply_disturbance(sil_harness_t *p_harness, float roll_torque_nm, float pitch_torque_nm);

#ifdef __cplusplus
}
#endif

#endif /* SIL_HARNESS_H */
