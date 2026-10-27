/**
 * @file flight_controller.h
 * @brief Top-Level 1 ms Deterministic Super Loop Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category B module. Top-level firmware orchestrator for QFC-1. Owns the startup
 * sequence and executes the deterministic 1 ms control pipeline:
 * 
 *   [IMU Read] -> [Attitude Est] -> [PID Compute] -> [Motor Mixer] -> [Safety Gate] -> [PWM Write]
 * 
 * Monitors all subsystem tripwires (bus timeout, attitude divergence, mixer overflow)
 * and guarantees immediate motor zeroing upon fault assert.
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "qfc_types.h"
#include "system_state_manager.h"
#include "imu_driver.h"
#include "attitude_estimator.h"
#include "setpoint_manager.h"
#include "pid_controller.h"
#include "motor_mixer.h"
#include "pwm_driver.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Top-level flight controller system context.
 */
typedef struct {
    system_state_manager_t state_mgr;     /**< Safety state machine and motor gatekeeper */
    imu_driver_t           imu_drv;       /**< Inertial sensor coordinator & calibrator */
    attitude_estimator_t   attitude_est;  /**< Sensor fusion orientation estimator */
    setpoint_manager_t     setpoint_mgr;  /**< Target orientation setpoint owner */
    pid_controller_t       pid_ctrl;      /**< Dual-axis PID stabilization coordinator */
    motor_commands_t       last_commands; /**< Most recent commanded motor outputs */
    attitude_deg_t         current_att;   /**< Latest estimated attitude */
    float                  base_throttle; /**< Baseline pilot throttle [0.0, 1.0] */
    bool                   b_initialized; /**< Startup initialization flag */
} flight_controller_t;

/**
 * @brief Executes the complete system startup and self-test sequence.
 * 
 * @param[out] p_fc        Pointer to flight controller context
 * @param[in]  p_roll_cfg  Tuning configuration for Roll PID axis
 * @param[in]  p_pitch_cfg Tuning configuration for Pitch PID axis
 * @return bool true if all subsystems initialized cleanly, false on hardware fault
 */
bool flight_controller_init(flight_controller_t     *p_fc,
                            const pid_axis_config_t *p_roll_cfg,
                            const pid_axis_config_t *p_pitch_cfg);

/**
 * @brief Executes one sample step of the startup gyro bias calibration sequence.
 * 
 * @param[in,out] p_fc         Pointer to flight controller context
 * @param[out]    p_b_complete Destination set to true when all 512 samples are complete
 * @return bool true on successful step, false on I2C fault or excessive bench motion
 */
bool flight_controller_calibrate_step(flight_controller_t *p_fc, bool *p_b_complete);

/**
 * @brief Requests the flight controller to transition into ARMED state.
 * 
 * Rejects arming if startup calibration has not finished or if in FAULT state.
 * 
 * @param[in,out] p_fc Pointer to flight controller context
 * @return bool true if successfully armed, false otherwise
 */
bool flight_controller_arm(flight_controller_t *p_fc);

/**
 * @brief Requests the flight controller to transition into DISARMED state.
 * 
 * @param[in,out] p_fc Pointer to flight controller context
 * @return bool true if disarmed, false if in FAULT state
 */
bool flight_controller_disarm(flight_controller_t *p_fc);

/**
 * @brief Sets the baseline hovering/climb throttle [0.0, 1.0].
 * 
 * @param[in,out] p_fc     Pointer to flight controller context
 * @param[in]     throttle Base throttle command [0.0, 1.0]
 * @return bool true if throttle set, false if out of range or p_fc is NULL
 */
bool flight_controller_set_throttle(flight_controller_t *p_fc, float throttle);

/**
 * @brief Executes the deterministic 1 ms control super loop pipeline cycle.
 * 
 * Pipeline order:
 * 1. Read calibrated IMU measurements from imu_driver.
 * 2. Update attitude estimation via attitude_estimator.
 * 3. Query target orientation from setpoint_manager.
 * 4. Compute stabilization demands via pid_controller.
 * 5. Mix demands across Quad-X geometry via motor_mixer.
 * 6. Gate motor commands through system_state_manager.
 * 7. Write PWM duty cycles via pwm_driver.
 * 
 * @param[in,out] p_fc Pointer to flight controller context
 * @return bool true on cycle completion, false if an emergency fault occurred
 */
bool flight_controller_process_tick(flight_controller_t *p_fc);

/**
 * @brief Asserts an emergency system fault, immediately latching FAULT state.
 * 
 * @param[in,out] p_fc Pointer to flight controller context
 * @param[in]     code Fault code to assert
 */
void flight_controller_assert_fault(flight_controller_t *p_fc, fault_code_t code);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROLLER_H */
