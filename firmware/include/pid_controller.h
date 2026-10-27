/**
 * @file pid_controller.h
 * @brief Closed-Loop Attitude Stabilization Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category B module. Coordinates dual-axis attitude stabilization by composing
 * two independent Strategy pattern instances of pid_axis (roll_pid and pitch_pid).
 * Receives target setpoints and estimated actual attitude, computes error, and
 * delivers dynamic stabilization demands to the motor mixer.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "qfc_types.h"
#include "pid_axis.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Closed-loop attitude controller state structure.
 */
typedef struct {
    pid_axis_t roll_axis;    /**< Strategy instance for Roll axis */
    pid_axis_t pitch_axis;   /**< Strategy instance for Pitch axis */
    bool       b_initialized;/**< Initialization flag */
} pid_controller_t;

/**
 * @brief Initializes the dual-axis PID controller subsystem.
 * 
 * @param[out] p_ctrl      Pointer to controller instance
 * @param[in]  p_roll_cfg  Tuning gains and anti-windup configuration for Roll axis
 * @param[in]  p_pitch_cfg Tuning gains and anti-windup configuration for Pitch axis
 * @return bool true on success, false if any pointer is NULL or configurations invalid
 */
bool pid_controller_init(pid_controller_t        *p_ctrl,
                         const pid_axis_config_t *p_roll_cfg,
                         const pid_axis_config_t *p_pitch_cfg);

/**
 * @brief Resets internal states (integrators and derivative history) of both axes to zero.
 * 
 * @param[in,out] p_ctrl Pointer to controller instance
 * @return bool true on success, false if p_ctrl is NULL or uninitialized
 */
bool pid_controller_reset(pid_controller_t *p_ctrl);

/**
 * @brief Computes 1 ms dynamic stabilization demands for Roll and Pitch axes.
 * 
 * @param[in,out] p_ctrl    Pointer to controller instance
 * @param[in]     p_target  Pointer to target attitude setpoints
 * @param[in]     p_actual  Pointer to estimated current attitude
 * @param[out]    p_demands Destination structure for computed stabilization demands
 * @return bool true on success, false if any pointer is NULL or controller uninitialized
 */
bool pid_controller_update(pid_controller_t      *p_ctrl,
                           const setpoint_deg_t  *p_target,
                           const attitude_deg_t  *p_actual,
                           pid_demands_t         *p_demands);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
