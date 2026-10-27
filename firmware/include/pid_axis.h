/**
 * @file pid_axis.h
 * @brief Discrete PID Axis Controller with Anti-Windup Protection
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category A module. Implements a single-axis discrete Proportional-Integral-Derivative
 * feedback control law with integral anti-windup clamping and derivative smoothing.
 * Re-used as concrete Strategy pattern instances for Roll and Pitch axes.
 */

#ifndef PID_AXIS_H
#define PID_AXIS_H

#include "qfc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration tuning gains and limits for a single PID axis.
 */
typedef struct {
    float kp;           /**< Proportional gain */
    float ki;           /**< Integral gain */
    float kd;           /**< Derivative gain */
    float windup_limit; /**< Symmetric limit on integral accumulation [-limit, +limit] */
    float dt_s;         /**< Discrete control sample period in seconds (typical: 0.001 s) */
} pid_axis_config_t;

/**
 * @brief Internal state variables for a single PID axis.
 */
typedef struct {
    float integral_accum;   /**< Persistent discrete integral accumulator */
    float prev_error;       /**< Error from previous sample step for diagnostics */
    float prev_measurement; /**< Measurement from previous sample step for derivative-on-measurement */
    float last_p_term;      /**< Cached proportional contribution */
    float last_i_term;      /**< Cached integral contribution */
    float last_d_term;      /**< Cached derivative contribution */
    bool  b_first_run;      /**< Flag to avoid derivative spike on first update call */
} pid_axis_state_t;

/**
 * @brief Complete PID axis instance.
 */
typedef struct {
    pid_axis_config_t config;
    pid_axis_state_t  state;
    bool              b_initialized;
} pid_axis_t;

/**
 * @brief Initializes a PID axis instance with tuning parameters.
 * 
 * @param[out] p_pid    Pointer to PID instance
 * @param[in]  p_config Pointer to configuration gains and limits
 * @return bool true if successfully configured, false on NULL or invalid gains
 */
bool pid_axis_init(pid_axis_t *p_pid, const pid_axis_config_t *p_config);

/**
 * @brief Resets the internal state (integrator accumulator and previous error) to zero.
 * 
 * @param[in,out] p_pid Pointer to PID instance
 * @return bool true if reset successfully, false if p_pid is NULL
 */
bool pid_axis_reset(pid_axis_t *p_pid);

/**
 * @brief Computes one discrete PID update step.
 * 
 * e(t) = setpoint - measurement
 * P = Kp * e(t)
 * I = clamp(I + Ki * e(t) * dt, -windup_limit, +windup_limit)
 * D = Kd * (e(t) - e(t-1)) / dt  (D = 0 on first run)
 * Output = P + I + D
 * 
 * @param[in,out] p_pid       Pointer to PID instance
 * @param[in]     setpoint    Target attitude angle or rate
 * @param[in]     measurement Current feedback attitude angle or rate
 * @return float Computed control demand
 */
float pid_axis_update(pid_axis_t *p_pid, float setpoint, float measurement);

/**
 * @brief Diagnostic accessor to inspect individual P, I, and D term contributions.
 * 
 * @param[in]  p_pid    Pointer to PID instance
 * @param[out] p_p_term Destination for P contribution (may be NULL)
 * @param[out] p_i_term Destination for I contribution (may be NULL)
 * @param[out] p_d_term Destination for D contribution (may be NULL)
 * @return bool true if terms retrieved, false if p_pid is NULL or uninitialized
 */
bool pid_axis_get_terms(const pid_axis_t *p_pid, float *p_p_term, float *p_i_term, float *p_d_term);

#ifdef __cplusplus
}
#endif

#endif /* PID_AXIS_H */
