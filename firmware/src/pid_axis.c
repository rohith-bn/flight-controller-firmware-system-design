/**
 * @file pid_axis.c
 * @brief Discrete PID Axis Controller with Anti-Windup Protection Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "pid_axis.h"
#include <stddef.h>

bool pid_axis_init(pid_axis_t *p_pid, const pid_axis_config_t *p_config) {
    if ((p_pid == NULL) || (p_config == NULL)) {
        return false;
    }

    if ((p_config->kp < 0.0f) || (p_config->ki < 0.0f) || (p_config->kd < 0.0f) ||
        (p_config->windup_limit < 0.0f) || (p_config->dt_s <= 0.0f)) {
        return false;
    }

    p_pid->config = *p_config;

    p_pid->state.integral_accum   = 0.0f;
    p_pid->state.prev_error       = 0.0f;
    p_pid->state.prev_measurement = 0.0f;
    p_pid->state.last_p_term      = 0.0f;
    p_pid->state.last_i_term      = 0.0f;
    p_pid->state.last_d_term      = 0.0f;
    p_pid->state.b_first_run      = true;

    p_pid->b_initialized          = true;

    return true;
}

bool pid_axis_reset(pid_axis_t *p_pid) {
    if ((p_pid == NULL) || (!p_pid->b_initialized)) {
        return false;
    }

    p_pid->state.integral_accum   = 0.0f;
    p_pid->state.prev_error       = 0.0f;
    p_pid->state.prev_measurement = 0.0f;
    p_pid->state.last_p_term      = 0.0f;
    p_pid->state.last_i_term      = 0.0f;
    p_pid->state.last_d_term      = 0.0f;
    p_pid->state.b_first_run      = true;

    return true;
}

float pid_axis_update(pid_axis_t *p_pid, float setpoint, float measurement) {
    if ((p_pid == NULL) || (!p_pid->b_initialized)) {
        return 0.0f;
    }

    /* 1. Error computation */
    float error = setpoint - measurement;

    /* 2. Proportional term */
    float p_term = p_pid->config.kp * error;

    /* 3. Integral term with symmetric anti-windup clamping */
    p_pid->state.integral_accum += (p_pid->config.ki * error * p_pid->config.dt_s);

    if (p_pid->state.integral_accum > p_pid->config.windup_limit) {
        p_pid->state.integral_accum = p_pid->config.windup_limit;
    } else if (p_pid->state.integral_accum < -p_pid->config.windup_limit) {
        p_pid->state.integral_accum = -p_pid->config.windup_limit;
    }
    float i_term = p_pid->state.integral_accum;

    /* 4. Derivative term on measurement (eliminates setpoint step derivative kick) */
    float d_term = 0.0f;
    if (p_pid->state.b_first_run) {
        d_term = 0.0f;
        p_pid->state.b_first_run = false;
    } else {
        float delta_meas = measurement - p_pid->state.prev_measurement;
        d_term = -p_pid->config.kd * (delta_meas / p_pid->config.dt_s);
    }
    p_pid->state.prev_measurement = measurement;
    p_pid->state.prev_error       = error;

    /* 5. Cache terms for diagnostics */
    p_pid->state.last_p_term = p_term;
    p_pid->state.last_i_term = i_term;
    p_pid->state.last_d_term = d_term;

    /* 6. Total control demand */
    return p_term + i_term + d_term;
}

bool pid_axis_get_terms(const pid_axis_t *p_pid, float *p_p_term, float *p_i_term, float *p_d_term) {
    if ((p_pid == NULL) || (!p_pid->b_initialized)) {
        return false;
    }

    if (p_p_term != NULL) {
        *p_p_term = p_pid->state.last_p_term;
    }
    if (p_i_term != NULL) {
        *p_i_term = p_pid->state.last_i_term;
    }
    if (p_d_term != NULL) {
        *p_d_term = p_pid->state.last_d_term;
    }

    return true;
}
