/**
 * @file pid_controller.c
 * @brief Closed-Loop Attitude Stabilization Coordinator Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "pid_controller.h"
#include <stddef.h>

bool pid_controller_init(pid_controller_t        *p_ctrl,
                         const pid_axis_config_t *p_roll_cfg,
                         const pid_axis_config_t *p_pitch_cfg) {
    if ((p_ctrl == NULL) || (p_roll_cfg == NULL) || (p_pitch_cfg == NULL)) {
        return false;
    }

    if (!pid_axis_init(&p_ctrl->roll_axis, p_roll_cfg)) {
        return false;
    }

    if (!pid_axis_init(&p_ctrl->pitch_axis, p_pitch_cfg)) {
        return false;
    }

    p_ctrl->b_initialized = true;
    return true;
}

bool pid_controller_reset(pid_controller_t *p_ctrl) {
    if ((p_ctrl == NULL) || (!p_ctrl->b_initialized)) {
        return false;
    }

    if (!pid_axis_reset(&p_ctrl->roll_axis)) {
        return false;
    }

    if (!pid_axis_reset(&p_ctrl->pitch_axis)) {
        return false;
    }

    return true;
}

bool pid_controller_update(pid_controller_t      *p_ctrl,
                           const setpoint_deg_t  *p_target,
                           const attitude_deg_t  *p_actual,
                           pid_demands_t         *p_demands) {
    if ((p_ctrl == NULL) || (p_target == NULL) || (p_actual == NULL) || (p_demands == NULL)) {
        return false;
    }

    if (!p_ctrl->b_initialized) {
        return false;
    }

    /* Delegate error computation and control law to independent axis Strategy instances */
    p_demands->roll_demand  = pid_axis_update(&p_ctrl->roll_axis,  p_target->roll_deg,  p_actual->roll_deg);
    p_demands->pitch_demand = pid_axis_update(&p_ctrl->pitch_axis, p_target->pitch_deg, p_actual->pitch_deg);

    return true;
}
