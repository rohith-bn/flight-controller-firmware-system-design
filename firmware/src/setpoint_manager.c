/**
 * @file setpoint_manager.c
 * @brief Target Orientation Setpoint Manager Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "setpoint_manager.h"
#include <stddef.h>

bool setpoint_manager_init(setpoint_manager_t *p_mgr) {
    if (p_mgr == NULL) {
        return false;
    }

    p_mgr->roll_target_deg  = 0.0f;
    p_mgr->pitch_target_deg = 0.0f;
    p_mgr->b_initialized    = true;

    return true;
}

bool setpoint_manager_set(setpoint_manager_t *p_mgr, float roll_deg, float pitch_deg) {
    if ((p_mgr == NULL) || (!p_mgr->b_initialized)) {
        return false;
    }

    bool b_saturated = false;
    float clamped_roll = roll_deg;
    float clamped_pitch = pitch_deg;

    /* Validate and clamp roll */
    if (clamped_roll > QFC_MAX_ALLOWABLE_TILT_DEG) {
        clamped_roll = QFC_MAX_ALLOWABLE_TILT_DEG;
        b_saturated = true;
    } else if (clamped_roll < -QFC_MAX_ALLOWABLE_TILT_DEG) {
        clamped_roll = -QFC_MAX_ALLOWABLE_TILT_DEG;
        b_saturated = true;
    }

    /* Validate and clamp pitch */
    if (clamped_pitch > QFC_MAX_ALLOWABLE_TILT_DEG) {
        clamped_pitch = QFC_MAX_ALLOWABLE_TILT_DEG;
        b_saturated = true;
    } else if (clamped_pitch < -QFC_MAX_ALLOWABLE_TILT_DEG) {
        clamped_pitch = -QFC_MAX_ALLOWABLE_TILT_DEG;
        b_saturated = true;
    }

    p_mgr->roll_target_deg  = clamped_roll;
    p_mgr->pitch_target_deg = clamped_pitch;

    return !b_saturated;
}

bool setpoint_manager_get(const setpoint_manager_t *p_mgr, setpoint_deg_t *p_setpoint) {
    if ((p_mgr == NULL) || (p_setpoint == NULL) || (!p_mgr->b_initialized)) {
        return false;
    }

    p_setpoint->roll_deg  = p_mgr->roll_target_deg;
    p_setpoint->pitch_deg = p_mgr->pitch_target_deg;

    return true;
}

bool setpoint_manager_reset(setpoint_manager_t *p_mgr) {
    if ((p_mgr == NULL) || (!p_mgr->b_initialized)) {
        return false;
    }

    p_mgr->roll_target_deg  = 0.0f;
    p_mgr->pitch_target_deg = 0.0f;

    return true;
}
