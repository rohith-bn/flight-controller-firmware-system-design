/**
 * @file system_state_manager.c
 * @brief Safety Lifecycle Gatekeeper and Fault Observer Coordinator Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "system_state_manager.h"
#include <stddef.h>

bool system_state_manager_init(system_state_manager_t *p_mgr) {
    if (p_mgr == NULL) {
        return false;
    }

    p_mgr->current_state  = SYSTEM_STATE_DISARMED;
    p_mgr->active_fault   = FAULT_NONE;
    p_mgr->listener_count = 0U;

    for (uint8_t i = 0U; i < SYSTEM_STATE_MAX_FAULT_LISTENERS; i++) {
        p_mgr->fault_listeners[i] = NULL;
    }

    p_mgr->b_initialized  = true;
    return true;
}

bool system_state_manager_arm(system_state_manager_t *p_mgr) {
    if ((p_mgr == NULL) || (!p_mgr->b_initialized)) {
        return false;
    }

    /* One-way lock: reject all arm commands while in FAULT */
    if (p_mgr->current_state == SYSTEM_STATE_FAULT) {
        return false;
    }

    p_mgr->current_state = SYSTEM_STATE_ARMED;
    return true;
}

bool system_state_manager_disarm(system_state_manager_t *p_mgr) {
    if ((p_mgr == NULL) || (!p_mgr->b_initialized)) {
        return false;
    }

    /* Reject transition while locked in FAULT */
    if (p_mgr->current_state == SYSTEM_STATE_FAULT) {
        return false;
    }

    p_mgr->current_state = SYSTEM_STATE_DISARMED;
    return true;
}

bool system_state_manager_assert_fault(system_state_manager_t *p_mgr, fault_code_t code) {
    if ((p_mgr == NULL) || (!p_mgr->b_initialized)) {
        return false;
    }

    p_mgr->current_state = SYSTEM_STATE_FAULT;
    p_mgr->active_fault  = code;

    /* Observer Pattern: notify all registered callbacks */
    for (uint8_t i = 0U; i < p_mgr->listener_count; i++) {
        if (p_mgr->fault_listeners[i] != NULL) {
            p_mgr->fault_listeners[i](code);
        }
    }

    return true;
}

bool system_state_manager_register_fault_callback(system_state_manager_t *p_mgr, fault_callback_t cb) {
    if ((p_mgr == NULL) || (cb == NULL) || (!p_mgr->b_initialized)) {
        return false;
    }

    if (p_mgr->listener_count >= SYSTEM_STATE_MAX_FAULT_LISTENERS) {
        return false;
    }

    p_mgr->fault_listeners[p_mgr->listener_count++] = cb;
    return true;
}

void system_state_manager_gate_motors(const system_state_manager_t *p_mgr,
                                      const motor_commands_t       *p_in,
                                      motor_commands_t             *p_out) {
    if (p_out == NULL) {
        return;
    }

    if ((p_mgr != NULL) && (p_in != NULL) && (p_mgr->b_initialized) &&
        (p_mgr->current_state == SYSTEM_STATE_ARMED)) {
        *p_out = *p_in;
    } else {
        /* Force 0.0 on DISARMED, FAULT, or uninitialized/NULL */
        p_out->m1 = 0.0f;
        p_out->m2 = 0.0f;
        p_out->m3 = 0.0f;
        p_out->m4 = 0.0f;
    }
}

system_state_t system_state_manager_get_state(const system_state_manager_t *p_mgr) {
    if ((p_mgr == NULL) || (!p_mgr->b_initialized)) {
        return SYSTEM_STATE_FAULT;
    }
    return p_mgr->current_state;
}

fault_code_t system_state_manager_get_fault(const system_state_manager_t *p_mgr) {
    if ((p_mgr == NULL) || (!p_mgr->b_initialized)) {
        return FAULT_SUPERLOOP_OVERRUN;
    }
    return p_mgr->active_fault;
}
