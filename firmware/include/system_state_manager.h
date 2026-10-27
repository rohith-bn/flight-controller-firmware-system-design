/**
 * @file system_state_manager.h
 * @brief Safety Lifecycle Gatekeeper and Fault Observer Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category B module. Manages the system safety state machine (DISARMED, ARMED, FAULT),
 * enforces irreversible lock upon emergency fault assert, gates motor thrust commands,
 * and notifies registered fault observer callbacks.
 */

#ifndef SYSTEM_STATE_MANAGER_H
#define SYSTEM_STATE_MANAGER_H

#include "qfc_types.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of registered fault observer callbacks */
#define SYSTEM_STATE_MAX_FAULT_LISTENERS (4U)

/**
 * @brief Function pointer signature for emergency fault observer callbacks.
 */
typedef void (*fault_callback_t)(fault_code_t code);

/**
 * @brief State structure for system state manager.
 */
typedef struct {
    system_state_t   current_state;                                  /**< Active system lifecycle state */
    fault_code_t     active_fault;                                   /**< Latched fault code */
    fault_callback_t fault_listeners[SYSTEM_STATE_MAX_FAULT_LISTENERS]; /**< Observer callbacks */
    uint8_t          listener_count;                                 /**< Registered observer count */
    bool             b_initialized;                                  /**< Initialization flag */
} system_state_manager_t;

/**
 * @brief Initializes the system state manager in default DISARMED state.
 * 
 * @param[out] p_mgr Pointer to state manager instance
 * @return bool true on success, false if p_mgr is NULL
 */
bool system_state_manager_init(system_state_manager_t *p_mgr);

/**
 * @brief Requests transition from DISARMED to ARMED flight mode.
 * 
 * Rejected if the system is currently in FAULT state.
 * 
 * @param[in,out] p_mgr Pointer to state manager instance
 * @return bool true if state transitioned to ARMED, false if rejected
 */
bool system_state_manager_arm(system_state_manager_t *p_mgr);

/**
 * @brief Requests transition from ARMED to DISARMED standby mode.
 * 
 * Rejected if the system is currently in FAULT state.
 * 
 * @param[in,out] p_mgr Pointer to state manager instance
 * @return bool true if state transitioned to DISARMED, false if rejected
 */
bool system_state_manager_disarm(system_state_manager_t *p_mgr);

/**
 * @brief Asserts an emergency system fault, immediately latching FAULT state.
 * 
 * Locks out all subsequent arm requests. Immediately notifies all registered
 * fault observer callbacks with the asserted fault code.
 * 
 * @param[in,out] p_mgr Pointer to state manager instance
 * @param[in]     code  Categorized fault code to assert
 * @return bool true on success, false if p_mgr is NULL
 */
bool system_state_manager_assert_fault(system_state_manager_t *p_mgr, fault_code_t code);

/**
 * @brief Registers an observer callback to be notified when a fault is asserted.
 * 
 * @param[in,out] p_mgr Pointer to state manager instance
 * @param[in]     cb    Callback function pointer
 * @return bool true if registered, false if table full or cb is NULL
 */
bool system_state_manager_register_fault_callback(system_state_manager_t *p_mgr, fault_callback_t cb);

/**
 * @brief Safety motor command gatekeeper.
 * 
 * Passes through mixed motor commands only when in ARMED state.
 * Forces all outputs to 0.0f when DISARMED or in FAULT.
 * 
 * @param[in]  p_mgr Pointer to state manager instance
 * @param[in]  p_in  Pointer to raw mixed motor demands
 * @param[out] p_out Destination pointer receiving gated commands
 */
void system_state_manager_gate_motors(const system_state_manager_t *p_mgr,
                                      const motor_commands_t       *p_in,
                                      motor_commands_t             *p_out);

/**
 * @brief Retrieves the active system state.
 * 
 * @param[in] p_mgr Pointer to state manager instance
 * @return system_state_t Current state (returns SYSTEM_STATE_FAULT if p_mgr is NULL)
 */
system_state_t system_state_manager_get_state(const system_state_manager_t *p_mgr);

/**
 * @brief Retrieves the active latched fault code.
 * 
 * @param[in] p_mgr Pointer to state manager instance
 * @return fault_code_t Latched fault code
 */
fault_code_t system_state_manager_get_fault(const system_state_manager_t *p_mgr);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_STATE_MANAGER_H */
