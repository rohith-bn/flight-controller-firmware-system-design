/**
 * @file pwm_driver.h
 * @brief Virtual API Hardware Interface for Motor PWM Actuator Control
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category C hardware boundary seam. Maps normalized motor commands [0.0, 1.0]
 * to hardware timer compare registers across 4 independent ESC channels.
 * Provides instant zeroing on emergency fault conditions.
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "qfc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the hardware timer compare PWM peripheral channels.
 * 
 * Sets all 4 channels to 0.0 duty cycle immediately upon initialization.
 * 
 * @return bool true on success, false on hardware timer fault
 */
bool pwm_driver_init(void);

/**
 * @brief Updates timer compare registers with new normalized motor thrust commands.
 * 
 * Clamps all inputs within [0.0, 1.0] before writing to hardware registers.
 * 
 * @param[in] p_thrust Pointer to normalized thrust commands structure
 * @return bool true on success, false if p_thrust is NULL or driver uninitialized
 */
bool pwm_driver_set_thrust(const motor_commands_t *p_thrust);

/**
 * @brief Immediately forces all 4 timer compare channels to zero duty cycle.
 * 
 * Critical safety function invoked upon FaultDetected or Disarm events.
 */
void pwm_driver_zero_all(void);

/**
 * @brief Diagnostic accessor to inspect currently applied motor commands.
 * 
 * @param[out] p_out Destination structure for active motor commands
 * @return bool true on success, false if p_out is NULL
 */
bool pwm_driver_get_active_thrust(motor_commands_t *p_out);

#ifdef __cplusplus
}
#endif

#endif /* PWM_DRIVER_H */
