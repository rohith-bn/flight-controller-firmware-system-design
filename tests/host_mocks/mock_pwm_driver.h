/**
 * @file mock_pwm_driver.h
 * @brief Host Mock & Test Double for PWM Timer Hardware Seam
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#ifndef MOCK_PWM_DRIVER_H
#define MOCK_PWM_DRIVER_H

#include "pwm_driver.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void mock_pwm_driver_reset(void);
void mock_pwm_driver_set_hardware_healthy(bool healthy);
uint32_t mock_pwm_driver_get_zero_count(void);
uint32_t mock_pwm_driver_get_update_count(void);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_PWM_DRIVER_H */
