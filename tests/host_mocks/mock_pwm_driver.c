/**
 * @file mock_pwm_driver.c
 * @brief Host Mock & Test Double for PWM Timer Hardware Seam Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "mock_pwm_driver.h"
#include <string.h>

static bool             s_mock_pwm_healthy = true;
static motor_commands_t s_active_thrust = {0.0f, 0.0f, 0.0f, 0.0f};
static uint32_t         s_zero_call_count = 0;
static uint32_t         s_update_call_count = 0;

void mock_pwm_driver_reset(void) {
    s_mock_pwm_healthy = true;
    s_active_thrust.m1 = 0.0f;
    s_active_thrust.m2 = 0.0f;
    s_active_thrust.m3 = 0.0f;
    s_active_thrust.m4 = 0.0f;
    s_zero_call_count = 0;
    s_update_call_count = 0;
}

void mock_pwm_driver_set_hardware_healthy(bool healthy) {
    s_mock_pwm_healthy = healthy;
}

uint32_t mock_pwm_driver_get_zero_count(void) {
    return s_zero_call_count;
}

uint32_t mock_pwm_driver_get_update_count(void) {
    return s_update_call_count;
}

/* ========================================================================== */
/* HARDWARE INTERFACE IMPLEMENTATION (LINKED BY FIRMWARE)                     */
/* ========================================================================== */

bool pwm_driver_init(void) {
    if (!s_mock_pwm_healthy) {
        return false;
    }
    pwm_driver_zero_all();
    return true;
}

bool pwm_driver_set_thrust(const motor_commands_t *p_thrust) {
    if (!s_mock_pwm_healthy || (p_thrust == NULL)) {
        return false;
    }

    s_active_thrust = *p_thrust;
    s_update_call_count++;
    return true;
}

void pwm_driver_zero_all(void) {
    s_active_thrust.m1 = 0.0f;
    s_active_thrust.m2 = 0.0f;
    s_active_thrust.m3 = 0.0f;
    s_active_thrust.m4 = 0.0f;
    s_zero_call_count++;
}

bool pwm_driver_get_active_thrust(motor_commands_t *p_out) {
    if (p_out == NULL) {
        return false;
    }
    *p_out = s_active_thrust;
    return true;
}
