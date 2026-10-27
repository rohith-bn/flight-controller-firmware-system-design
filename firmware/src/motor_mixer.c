/**
 * @file motor_mixer.c
 * @brief Quad-X Geometry Motor Thrust Mixer and Safety Output Clamper Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "motor_mixer.h"
#include <stddef.h>

static float clamp_thrust(float val) {
    if (val < QFC_MOTOR_MIN_COMMAND) {
        return QFC_MOTOR_MIN_COMMAND;
    }
    if (val > QFC_MOTOR_MAX_COMMAND) {
        return QFC_MOTOR_MAX_COMMAND;
    }
    return val;
}

bool motor_mixer_compute(float throttle, float roll_demand, float pitch_demand, motor_commands_t *p_motors) {
    if (p_motors == NULL) {
        return false;
    }

    /* 1. Quad-X mixing equations */
    float m1_raw = throttle + roll_demand - pitch_demand;
    float m2_raw = throttle - roll_demand - pitch_demand;
    float m3_raw = throttle + roll_demand + pitch_demand;
    float m4_raw = throttle - roll_demand + pitch_demand;

    /* 2. Extreme runaway detection check */
    if ((m1_raw < MOTOR_MIXER_EXTREME_LOWER_BOUND) || (m1_raw > MOTOR_MIXER_EXTREME_UPPER_BOUND) ||
        (m2_raw < MOTOR_MIXER_EXTREME_LOWER_BOUND) || (m2_raw > MOTOR_MIXER_EXTREME_UPPER_BOUND) ||
        (m3_raw < MOTOR_MIXER_EXTREME_LOWER_BOUND) || (m3_raw > MOTOR_MIXER_EXTREME_UPPER_BOUND) ||
        (m4_raw < MOTOR_MIXER_EXTREME_LOWER_BOUND) || (m4_raw > MOTOR_MIXER_EXTREME_UPPER_BOUND)) {
        motor_mixer_zero(p_motors);
        return false;
    }

    /* 3. Safe saturation clamping [0.0, 1.0] */
    p_motors->m1 = clamp_thrust(m1_raw);
    p_motors->m2 = clamp_thrust(m2_raw);
    p_motors->m3 = clamp_thrust(m3_raw);
    p_motors->m4 = clamp_thrust(m4_raw);

    return true;
}

void motor_mixer_zero(motor_commands_t *p_motors) {
    if (p_motors != NULL) {
        p_motors->m1 = 0.0f;
        p_motors->m2 = 0.0f;
        p_motors->m3 = 0.0f;
        p_motors->m4 = 0.0f;
    }
}
