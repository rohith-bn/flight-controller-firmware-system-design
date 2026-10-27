/**
 * @file motor_mixer.h
 * @brief Quad-X Geometry Motor Thrust Mixer and Safety Output Clamper
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category A module. Combines base throttle with PID roll and pitch stabilization
 * demands across standard Quad-X quadrotor geometry:
 * 
 *   M4 (FL, CW)     M1 (FR, CCW)
 *          \       /
 *           [ QFC ]
 *          /       \
 *   M3 (RL, CCW)    M2 (RR, CW)
 * 
 * M1 = Throttle + Roll - Pitch
 * M2 = Throttle - Roll - Pitch
 * M3 = Throttle + Roll + Pitch
 * M4 = Throttle - Roll + Pitch
 */

#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "qfc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Extreme bounds threshold beyond which mixing indicates controller runaway */
#define MOTOR_MIXER_EXTREME_LOWER_BOUND  (-0.25f)
#define MOTOR_MIXER_EXTREME_UPPER_BOUND  (1.25f)

/**
 * @brief Computes normalized individual motor thrust commands for Quad-X frame.
 * 
 * Clamps all final outputs within safe range [0.0, 1.0].
 * If any motor command before clamping breaches extreme thresholds (< -0.25 or > 1.25),
 * returns false indicating an out-of-bounds runaway fault.
 * 
 * @param[in]  throttle     Base throttle command [0.0, 1.0]
 * @param[in]  roll_demand  PID roll axis differential demand
 * @param[in]  pitch_demand PID pitch axis differential demand
 * @param[out] p_motors     Pointer to motor output structure
 * @return bool true if mixing succeeded within tolerable bounds, false on NULL or runaway fault
 */
bool motor_mixer_compute(float throttle, float roll_demand, float pitch_demand, motor_commands_t *p_motors);

/**
 * @brief Immediately forces all motor outputs strictly to 0.0 (idle/disarmed state).
 * 
 * @param[out] p_motors Pointer to motor output structure
 */
void motor_mixer_zero(motor_commands_t *p_motors);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MIXER_H */
