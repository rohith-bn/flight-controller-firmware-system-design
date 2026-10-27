/**
 * @file sil_scenarios.h
 * @brief Automated Flight Simulation Verification Scenarios
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 Compliance
 */

#ifndef SIL_SCENARIOS_H
#define SIL_SCENARIOS_H

#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Scenario 1: Initial Severe Tilt Recovery to Level Hover.
 * 
 * Drops quadrotor at Roll = +20.0 deg, Pitch = -15.0 deg.
 * Verifies closed-loop stabilization converges to < 0.5 deg within 800 ms.
 * 
 * @param p_csv_file Optional open FILE* to write telemetry, or NULL
 * @return bool true if scenario passes all acceptance criteria, false otherwise
 */
bool sil_scenario_tilt_recovery(FILE *p_csv_file);

/**
 * @brief Scenario 2: Wind Gust / Torque Impulse Disturbance Rejection.
 * 
 * During level hover, applies a 0.04 N*m roll torque impulse for 80 ms.
 * Verifies attitude recovers to level flight (< 0.5 deg) within 500 ms after gust.
 * 
 * @param p_csv_file Optional open FILE* to write telemetry, or NULL
 * @return bool true if scenario passes all acceptance criteria, false otherwise
 */
bool sil_scenario_gust_rejection(FILE *p_csv_file);

/**
 * @brief Scenario 3: Target Attitude Dynamic Setpoint Tracking.
 * 
 * Commands step changes: Level -> +12 deg -> -12 deg -> Level.
 * Verifies rise time, steady-state accuracy (< 0.5 deg), and overshoot damping.
 * 
 * @param p_csv_file Optional open FILE* to write telemetry, or NULL
 * @return bool true if scenario passes all acceptance criteria, false otherwise
 */
bool sil_scenario_setpoint_tracking(FILE *p_csv_file);

/**
 * @brief Scenario 4: Extreme Inversion Emergency Divergence Tripwire.
 * 
 * Injects catastrophic tumble driving attitude beyond 90 degrees.
 * Verifies FAULT latching and immediate motor shutdown to 0.0.
 * 
 * @param p_csv_file Optional open FILE* to write telemetry, or NULL
 * @return bool true if scenario passes all acceptance criteria, false otherwise
 */
bool sil_scenario_emergency_tripwire(FILE *p_csv_file);

#ifdef __cplusplus
}
#endif

#endif /* SIL_SCENARIOS_H */
