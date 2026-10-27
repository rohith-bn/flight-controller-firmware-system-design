/**
 * @file main_sil.c
 * @brief Software-in-the-Loop Simulation Entry Point and Runner CLI
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 Compliance
 */

#include "sil_scenarios.h"
#include <stdio.h>
#include <stdbool.h>

#define ANSI_GREEN   "\033[1;32m"
#define ANSI_RED     "\033[1;31m"
#define ANSI_YELLOW  "\033[1;33m"
#define ANSI_CYAN    "\033[1;36m"
#define ANSI_RESET   "\033[0m"

int main(void) {
    printf("\n");
    printf(ANSI_CYAN "==========================================================\n" ANSI_RESET);
    printf(ANSI_CYAN "  QFC-1 FLIGHT CONTROLLER - CLOSED-LOOP SIL SIMULATOR      \n" ANSI_RESET);
    printf(ANSI_CYAN "  6-DOF Rigid-Body Dynamics & 1000 Hz Software-in-the-Loop \n" ANSI_RESET);
    printf(ANSI_CYAN "==========================================================\n\n" ANSI_RESET);

    const char *csv_path = "./simulation/logs/sil_telemetry.csv";
    FILE *fp = fopen(csv_path, "w");
    if (fp != NULL) {
        fprintf(fp, "scenario,time_s,true_roll_deg,true_pitch_deg,est_roll_deg,est_pitch_deg,target_roll_deg,target_pitch_deg,m1,m2,m3,m4,state,fault\n");
    } else {
        printf(ANSI_YELLOW "[WARN] Unable to open %s for writing. Continuing in memory.\n" ANSI_RESET, csv_path);
    }

    int passed_count = 0;
    int total_count  = 4;

    /* --- Scenario 1 --- */
    printf("[*] Running Scenario 1: Initial Severe Tilt Recovery (+20 deg Roll, -15 deg Pitch)...\n");
    bool sc1 = sil_scenario_tilt_recovery(fp);
    if (sc1) {
        printf("    " ANSI_GREEN "[PASS]" ANSI_RESET " Converged to level flight (< 0.5 deg) with zero persistent error.\n\n");
        passed_count++;
    } else {
        printf("    " ANSI_RED "[FAIL]" ANSI_RESET " Failed to achieve attitude stabilization.\n\n");
    }

    /* --- Scenario 2 --- */
    printf("[*] Running Scenario 2: Wind Gust Torque Impulse Rejection (0.04 N*m for 80 ms)...\n");
    bool sc2 = sil_scenario_gust_rejection(fp);
    if (sc2) {
        printf("    " ANSI_GREEN "[PASS]" ANSI_RESET " Maximum tilt clamped < 15 deg; fully stabilized within 500 ms.\n\n");
        passed_count++;
    } else {
        printf("    " ANSI_RED "[FAIL]" ANSI_RESET " Failed to reject disturbance.\n\n");
    }

    /* --- Scenario 3 --- */
    printf("[*] Running Scenario 3: Target Attitude Dynamic Setpoint Tracking (0 -> +12 -> -12 -> 0 deg)...\n");
    bool sc3 = sil_scenario_setpoint_tracking(fp);
    if (sc3) {
        printf("    " ANSI_GREEN "[PASS]" ANSI_RESET " Setpoints tracked with rapid rise time and < 0.8 deg steady-state error.\n\n");
        passed_count++;
    } else {
        printf("    " ANSI_RED "[FAIL]" ANSI_RESET " Failed setpoint tracking criteria.\n\n");
    }

    /* --- Scenario 4 --- */
    printf("[*] Running Scenario 4: Extreme Inversion Emergency Tripwire (External Tumble > 90 deg)...\n");
    bool sc4 = sil_scenario_emergency_tripwire(fp);
    if (sc4) {
        printf("    " ANSI_GREEN "[PASS]" ANSI_RESET " FAULT_ATTITUDE_DIVERGENCE asserted immediately; all motors forced to 0.0.\n\n");
        passed_count++;
    } else {
        printf("    " ANSI_RED "[FAIL]" ANSI_RESET " Emergency tripwire failed to shut down motors.\n\n");
    }

    if (fp != NULL) {
        fclose(fp);
        printf("[+] Telemetry stream logged to: %s\n", csv_path);
    }

    printf("\n" ANSI_CYAN "==========================================================\n" ANSI_RESET);
    printf("  SIL SIMULATION RESULTS: %d / %d SCENARIOS PASSED\n", passed_count, total_count);
    if (passed_count == total_count) {
        printf("  " ANSI_GREEN "[SUCCESS] 100%% OF CLOSED-LOOP FLIGHT SCENARIOS PASSED!\n" ANSI_RESET);
    } else {
        printf("  " ANSI_RED "[FAILURE] Some flight scenarios failed.\n" ANSI_RESET);
    }
    printf(ANSI_CYAN "==========================================================\n\n" ANSI_RESET);

    return (passed_count == total_count) ? 0 : 1;
}
