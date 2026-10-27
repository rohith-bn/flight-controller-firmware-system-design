/**
 * @file sil_scenarios.c
 * @brief Automated Flight Simulation Verification Scenarios Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 Compliance
 */

#include "sil_scenarios.h"
#include "sil_harness.h"
#include <math.h>

static void log_telemetry_csv(FILE *fp, const char *scenario_name, const sil_telemetry_t *t) {
    if (fp != NULL) {
        fprintf(fp, "%s,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%d,%d\n",
                scenario_name,
                t->time_s,
                t->true_roll_deg,
                t->true_pitch_deg,
                t->est_roll_deg,
                t->est_pitch_deg,
                t->target_roll_deg,
                t->target_pitch_deg,
                t->m1_thrust,
                t->m2_thrust,
                t->m3_thrust,
                t->m4_thrust,
                (int)t->sys_state,
                (int)t->fault);
    }
}

bool sil_scenario_tilt_recovery(FILE *p_csv_file) {
    sil_harness_t harness;
    /* Initial severe tilt: +20.0 deg roll, -15.0 deg pitch */
    if (!sil_harness_init(&harness, 20.0f, -15.0f, 0.45f, NULL)) {
        return false;
    }

    sil_telemetry_t telem;
    /* Run 1500 ms closed-loop simulation */
    for (int i = 0; i < 1500; i++) {
        (void)sil_harness_step(&harness);
        sil_harness_get_telemetry(&harness, &telem);
        log_telemetry_csv(p_csv_file, "TiltRecovery", &telem);
    }

    /* Verification criteria */
    bool ok_state = (telem.sys_state == SYSTEM_STATE_ARMED) && (telem.fault == FAULT_NONE);
    bool ok_roll  = (fabsf(telem.true_roll_deg) < 0.5f);
    bool ok_pitch = (fabsf(telem.true_pitch_deg) < 0.5f);

    return (ok_state && ok_roll && ok_pitch);
}

bool sil_scenario_gust_rejection(FILE *p_csv_file) {
    sil_harness_t harness;
    /* Start level hover */
    if (!sil_harness_init(&harness, 0.0f, 0.0f, 0.45f, NULL)) {
        return false;
    }

    sil_telemetry_t telem;
    float max_roll_deflection = 0.0f;

    /* Run 1500 ms simulation */
    for (int i = 0; i < 1500; i++) {
        /* Apply wind gust torque impulse between 500 ms and 580 ms */
        if (i == 500) {
            sil_harness_apply_disturbance(&harness, 0.04f, 0.0f);
        } else if (i == 580) {
            sil_harness_apply_disturbance(&harness, 0.0f, 0.0f);
        }

        (void)sil_harness_step(&harness);
        sil_harness_get_telemetry(&harness, &telem);
        log_telemetry_csv(p_csv_file, "GustRejection", &telem);

        if (fabsf(telem.true_roll_deg) > max_roll_deflection) {
            max_roll_deflection = fabsf(telem.true_roll_deg);
        }
    }

    /* Verification criteria */
    bool ok_state = (telem.sys_state == SYSTEM_STATE_ARMED) && (telem.fault == FAULT_NONE);
    bool ok_peak  = (max_roll_deflection < 15.0f);      /* Maximum deflection stays within safe limits */
    bool ok_settle = (fabsf(telem.true_roll_deg) < 0.5f); /* Restores level hover */

    return (ok_state && ok_peak && ok_settle);
}

bool sil_scenario_setpoint_tracking(FILE *p_csv_file) {
    sil_harness_t harness;
    /* Start level hover */
    if (!sil_harness_init(&harness, 0.0f, 0.0f, 0.45f, NULL)) {
        return false;
    }

    sil_telemetry_t telem;
    float err_pos_step = 999.0f;
    float err_neg_step = 999.0f;
    float err_return   = 999.0f;

    /* Run 3500 ms simulation with setpoint changes */
    for (int i = 0; i < 3500; i++) {
        if (i == 500) {
            sil_harness_set_setpoint(&harness, 12.0f, 0.0f);
        } else if (i == 1500) {
            sil_harness_set_setpoint(&harness, -12.0f, 0.0f);
        } else if (i == 2500) {
            sil_harness_set_setpoint(&harness, 0.0f, 0.0f);
        }

        (void)sil_harness_step(&harness);
        sil_harness_get_telemetry(&harness, &telem);
        log_telemetry_csv(p_csv_file, "SetpointTracking", &telem);

        /* Sample tracking error near end of each step window */
        if (i == 1450) {
            err_pos_step = fabsf(telem.true_roll_deg - 12.0f);
        } else if (i == 2450) {
            err_neg_step = fabsf(telem.true_roll_deg - (-12.0f));
        } else if (i == 3450) {
            err_return   = fabsf(telem.true_roll_deg - 0.0f);
        }
    }

    bool ok_state = (telem.sys_state == SYSTEM_STATE_ARMED) && (telem.fault == FAULT_NONE);
    bool ok_pos   = (err_pos_step < 0.8f);
    bool ok_neg   = (err_neg_step < 0.8f);
    bool ok_ret   = (err_return < 0.8f);

    return (ok_state && ok_pos && ok_neg && ok_ret);
}

bool sil_scenario_emergency_tripwire(FILE *p_csv_file) {
    sil_harness_t harness;
    /* Start level hover */
    if (!sil_harness_init(&harness, 0.0f, 0.0f, 0.45f, NULL)) {
        return false;
    }

    sil_telemetry_t telem;
    bool b_fault_latched = false;

    /* Run simulation, inject extreme tumble at 200 ms */
    for (int i = 0; i < 800; i++) {
        if (i == 200) {
            /* Steady overpowering torque that drives aircraft past 90 deg divergence threshold */
            sil_harness_apply_disturbance(&harness, 0.45f, 0.0f);
        }

        (void)sil_harness_step(&harness);
        sil_harness_get_telemetry(&harness, &telem);
        log_telemetry_csv(p_csv_file, "EmergencyTripwire", &telem);

        if (telem.sys_state == SYSTEM_STATE_FAULT) {
            b_fault_latched = true;
            break;
        }
    }

    /* Verify fault latching, divergence fault code, and complete motor shutoff */
    bool ok_latch = b_fault_latched;
    bool ok_code  = (telem.fault == FAULT_ATTITUDE_DIVERGENCE);
    bool ok_m1    = (telem.m1_thrust == 0.0f);
    bool ok_m2    = (telem.m2_thrust == 0.0f);
    bool ok_m3    = (telem.m3_thrust == 0.0f);
    bool ok_m4    = (telem.m4_thrust == 0.0f);

    return (ok_latch && ok_code && ok_m1 && ok_m2 && ok_m3 && ok_m4);
}
