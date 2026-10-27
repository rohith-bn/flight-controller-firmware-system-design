/**
 * @file test_flight_controller.c
 * @brief Host Unit Tests for 1 ms Deterministic Pipeline Orchestrator
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "flight_controller.h"
#include "mock_imu_comm.h"
#include "mock_pwm_driver.h"

static void test_startup_and_calibration_gate(void) {
    TEST_CASE("Flight Controller Boot & Calibration Arming Gate");

    mock_imu_comm_reset();
    mock_pwm_driver_reset();
    mock_imu_comm_set_whoami(0x68);

    flight_controller_t fc;
    pid_axis_config_t cfg = {
        .kp = 1.0f, .ki = 0.05f, .kd = 0.1f, .windup_limit = 25.0f, .dt_s = 0.001f
    };

    bool ok = flight_controller_init(&fc, &cfg, &cfg);
    TEST_ASSERT(ok == true, "Top-level init succeeds");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&fc.state_mgr), SYSTEM_STATE_DISARMED, "Boot state is DISARMED");

    /* Attempt to arm before calibration */
    ok = flight_controller_arm(&fc);
    TEST_ASSERT(ok == false, "Arming strictly rejected before calibration completes");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&fc.state_mgr), SYSTEM_STATE_DISARMED, "State remains DISARMED");

    /* Run 512 calibration steps */
    mock_imu_comm_set_canned_raw(0, 0, 16384, 0, 0, 0, 0);
    bool b_complete = false;
    for (int i = 0; i < 512; i++) {
        flight_controller_calibrate_step(&fc, &b_complete);
    }
    TEST_ASSERT(b_complete == true, "Calibration complete at 512 samples");

    /* Now arming must succeed */
    ok = flight_controller_arm(&fc);
    TEST_ASSERT(ok == true, "Arming succeeds after calibration");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&fc.state_mgr), SYSTEM_STATE_ARMED, "State is now ARMED");
}

static void test_end_to_end_1ms_pipeline(void) {
    TEST_CASE("End-to-End 1 ms Super Loop Pipeline Execution (ARMED Hover)");

    mock_imu_comm_reset();
    mock_pwm_driver_reset();
    mock_imu_comm_set_whoami(0x68);

    flight_controller_t fc;
    pid_axis_config_t cfg = {
        .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .windup_limit = 25.0f, .dt_s = 0.001f
    };
    flight_controller_init(&fc, &cfg, &cfg);

    /* Calibrate and Arm */
    mock_imu_comm_set_canned_raw(0, 0, 16384, 0, 0, 0, 0);
    bool b_complete = false;
    for (int i = 0; i < 512; i++) {
        flight_controller_calibrate_step(&fc, &b_complete);
    }
    flight_controller_arm(&fc);

    /* Set hover throttle = 0.50 */
    flight_controller_set_throttle(&fc, 0.50f);

    /* Execute 10 ticks in steady level flight */
    for (int i = 0; i < 10; i++) {
        bool ok = flight_controller_process_tick(&fc);
        TEST_ASSERT(ok == true, "1 ms control tick succeeds");
    }

    /* Verify motor outputs via mock_pwm_driver */
    motor_commands_t active;
    pwm_driver_get_active_thrust(&active);
    TEST_ASSERT_FLOAT_NEAR(active.m1, 0.50f, 0.001f, "M1 driven at 0.50");
    TEST_ASSERT_FLOAT_NEAR(active.m2, 0.50f, 0.001f, "M2 driven at 0.50");
    TEST_ASSERT_FLOAT_NEAR(active.m3, 0.50f, 0.001f, "M3 driven at 0.50");
    TEST_ASSERT_FLOAT_NEAR(active.m4, 0.50f, 0.001f, "M4 driven at 0.50");
}

static void test_emergency_divergence_tripwire(void) {
    TEST_CASE("Emergency Divergence Tripwire Shuts Off Motors Immediately");

    mock_imu_comm_reset();
    mock_pwm_driver_reset();
    mock_imu_comm_set_whoami(0x68);

    flight_controller_t fc;
    pid_axis_config_t cfg = {
        .kp = 0.002f, .ki = 0.0f, .kd = 0.0f, .windup_limit = 25.0f, .dt_s = 0.001f
    };
    flight_controller_init(&fc, &cfg, &cfg);

    /* Calibrate, Arm, and fly at 0.60 throttle */
    mock_imu_comm_set_canned_raw(0, 0, 16384, 0, 0, 0, 0);
    bool b_complete = false;
    for (int i = 0; i < 512; i++) {
        flight_controller_calibrate_step(&fc, &b_complete);
    }
    flight_controller_arm(&fc);
    flight_controller_set_throttle(&fc, 0.60f);
    flight_controller_process_tick(&fc);

    /* Inject extreme flip (> 90 deg roll divergence) */
    mock_imu_comm_set_canned_raw(0, 16300, -1400, 26200, 0, 0, 0);

    /* Process ticks until divergence trips */
    for (int i = 0; i < 600; i++) {
        flight_controller_process_tick(&fc);
        if (system_state_manager_get_state(&fc.state_mgr) == SYSTEM_STATE_FAULT) {
            break;
        }
    }

    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&fc.state_mgr), SYSTEM_STATE_FAULT, "State latched to FAULT");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_fault(&fc.state_mgr), FAULT_ATTITUDE_DIVERGENCE, "Fault is ATTITUDE_DIVERGENCE");

    /* Verify PWM outputs were forced to zero */
    motor_commands_t active;
    pwm_driver_get_active_thrust(&active);
    TEST_ASSERT_FLOAT_NEAR(active.m1, 0.0f, 0.0001f, "M1 forced 0.0");
    TEST_ASSERT_FLOAT_NEAR(active.m2, 0.0f, 0.0001f, "M2 forced 0.0");
    TEST_ASSERT_FLOAT_NEAR(active.m3, 0.0f, 0.0001f, "M3 forced 0.0");
    TEST_ASSERT_FLOAT_NEAR(active.m4, 0.0f, 0.0001f, "M4 forced 0.0");
}

int main(void) {
    TEST_SUITE_START("Flight Controller (Top-Level 1 ms Super Loop Orchestrator)");

    test_startup_and_calibration_gate();
    test_end_to_end_1ms_pipeline();
    test_emergency_divergence_tripwire();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
