/**
 * @file test_system_state_manager.c
 * @brief Host Unit Tests for Safety Lifecycle State Machine & Fault Observers
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "system_state_manager.h"

/* Spy callback variables */
static fault_code_t s_spy_fault_code_1 = FAULT_NONE;
static uint32_t     s_spy_call_count_1 = 0;
static fault_code_t s_spy_fault_code_2 = FAULT_NONE;
static uint32_t     s_spy_call_count_2 = 0;

static void spy_callback_1(fault_code_t code) {
    s_spy_fault_code_1 = code;
    s_spy_call_count_1++;
}

static void spy_callback_2(fault_code_t code) {
    s_spy_fault_code_2 = code;
    s_spy_call_count_2++;
}

static void test_initialization_and_transitions(void) {
    TEST_CASE("State Lifecycle Transitions (DISARMED <-> ARMED)");

    system_state_manager_t mgr;
    bool ok = system_state_manager_init(&mgr);
    TEST_ASSERT(ok == true, "Init succeeds");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&mgr), SYSTEM_STATE_DISARMED, "Boot state is DISARMED");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_fault(&mgr), FAULT_NONE, "Boot fault code is FAULT_NONE");

    /* Arm system */
    ok = system_state_manager_arm(&mgr);
    TEST_ASSERT(ok == true, "Arming from DISARMED succeeds");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&mgr), SYSTEM_STATE_ARMED, "State is now ARMED");

    /* Disarm system */
    ok = system_state_manager_disarm(&mgr);
    TEST_ASSERT(ok == true, "Disarming from ARMED succeeds");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&mgr), SYSTEM_STATE_DISARMED, "State returns to DISARMED");
}

static void test_fault_latch_and_arm_lockout(void) {
    TEST_CASE("Emergency Fault Latch & One-Way Arm Lockout");

    system_state_manager_t mgr;
    system_state_manager_init(&mgr);
    system_state_manager_arm(&mgr);

    /* Assert emergency fault */
    bool ok = system_state_manager_assert_fault(&mgr, FAULT_ATTITUDE_DIVERGENCE);
    TEST_ASSERT(ok == true, "Fault assert returns true");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&mgr), SYSTEM_STATE_FAULT, "State latches to FAULT");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_fault(&mgr), FAULT_ATTITUDE_DIVERGENCE, "Fault code stored");

    /* Attempt to re-arm while in FAULT */
    ok = system_state_manager_arm(&mgr);
    TEST_ASSERT(ok == false, "Arm command strictly rejected while in FAULT");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&mgr), SYSTEM_STATE_FAULT, "State remains locked in FAULT");

    /* Attempt to disarm while in FAULT */
    ok = system_state_manager_disarm(&mgr);
    TEST_ASSERT(ok == false, "Disarm command rejected while in FAULT");
    TEST_ASSERT_INT_EQUAL(system_state_manager_get_state(&mgr), SYSTEM_STATE_FAULT, "State remains locked in FAULT");
}

static void test_motor_gating(void) {
    TEST_CASE("Motor Command Gating Across States");

    system_state_manager_t mgr;
    system_state_manager_init(&mgr);

    motor_commands_t in = { .m1 = 0.6f, .m2 = 0.5f, .m3 = 0.4f, .m4 = 0.7f };
    motor_commands_t out;

    /* 1. In DISARMED state: commands must be forced to 0.0 */
    system_state_manager_gate_motors(&mgr, &in, &out);
    TEST_ASSERT_FLOAT_NEAR(out.m1, 0.0f, 0.0001f, "DISARMED: M1 is 0.0");
    TEST_ASSERT_FLOAT_NEAR(out.m2, 0.0f, 0.0001f, "DISARMED: M2 is 0.0");
    TEST_ASSERT_FLOAT_NEAR(out.m3, 0.0f, 0.0001f, "DISARMED: M3 is 0.0");
    TEST_ASSERT_FLOAT_NEAR(out.m4, 0.0f, 0.0001f, "DISARMED: M4 is 0.0");

    /* 2. In ARMED state: commands pass through verbatim */
    system_state_manager_arm(&mgr);
    system_state_manager_gate_motors(&mgr, &in, &out);
    TEST_ASSERT_FLOAT_NEAR(out.m1, 0.6f, 0.001f, "ARMED: M1 passes through (0.6)");
    TEST_ASSERT_FLOAT_NEAR(out.m2, 0.5f, 0.001f, "ARMED: M2 passes through (0.5)");
    TEST_ASSERT_FLOAT_NEAR(out.m3, 0.4f, 0.001f, "ARMED: M3 passes through (0.4)");
    TEST_ASSERT_FLOAT_NEAR(out.m4, 0.7f, 0.001f, "ARMED: M4 passes through (0.7)");

    /* 3. In FAULT state: commands immediately forced to 0.0 */
    system_state_manager_assert_fault(&mgr, FAULT_SENSOR_SATURATION);
    system_state_manager_gate_motors(&mgr, &in, &out);
    TEST_ASSERT_FLOAT_NEAR(out.m1, 0.0f, 0.0001f, "FAULT: M1 forced to 0.0");
    TEST_ASSERT_FLOAT_NEAR(out.m2, 0.0f, 0.0001f, "FAULT: M2 forced to 0.0");
    TEST_ASSERT_FLOAT_NEAR(out.m3, 0.0f, 0.0001f, "FAULT: M3 forced to 0.0");
    TEST_ASSERT_FLOAT_NEAR(out.m4, 0.0f, 0.0001f, "FAULT: M4 forced to 0.0");
}

static void test_observer_pattern_callbacks(void) {
    TEST_CASE("Observer Pattern: Fault Event Notifications");

    s_spy_fault_code_1 = FAULT_NONE;
    s_spy_call_count_1 = 0;
    s_spy_fault_code_2 = FAULT_NONE;
    s_spy_call_count_2 = 0;

    system_state_manager_t mgr;
    system_state_manager_init(&mgr);

    bool ok1 = system_state_manager_register_fault_callback(&mgr, spy_callback_1);
    bool ok2 = system_state_manager_register_fault_callback(&mgr, spy_callback_2);
    TEST_ASSERT(ok1 == true && ok2 == true, "Registered 2 fault callbacks successfully");

    /* Trigger fault */
    system_state_manager_assert_fault(&mgr, FAULT_IMU_COMM_FAIL);

    TEST_ASSERT_INT_EQUAL(s_spy_call_count_1, 1, "Observer 1 notified exactly once");
    TEST_ASSERT_INT_EQUAL(s_spy_fault_code_1, FAULT_IMU_COMM_FAIL, "Observer 1 received FAULT_IMU_COMM_FAIL");
    TEST_ASSERT_INT_EQUAL(s_spy_call_count_2, 1, "Observer 2 notified exactly once");
    TEST_ASSERT_INT_EQUAL(s_spy_fault_code_2, FAULT_IMU_COMM_FAIL, "Observer 2 received FAULT_IMU_COMM_FAIL");
}

int main(void) {
    TEST_SUITE_START("System State Manager (Safety FSM & Observer Gatekeeper)");

    test_initialization_and_transitions();
    test_fault_latch_and_arm_lockout();
    test_motor_gating();
    test_observer_pattern_callbacks();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
