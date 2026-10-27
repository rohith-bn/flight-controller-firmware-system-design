/**
 * @file test_setpoint_manager.c
 * @brief Host Unit Tests for Target Orientation Setpoint Manager
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "setpoint_manager.h"

static void test_initialization(void) {
    TEST_CASE("Setpoint Manager Initialization (Level Hover Defaults)");

    setpoint_manager_t mgr;
    bool ok = setpoint_manager_init(&mgr);
    TEST_ASSERT(ok == true, "Initialization succeeds");

    setpoint_deg_t sp;
    ok = setpoint_manager_get(&mgr, &sp);
    TEST_ASSERT(ok == true, "Get succeeds after init");
    TEST_ASSERT_FLOAT_NEAR(sp.roll_deg, 0.0f, 0.001f, "Default roll setpoint is 0.0 deg");
    TEST_ASSERT_FLOAT_NEAR(sp.pitch_deg, 0.0f, 0.001f, "Default pitch setpoint is 0.0 deg");

    /* Defensive: NULL check */
    TEST_ASSERT(setpoint_manager_init(NULL) == false, "NULL init rejected");
}

static void test_valid_setpoint_updates(void) {
    TEST_CASE("Valid Setpoint Updates Within Safe Tilt Limits (+/- 45 deg)");

    setpoint_manager_t mgr;
    setpoint_manager_init(&mgr);

    bool ok = setpoint_manager_set(&mgr, 15.0f, -20.0f);
    TEST_ASSERT(ok == true, "Setting (15.0, -20.0) within +/- 45 deg succeeds");

    setpoint_deg_t sp;
    setpoint_manager_get(&mgr, &sp);
    TEST_ASSERT_FLOAT_NEAR(sp.roll_deg, 15.0f, 0.001f, "Retrieved roll is 15.0 deg");
    TEST_ASSERT_FLOAT_NEAR(sp.pitch_deg, -20.0f, 0.001f, "Retrieved pitch is -20.0 deg");
}

static void test_boundary_saturation_clamping(void) {
    TEST_CASE("Boundary Saturation Clamping Beyond Max Tilt Limits");

    setpoint_manager_t mgr;
    setpoint_manager_init(&mgr);

    /* Request 60.0 deg roll (exceeds +45.0) and -50.0 deg pitch (exceeds -45.0) */
    bool ok = setpoint_manager_set(&mgr, 60.0f, -50.0f);
    TEST_ASSERT(ok == false, "Out-of-bounds target returns false to signal saturation");

    setpoint_deg_t sp;
    setpoint_manager_get(&mgr, &sp);
    TEST_ASSERT_FLOAT_NEAR(sp.roll_deg, 45.0f, 0.001f, "Roll setpoint clamped to +45.0 deg");
    TEST_ASSERT_FLOAT_NEAR(sp.pitch_deg, -45.0f, 0.001f, "Pitch setpoint clamped to -45.0 deg");
}

static void test_reset(void) {
    TEST_CASE("Setpoint Reset to Default Level Hover");

    setpoint_manager_t mgr;
    setpoint_manager_init(&mgr);
    setpoint_manager_set(&mgr, 25.0f, 30.0f);

    bool ok = setpoint_manager_reset(&mgr);
    TEST_ASSERT(ok == true, "Reset succeeds");

    setpoint_deg_t sp;
    setpoint_manager_get(&mgr, &sp);
    TEST_ASSERT_FLOAT_NEAR(sp.roll_deg, 0.0f, 0.001f, "Roll reset to 0.0 deg");
    TEST_ASSERT_FLOAT_NEAR(sp.pitch_deg, 0.0f, 0.001f, "Pitch reset to 0.0 deg");
}

int main(void) {
    TEST_SUITE_START("Setpoint Manager (Target Orientation ADT)");

    test_initialization();
    test_valid_setpoint_updates();
    test_boundary_saturation_clamping();
    test_reset();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
