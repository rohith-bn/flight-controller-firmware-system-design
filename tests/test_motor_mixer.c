/**
 * @file test_motor_mixer.c
 * @brief Host Unit Tests for Quad-X Geometry Motor Thrust Mixer
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "motor_mixer.h"

static void test_hover_equilibrium(void) {
    TEST_CASE("Pure Throttle Hover Equilibrium (Zero PID Demands)");

    motor_commands_t motors;
    bool ok = motor_mixer_compute(0.50f, 0.0f, 0.0f, &motors);

    TEST_ASSERT(ok == true, "Hover mixing returns true");
    TEST_ASSERT_FLOAT_NEAR(motors.m1, 0.50f, 0.001f, "M1 (FR) is 0.50");
    TEST_ASSERT_FLOAT_NEAR(motors.m2, 0.50f, 0.001f, "M2 (RR) is 0.50");
    TEST_ASSERT_FLOAT_NEAR(motors.m3, 0.50f, 0.001f, "M3 (RL) is 0.50");
    TEST_ASSERT_FLOAT_NEAR(motors.m4, 0.50f, 0.001f, "M4 (FL) is 0.50");
}

static void test_differential_roll_and_pitch(void) {
    TEST_CASE("Differential Roll and Pitch Demands");

    motor_commands_t motors;

    /* Roll demand = +0.10: Right motors (M1, M3) increase, Left motors (M2, M4) decrease */
    motor_mixer_compute(0.50f, 0.10f, 0.0f, &motors);
    TEST_ASSERT_FLOAT_NEAR(motors.m1, 0.60f, 0.001f, "M1 = 0.5 + 0.1 = 0.60");
    TEST_ASSERT_FLOAT_NEAR(motors.m2, 0.40f, 0.001f, "M2 = 0.5 - 0.1 = 0.40");
    TEST_ASSERT_FLOAT_NEAR(motors.m3, 0.60f, 0.001f, "M3 = 0.5 + 0.1 = 0.60");
    TEST_ASSERT_FLOAT_NEAR(motors.m4, 0.40f, 0.001f, "M4 = 0.5 - 0.1 = 0.40");

    /* Pitch demand = +0.10: Rear motors (M3, M4) increase, Front motors (M1, M2) decrease */
    motor_mixer_compute(0.50f, 0.0f, 0.10f, &motors);
    TEST_ASSERT_FLOAT_NEAR(motors.m1, 0.40f, 0.001f, "M1 = 0.5 - 0.1 = 0.40");
    TEST_ASSERT_FLOAT_NEAR(motors.m2, 0.40f, 0.001f, "M2 = 0.5 - 0.1 = 0.40");
    TEST_ASSERT_FLOAT_NEAR(motors.m3, 0.60f, 0.001f, "M3 = 0.5 + 0.1 = 0.60");
    TEST_ASSERT_FLOAT_NEAR(motors.m4, 0.60f, 0.001f, "M4 = 0.5 + 0.1 = 0.60");
}

static void test_output_saturation_clamping(void) {
    TEST_CASE("Output Saturation Clamping [0.0, 1.0]");

    motor_commands_t motors;

    /* Upper clamping: Throttle = 0.95, Roll = +0.10 -> M1, M3 unconstrained = 1.05 -> clamped to 1.00 */
    bool ok = motor_mixer_compute(0.95f, 0.10f, 0.0f, &motors);
    TEST_ASSERT(ok == true, "Minor saturation within extreme bounds returns true");
    TEST_ASSERT_FLOAT_NEAR(motors.m1, 1.00f, 0.001f, "M1 clamped at 1.00");
    TEST_ASSERT_FLOAT_NEAR(motors.m3, 1.00f, 0.001f, "M3 clamped at 1.00");

    /* Lower clamping: Throttle = 0.05, Roll = +0.10 -> M2, M4 unconstrained = -0.05 -> clamped to 0.00 */
    ok = motor_mixer_compute(0.05f, 0.10f, 0.0f, &motors);
    TEST_ASSERT(ok == true, "Minor negative saturation within extreme bounds returns true");
    TEST_ASSERT_FLOAT_NEAR(motors.m2, 0.00f, 0.001f, "M2 clamped at 0.00");
    TEST_ASSERT_FLOAT_NEAR(motors.m4, 0.00f, 0.001f, "M4 clamped at 0.00");
}

static void test_runaway_fault_detection(void) {
    TEST_CASE("Extreme Runaway Bounds Fault Detection (> 1.25 or < -0.25)");

    motor_commands_t motors;

    /* Wild roll demand = 0.85 -> M1 = 0.50 + 0.85 = 1.35 > 1.25 -> Runaway fault! */
    bool ok = motor_mixer_compute(0.50f, 0.85f, 0.0f, &motors);
    TEST_ASSERT(ok == false, "Extreme positive runaway demand rejected with false");

    /* Wild negative throttle = -0.50 -> Runaway fault! */
    ok = motor_mixer_compute(-0.50f, 0.0f, 0.0f, &motors);
    TEST_ASSERT(ok == false, "Negative extreme throttle rejected with false");
}

static void test_motor_zeroing(void) {
    TEST_CASE("Motor Zeroing Safeguard");

    motor_commands_t motors = { .m1 = 0.8f, .m2 = 0.7f, .m3 = 0.9f, .m4 = 0.6f };
    motor_mixer_zero(&motors);

    TEST_ASSERT_FLOAT_NEAR(motors.m1, 0.0f, 0.0001f, "M1 forced 0.0");
    TEST_ASSERT_FLOAT_NEAR(motors.m2, 0.0f, 0.0001f, "M2 forced 0.0");
    TEST_ASSERT_FLOAT_NEAR(motors.m3, 0.0f, 0.0001f, "M3 forced 0.0");
    TEST_ASSERT_FLOAT_NEAR(motors.m4, 0.0f, 0.0001f, "M4 forced 0.0");
}

int main(void) {
    TEST_SUITE_START("Motor Mixer (Quad-X Geometry and Safety Limiter)");

    test_hover_equilibrium();
    test_differential_roll_and_pitch();
    test_output_saturation_clamping();
    test_runaway_fault_detection();
    test_motor_zeroing();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
