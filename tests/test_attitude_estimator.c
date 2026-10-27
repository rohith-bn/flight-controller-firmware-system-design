/**
 * @file test_attitude_estimator.c
 * @brief Host Unit Tests for Sensor Fusion Subsystem Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "attitude_estimator.h"
#include <math.h>

static void test_initialization_and_reset(void) {
    TEST_CASE("Attitude Estimator Initialization & Level Defaults");

    attitude_estimator_t est;
    bool ok = attitude_estimator_init(&est, 0.98f, 0.001f);
    TEST_ASSERT(ok == true, "Init succeeds with alpha=0.98, dt=0.001");

    attitude_deg_t att;
    ok = attitude_estimator_get_attitude(&est, &att);
    TEST_ASSERT(ok == true, "Get attitude succeeds");
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 0.0f, 0.001f, "Initial roll is 0.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, 0.0f, 0.001f, "Initial pitch is 0.0 deg");

    /* Reset */
    ok = attitude_estimator_reset(&est);
    TEST_ASSERT(ok == true, "Reset succeeds");
}

static void test_steady_level_tracking(void) {
    TEST_CASE("Steady Level Flight Tracking (100 ticks at 1g Z, zero gyro)");

    attitude_estimator_t est;
    attitude_estimator_init(&est, 0.98f, 0.001f);

    vector3f_t accel = { .x = 0.0f, .y = 0.0f, .z = 9.80665f };
    vector3f_t gyro  = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    attitude_deg_t att_out;

    for (int i = 0; i < 100; i++) {
        bool ok = attitude_estimator_update(&est, &accel, &gyro, &att_out);
        TEST_ASSERT(ok == true, "Update returns true during level flight");
    }

    TEST_ASSERT_FLOAT_NEAR(att_out.roll_deg, 0.0f, 0.001f, "Roll remains 0.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att_out.pitch_deg, 0.0f, 0.001f, "Pitch remains 0.0 deg");
}

static void test_attitude_divergence_fault(void) {
    TEST_CASE("Attitude Divergence Detection (> +/- 90 degrees)");

    attitude_estimator_t est;
    attitude_estimator_init(&est, 0.98f, 0.001f);

    /* Extreme roll excursion: drone inverted or flipping (Roll = +95 degrees) */
    /* Accelerometer: ay = 9.80665 * sin(95 deg) = 9.769, az = 9.80665 * cos(95 deg) = -0.854 */
    /* Gyro: spinning at +200 deg/s roll rate */
    vector3f_t accel_extreme = { .x = 0.0f, .y = 9.77f, .z = -0.85f };
    vector3f_t gyro_extreme  = { .x = 200.0f, .y = 0.0f, .z = 0.0f };
    attitude_deg_t att_out;

    /* Integrate up towards divergence */
    bool diverged = false;
    for (int i = 0; i < 600; i++) {
        bool ok = attitude_estimator_update(&est, &accel_extreme, &gyro_extreme, &att_out);
        if (!ok) {
            diverged = true;
            break;
        }
    }

    TEST_ASSERT(diverged == true, "Unrecoverable attitude divergence (> 90 deg) detected and returned false");
}

int main(void) {
    TEST_SUITE_START("Attitude Estimator (Sensor Fusion Coordinator)");

    test_initialization_and_reset();
    test_steady_level_tracking();
    test_attitude_divergence_fault();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
