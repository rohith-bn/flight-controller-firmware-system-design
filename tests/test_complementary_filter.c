/**
 * @file test_complementary_filter.c
 * @brief Host Unit Tests for Discrete-Time Complementary Orientation Filter
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "complementary_filter.h"
#include <math.h>

static void test_initialization_and_reset(void) {
    TEST_CASE("Filter Initialization and State Reset");

    complementary_filter_t filter;
    bool ok = complementary_filter_init(&filter, 0.98f, 0.001f);
    TEST_ASSERT(ok == true, "Filter initializes successfully with valid params");

    attitude_deg_t att;
    complementary_filter_get_attitude(&filter, &att);
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 0.0f, 0.001f, "Initial roll is 0.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, 0.0f, 0.001f, "Initial pitch is 0.0 deg");

    /* Reset to non-zero baseline */
    ok = complementary_filter_reset(&filter, 15.0f, -20.0f);
    TEST_ASSERT(ok == true, "Filter resets to baseline angles");
    complementary_filter_get_attitude(&filter, &att);
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 15.0f, 0.001f, "Reset roll is 15.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, -20.0f, 0.001f, "Reset pitch is -20.0 deg");

    /* Defensive: invalid init params */
    TEST_ASSERT(complementary_filter_init(&filter, -0.1f, 0.001f) == false, "Negative alpha rejected");
    TEST_ASSERT(complementary_filter_init(&filter, 1.05f, 0.001f) == false, "Alpha > 1.0 rejected");
    TEST_ASSERT(complementary_filter_init(&filter, 0.98f, 0.0f) == false, "Zero dt rejected");
    TEST_ASSERT(complementary_filter_init(NULL, 0.98f, 0.001f) == false, "NULL filter rejected");
}

static void test_pure_gyro_integration(void) {
    TEST_CASE("Pure Gyro Dynamic Rate Integration (alpha = 1.0)");

    complementary_filter_t filter;
    /* Use alpha = 1.0 to isolate gyro integration */
    complementary_filter_init(&filter, 1.0f, 0.001f);

    attitude_deg_t accel_att = { .roll_deg = 0.0f, .pitch_deg = 0.0f };
    /* Constant gyro angular velocity: Roll = +100 deg/s, Pitch = -50 deg/s */
    vector3f_t gyro_dps = { .x = 100.0f, .y = -50.0f, .z = 0.0f };

    attitude_deg_t att_out;

    /* Run for 1000 ticks (exactly 1.0 second) */
    for (int i = 0; i < 1000; i++) {
        complementary_filter_update(&filter, &accel_att, &gyro_dps, &att_out);
    }

    /* Expected: Roll = 0 + 100 * 1.0 = 100.0 deg, Pitch = 0 + (-50) * 1.0 = -50.0 deg */
    TEST_ASSERT_FLOAT_NEAR(att_out.roll_deg, 100.0f, 0.1f, "100 deg/s over 1.0 s integrates to 100 deg roll");
    TEST_ASSERT_FLOAT_NEAR(att_out.pitch_deg, -50.0f, 0.1f, "-50 deg/s over 1.0 s integrates to -50 deg pitch");
}

static void test_accel_convergence_on_step(void) {
    TEST_CASE("Accelerometer Convergence on Step Input (gyro = 0, alpha = 0.98)");

    complementary_filter_t filter;
    complementary_filter_init(&filter, 0.98f, 0.001f);

    /* Gyro is still (0 deg/s), Accel suddenly steps to 10.0 degrees */
    vector3f_t gyro_dps = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    attitude_deg_t accel_att = { .roll_deg = 10.0f, .pitch_deg = -10.0f };
    attitude_deg_t att_out;

    /* After 1 tick: output = 0.98*(0) + 0.02*(10.0) = 0.20 deg */
    complementary_filter_update(&filter, &accel_att, &gyro_dps, &att_out);
    TEST_ASSERT_FLOAT_NEAR(att_out.roll_deg, 0.20f, 0.01f, "Tick 1 gives 2% of accel step (0.20 deg)");

    /* Run for 300 ticks (0.3 seconds): 1 - 0.98^300 = 1 - 0.0023 = 99.77% convergence */
    for (int i = 1; i < 300; i++) {
        complementary_filter_update(&filter, &accel_att, &gyro_dps, &att_out);
    }

    TEST_ASSERT_FLOAT_NEAR(att_out.roll_deg, 10.0f, 0.1f, "Roll converges to 10.0 deg within 300 ms");
    TEST_ASSERT_FLOAT_NEAR(att_out.pitch_deg, -10.0f, 0.1f, "Pitch converges to -10.0 deg within 300 ms");
}

int main(void) {
    TEST_SUITE_START("Complementary Filter (Discrete Orientation Fusion)");

    test_initialization_and_reset();
    test_pure_gyro_integration();
    test_accel_convergence_on_step();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
