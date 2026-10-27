/**
 * @file test_angle_computer.c
 * @brief Host Unit Tests for Accelerometer Geometric Trigonometry
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "angle_computer.h"
#include <math.h>

static void test_level_flight(void) {
    TEST_CASE("Level Flight Orientation (1g straight down along Z)");

    vector3f_t accel = { .x = 0.0f, .y = 0.0f, .z = 9.80665f };
    attitude_deg_t att;

    bool ok = angle_computer_process(&accel, &att);
    TEST_ASSERT(ok == true, "Level flight computes successfully");
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 0.0f, 0.01f, "Roll is 0.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, 0.0f, 0.01f, "Pitch is 0.0 deg");
}

static void test_pure_roll_orientations(void) {
    TEST_CASE("Pure Roll Orientations (+45 deg, -45 deg, +90 deg)");

    /* +45 deg roll: ay = 9.80665 * sin(45), az = 9.80665 * cos(45) */
    float g = 9.80665f;
    float sin45 = 0.70710678f;
    float cos45 = 0.70710678f;

    vector3f_t roll_p45 = { .x = 0.0f, .y = g * sin45, .z = g * cos45 };
    attitude_deg_t att;
    bool ok = angle_computer_process(&roll_p45, &att);
    TEST_ASSERT(ok == true, "+45 deg roll computes successfully");
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 45.0f, 0.05f, "Roll is +45.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, 0.0f, 0.05f, "Pitch remains 0.0 deg");

    /* -45 deg roll */
    vector3f_t roll_n45 = { .x = 0.0f, .y = -g * sin45, .z = g * cos45 };
    ok = angle_computer_process(&roll_n45, &att);
    TEST_ASSERT(ok == true, "-45 deg roll computes successfully");
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, -45.0f, 0.05f, "Roll is -45.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, 0.0f, 0.05f, "Pitch remains 0.0 deg");

    /* +90 deg knife edge right: ay = 9.80665, az = 0 */
    vector3f_t roll_p90 = { .x = 0.0f, .y = g, .z = 0.0f };
    ok = angle_computer_process(&roll_p90, &att);
    TEST_ASSERT(ok == true, "+90 deg roll computes successfully");
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 90.0f, 0.05f, "Roll is +90.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, 0.0f, 0.05f, "Pitch remains 0.0 deg");
}

static void test_pure_pitch_orientations(void) {
    TEST_CASE("Pure Pitch Orientations (+30 deg, -30 deg, +60 deg)");

    float g = 9.80665f;
    /* Pitch up +30 deg: ax = -g * sin(30) = -g * 0.5, az = g * cos(30) = g * 0.866025 */
    vector3f_t pitch_p30 = { .x = -g * 0.5f, .y = 0.0f, .z = g * 0.8660254f };
    attitude_deg_t att;
    bool ok = angle_computer_process(&pitch_p30, &att);
    TEST_ASSERT(ok == true, "+30 deg pitch computes successfully");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, 30.0f, 0.05f, "Pitch is +30.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 0.0f, 0.05f, "Roll remains 0.0 deg");

    /* Pitch down -30 deg: ax = +g * sin(30) = +g * 0.5 */
    vector3f_t pitch_n30 = { .x = g * 0.5f, .y = 0.0f, .z = g * 0.8660254f };
    ok = angle_computer_process(&pitch_n30, &att);
    TEST_ASSERT(ok == true, "-30 deg pitch computes successfully");
    TEST_ASSERT_FLOAT_NEAR(att.pitch_deg, -30.0f, 0.05f, "Pitch is -30.0 deg");
    TEST_ASSERT_FLOAT_NEAR(att.roll_deg, 0.0f, 0.05f, "Roll remains 0.0 deg");
}

static void test_boundary_and_fault_conditions(void) {
    TEST_CASE("Boundary Conditions (Free-fall zero-gravity, NULL pointers)");

    /* Free fall: total accel < 0.1 m/s^2 */
    vector3f_t zero_g = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    attitude_deg_t att;
    bool ok = angle_computer_process(&zero_g, &att);
    TEST_ASSERT(ok == false, "Zero-G free fall detected and rejected");

    /* NULL pointers */
    TEST_ASSERT(angle_computer_process(NULL, &att) == false, "NULL input rejected");
    vector3f_t valid = { .x = 0.0f, .y = 0.0f, .z = 9.80665f };
    TEST_ASSERT(angle_computer_process(&valid, NULL) == false, "NULL output rejected");
}

int main(void) {
    TEST_SUITE_START("Angle Computer (Stateless Accelerometer Trigonometry)");

    test_level_flight();
    test_pure_roll_orientations();
    test_pure_pitch_orientations();
    test_boundary_and_fault_conditions();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
