/**
 * @file test_pid_controller.c
 * @brief Host Unit Tests for Closed-Loop Attitude Stabilization Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "pid_controller.h"

static void test_initialization_and_reset(void) {
    TEST_CASE("PID Controller Initialization & Configuration");

    pid_controller_t ctrl;
    pid_axis_config_t roll_cfg = {
        .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .windup_limit = 50.0f, .dt_s = 0.001f
    };
    pid_axis_config_t pitch_cfg = {
        .kp = 2.0f, .ki = 0.0f, .kd = 0.0f, .windup_limit = 50.0f, .dt_s = 0.001f
    };

    bool ok = pid_controller_init(&ctrl, &roll_cfg, &pitch_cfg);
    TEST_ASSERT(ok == true, "PID controller initializes successfully");

    ok = pid_controller_reset(&ctrl);
    TEST_ASSERT(ok == true, "Reset succeeds");

    /* Defensive: NULL pointers */
    TEST_ASSERT(pid_controller_init(NULL, &roll_cfg, &pitch_cfg) == false, "NULL ctrl rejected");
    TEST_ASSERT(pid_controller_init(&ctrl, NULL, &pitch_cfg) == false, "NULL roll_cfg rejected");
}

static void test_dual_axis_demands(void) {
    TEST_CASE("Dual-Axis Proportional Demands Computation");

    pid_controller_t ctrl;
    /* Roll Kp = 1.0, Pitch Kp = 2.0 */
    pid_axis_config_t roll_cfg = {
        .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .windup_limit = 50.0f, .dt_s = 0.001f
    };
    pid_axis_config_t pitch_cfg = {
        .kp = 2.0f, .ki = 0.0f, .kd = 0.0f, .windup_limit = 50.0f, .dt_s = 0.001f
    };
    pid_controller_init(&ctrl, &roll_cfg, &pitch_cfg);

    setpoint_deg_t target = { .roll_deg = 0.0f, .pitch_deg = 0.0f };
    /* Actual attitude: Roll tilted +5.0 deg (needs negative correction), Pitch tilted -10.0 deg (needs positive correction) */
    attitude_deg_t actual = { .roll_deg = 5.0f, .pitch_deg = -10.0f };

    pid_demands_t demands;
    bool ok = pid_controller_update(&ctrl, &target, &actual, &demands);

    TEST_ASSERT(ok == true, "Update computes demands successfully");
    /* Roll demand = Kp * (0 - 5.0) = 1.0 * -5.0 = -5.0 */
    TEST_ASSERT_FLOAT_NEAR(demands.roll_demand, -5.0f, 0.001f, "Roll demand is -5.0");
    /* Pitch demand = Kp * (0 - (-10.0)) = 2.0 * +10.0 = +20.0 */
    TEST_ASSERT_FLOAT_NEAR(demands.pitch_demand, 20.0f, 0.001f, "Pitch demand is +20.0");
}

static void test_axis_decoupling(void) {
    TEST_CASE("Axis Decoupling: Pure Pitch Disturbance leaves Roll at Zero");

    pid_controller_t ctrl;
    pid_axis_config_t cfg = {
        .kp = 1.5f, .ki = 0.0f, .kd = 0.0f, .windup_limit = 50.0f, .dt_s = 0.001f
    };
    pid_controller_init(&ctrl, &cfg, &cfg);

    setpoint_deg_t target = { .roll_deg = 0.0f, .pitch_deg = 0.0f };
    attitude_deg_t actual = { .roll_deg = 0.0f, .pitch_deg = 8.0f };

    pid_demands_t demands;
    pid_controller_update(&ctrl, &target, &actual, &demands);

    TEST_ASSERT_FLOAT_NEAR(demands.roll_demand, 0.0f, 0.0001f, "Roll demand remains exactly 0.0");
    TEST_ASSERT_FLOAT_NEAR(demands.pitch_demand, -12.0f, 0.001f, "Pitch demand responds with -12.0");
}

int main(void) {
    TEST_SUITE_START("PID Controller (Dual-Axis Attitude Coordinator)");

    test_initialization_and_reset();
    test_dual_axis_demands();
    test_axis_decoupling();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
