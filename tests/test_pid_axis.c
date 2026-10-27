/**
 * @file test_pid_axis.c
 * @brief Host Unit Tests for Discrete PID Axis Controller with Anti-Windup
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "pid_axis.h"

static void test_initialization(void) {
    TEST_CASE("PID Axis Initialization & Gain Validation");

    pid_axis_t pid;
    pid_axis_config_t cfg = {
        .kp = 1.5f,
        .ki = 0.05f,
        .kd = 0.2f,
        .windup_limit = 100.0f,
        .dt_s = 0.001f
    };

    bool ok = pid_axis_init(&pid, &cfg);
    TEST_ASSERT(ok == true, "PID initializes with valid config");

    /* Defensive: negative gains or non-positive dt */
    cfg.kp = -1.0f;
    TEST_ASSERT(pid_axis_init(&pid, &cfg) == false, "Negative Kp rejected");
    cfg.kp = 1.5f; cfg.ki = -0.1f;
    TEST_ASSERT(pid_axis_init(&pid, &cfg) == false, "Negative Ki rejected");
    cfg.ki = 0.05f; cfg.dt_s = 0.0f;
    TEST_ASSERT(pid_axis_init(&pid, &cfg) == false, "Zero dt rejected");
    TEST_ASSERT(pid_axis_init(NULL, &cfg) == false, "NULL instance rejected");
}

static void test_proportional_response(void) {
    TEST_CASE("Pure Proportional Response (Kp = 2.5, Ki = 0, Kd = 0)");

    pid_axis_t pid;
    pid_axis_config_t cfg = {
        .kp = 2.5f,
        .ki = 0.0f,
        .kd = 0.0f,
        .windup_limit = 50.0f,
        .dt_s = 0.001f
    };
    pid_axis_init(&pid, &cfg);

    /* Target = 10.0, Actual = 2.0 -> Error = 8.0 -> Demand = 2.5 * 8.0 = 20.0 */
    float out = pid_axis_update(&pid, 10.0f, 2.0f);
    TEST_ASSERT_FLOAT_NEAR(out, 20.0f, 0.001f, "Proportional output = Kp * error (20.0)");

    /* Negative error: Target = -5.0, Actual = 0.0 -> Error = -5.0 -> Demand = -12.5 */
    out = pid_axis_update(&pid, -5.0f, 0.0f);
    TEST_ASSERT_FLOAT_NEAR(out, -12.5f, 0.001f, "Negative error gives negative demand (-12.5)");
}

static void test_integral_accumulation_and_anti_windup(void) {
    TEST_CASE("Integral Accumulation & Anti-Windup Clamping");

    pid_axis_t pid;
    pid_axis_config_t cfg = {
        .kp = 0.0f,
        .ki = 50.0f,          /* 50 per second */
        .kd = 0.0f,
        .windup_limit = 25.0f, /* Clamped at +/- 25.0 */
        .dt_s = 0.001f
    };
    pid_axis_init(&pid, &cfg);

    /* Apply constant error = 2.0 deg for 100 ticks (0.100 s) */
    /* Integrated error = 2.0 * 0.100 = 0.200 deg*s */
    /* Integral contribution = Ki * sum(e*dt) = 50.0 * 0.200 = 10.0 */
    float out = 0.0f;
    for (int i = 0; i < 100; i++) {
        out = pid_axis_update(&pid, 2.0f, 0.0f);
    }
    TEST_ASSERT_FLOAT_NEAR(out, 10.0f, 0.01f, "Integral accumulates to 10.0 over 100 ms");

    /* Continue running for 1000 more ticks: unconstrained would reach ~110.0 */
    for (int i = 0; i < 1000; i++) {
        out = pid_axis_update(&pid, 2.0f, 0.0f);
    }
    TEST_ASSERT_FLOAT_NEAR(out, 25.0f, 0.001f, "Anti-windup clamps output at exactly +25.0");

    /* Reverse error to -2.0 deg: output must immediately begin unwinding from 25.0 */
    out = pid_axis_update(&pid, -2.0f, 0.0f);
    TEST_ASSERT(out < 25.0f, "Integrator immediately unwinds upon sign reversal (no lag)");
}

static void test_derivative_action_and_first_run(void) {
    TEST_CASE("Derivative Action and First-Run Spike Protection");

    pid_axis_t pid;
    pid_axis_config_t cfg = {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.1f,           /* Kd = 0.1 */
        .windup_limit = 50.0f,
        .dt_s = 0.001f
    };
    pid_axis_init(&pid, &cfg);

    /* First run: Target = 10.0, Actual = 0.0 -> Error = 10.0 */
    /* Must NOT produce a huge derivative spike from 0 -> 10.0! */
    float out1 = pid_axis_update(&pid, 10.0f, 0.0f);
    TEST_ASSERT_FLOAT_NEAR(out1, 0.0f, 0.001f, "First run derivative spike is suppressed (D = 0)");

    /* Second run: Target = 10.0, Actual = 1.0 -> Error = 9.0 */
    /* Delta error = 9.0 - 10.0 = -1.0 */
    /* D = Kd * (de/dt) = 0.1 * (-1.0 / 0.001) = -100.0 */
    float out2 = pid_axis_update(&pid, 10.0f, 1.0f);
    TEST_ASSERT_FLOAT_NEAR(out2, -100.0f, 0.1f, "Derivative acts as damping against rate of change (-100.0)");
}

static void test_reset_behavior(void) {
    TEST_CASE("PID State Reset");

    pid_axis_t pid;
    pid_axis_config_t cfg = {
        .kp = 1.0f,
        .ki = 10.0f,
        .kd = 0.5f,
        .windup_limit = 50.0f,
        .dt_s = 0.001f
    };
    pid_axis_init(&pid, &cfg);

    /* Pump errors into PID */
    for (int i = 0; i < 50; i++) {
        pid_axis_update(&pid, 5.0f, 0.0f);
    }

    /* Reset PID state */
    bool ok = pid_axis_reset(&pid);
    TEST_ASSERT(ok == true, "Reset returns true");

    float p_term, i_term, d_term;
    pid_axis_get_terms(&pid, &p_term, &i_term, &d_term);
    TEST_ASSERT_FLOAT_NEAR(i_term, 0.0f, 0.0001f, "Integral accumulator is 0.0 after reset");
}

int main(void) {
    TEST_SUITE_START("PID Axis (Discrete Strategy Instance with Anti-Windup)");

    test_initialization();
    test_proportional_response();
    test_integral_accumulation_and_anti_windup();
    test_derivative_action_and_first_run();
    test_reset_behavior();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
