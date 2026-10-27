/**
 * @file test_gyro_calibrator.c
 * @brief Host Unit Tests for Startup Zero-Rate Gyroscope Bias Characterization
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "gyro_calibrator.h"

static void test_initialization_and_empty_state(void) {
    TEST_CASE("Calibrator Initialization & State Checking");

    gyro_calibrator_t cal;
    bool ok = gyro_calibrator_init(&cal);
    TEST_ASSERT(ok == true, "Initialization succeeds");
    TEST_ASSERT(gyro_calibrator_is_complete(&cal) == false, "New calibrator is not complete");

    gyro_bias_t bias;
    TEST_ASSERT(gyro_calibrator_get_bias(&cal, &bias) == false, "Get bias fails before 512 samples");

    /* Defensive: NULL handling */
    TEST_ASSERT(gyro_calibrator_init(NULL) == false, "NULL init rejected");
    TEST_ASSERT(gyro_calibrator_is_complete(NULL) == false, "NULL is_complete rejected");
}

static void test_512_sample_bias_extraction(void) {
    TEST_CASE("512-Sample Accumulation and Accurate Bias Extraction");

    gyro_calibrator_t cal;
    gyro_calibrator_init(&cal);

    /* Synthetic ground truth bias: X = +1.50 deg/s, Y = -2.25 deg/s, Z = +0.80 deg/s */
    /* Add small alternating noise (-0.1, +0.1) that sums to zero over 512 samples */
    for (uint16_t i = 0; i < QFC_GYRO_CALIBRATION_SAMPLES; i++) {
        float noise = (i % 2 == 0) ? +0.05f : -0.05f;
        vector3f_t sample = {
            .x = 1.50f + noise,
            .y = -2.25f + noise,
            .z = 0.80f + noise
        };

        bool ok = gyro_calibrator_push_sample(&cal, &sample);
        TEST_ASSERT(ok == true, "Pushing sample within motion threshold succeeds");

        if (i < (QFC_GYRO_CALIBRATION_SAMPLES - 1)) {
            TEST_ASSERT(gyro_calibrator_is_complete(&cal) == false, "Incomplete before sample 512");
        }
    }

    TEST_ASSERT(gyro_calibrator_is_complete(&cal) == true, "Calibrator is complete at sample 512");

    gyro_bias_t bias;
    bool ok = gyro_calibrator_get_bias(&cal, &bias);
    TEST_ASSERT(ok == true, "Bias retrieved successfully once complete");
    TEST_ASSERT_FLOAT_NEAR(bias.x_dps, 1.50f, 0.001f, "Extracted X bias is +1.50 deg/s");
    TEST_ASSERT_FLOAT_NEAR(bias.y_dps, -2.25f, 0.001f, "Extracted Y bias is -2.25 deg/s");
    TEST_ASSERT_FLOAT_NEAR(bias.z_dps, 0.80f, 0.001f, "Extracted Z bias is +0.80 deg/s");
}

static void test_excessive_motion_fault_detection(void) {
    TEST_CASE("Excessive Motion Fault Detection (> 10.0 deg/s)");

    gyro_calibrator_t cal;
    gyro_calibrator_init(&cal);

    /* Push 10 stationary samples */
    vector3f_t stationary = { .x = 0.2f, .y = -0.1f, .z = 0.05f };
    for (int i = 0; i < 10; i++) {
        gyro_calibrator_push_sample(&cal, &stationary);
    }

    /* Push sample with excessive motion (e.g. drone moved or bumped: X = 15.0 deg/s > 10.0) */
    vector3f_t bump = { .x = 15.0f, .y = 0.0f, .z = 0.0f };
    bool ok = gyro_calibrator_push_sample(&cal, &bump);
    TEST_ASSERT(ok == false, "Motion > 10.0 deg/s triggers fault and returns false");
    TEST_ASSERT(gyro_calibrator_is_complete(&cal) == false, "Calibrator marked incomplete on motion fault");

    /* Subsequent samples must be rejected while faulted */
    ok = gyro_calibrator_push_sample(&cal, &stationary);
    TEST_ASSERT(ok == false, "Subsequent samples rejected after motion fault");

    /* Reset clears fault */
    ok = gyro_calibrator_reset(&cal);
    TEST_ASSERT(ok == true, "Reset succeeds");
    ok = gyro_calibrator_push_sample(&cal, &stationary);
    TEST_ASSERT(ok == true, "Samples accepted again after reset");
}

int main(void) {
    TEST_SUITE_START("Gyro Calibrator (Startup Zero-Rate Bias Characterization)");

    test_initialization_and_empty_state();
    test_512_sample_bias_extraction();
    test_excessive_motion_fault_detection();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
