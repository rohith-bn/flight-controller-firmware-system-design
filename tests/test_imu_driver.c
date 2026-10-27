/**
 * @file test_imu_driver.c
 * @brief Host Unit Tests for Inertial Sensor Subsystem Coordinator
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "imu_driver.h"
#include "mock_imu_comm.h"

static void test_initialization_and_whoami(void) {
    TEST_CASE("IMU Initialization & WHO_AM_I Device Verification");

    mock_imu_comm_reset();
    imu_driver_t driver;

    /* 1. Nominal case: WHO_AM_I is 0x68 */
    mock_imu_comm_set_whoami(0x68);
    bool ok = imu_driver_init(&driver);
    TEST_ASSERT(ok == true, "IMU driver init succeeds when WHO_AM_I is 0x68");
    TEST_ASSERT(imu_driver_is_calibrated(&driver) == false, "Driver uncalibrated on fresh init");

    /* 2. Device mismatch: WHO_AM_I returns wrong ID 0x12 */
    mock_imu_comm_reset();
    mock_imu_comm_set_whoami(0x12);
    ok = imu_driver_init(&driver);
    TEST_ASSERT(ok == false, "IMU driver init rejected on WHO_AM_I mismatch");

    /* 3. Bus fault: I2C hardware error */
    mock_imu_comm_reset();
    mock_imu_comm_set_bus_healthy(false);
    ok = imu_driver_init(&driver);
    TEST_ASSERT(ok == false, "IMU driver init rejected on I2C bus failure");
}

static void test_calibration_and_bias_cancellation(void) {
    TEST_CASE("Boot Calibration & Real-Time Gyro Bias Cancellation");

    mock_imu_comm_reset();
    mock_imu_comm_set_whoami(0x68);

    imu_driver_t driver;
    imu_driver_init(&driver);

    /* Inject persistent zero-rate resting bias into mock:
     * Accel Z = 16384 (1g), Gyro X = +131 counts (+1.00 deg/s), Gyro Y = -262 counts (-2.00 deg/s) */
    mock_imu_comm_set_canned_raw(0, 0, 16384, 131, -262, 0, 0);

    /* Run 512 calibration steps */
    bool b_complete = false;
    for (uint16_t i = 0; i < QFC_GYRO_CALIBRATION_SAMPLES; i++) {
        bool step_ok = imu_driver_calibrate_step(&driver, &b_complete);
        TEST_ASSERT(step_ok == true, "Calibration step succeeds");
    }

    TEST_ASSERT(b_complete == true, "Calibration complete flag asserted at 512 samples");
    TEST_ASSERT(imu_driver_is_calibrated(&driver) == true, "Driver reports calibrated status");

    gyro_bias_t bias;
    bool ok = imu_driver_get_bias(&driver, &bias);
    TEST_ASSERT(ok == true, "Bias retrieved successfully");
    TEST_ASSERT_FLOAT_NEAR(bias.x_dps, 1.0f, 0.01f, "Stored X bias is +1.0 deg/s");
    TEST_ASSERT_FLOAT_NEAR(bias.y_dps, -2.0f, 0.01f, "Stored Y bias is -2.0 deg/s");

    /* Now test live reading with bias cancellation active:
     * Sensor is still resting at +131 counts (+1.0 deg/s).
     * Output should have the +1.0 deg/s bias subtracted, yielding 0.0 deg/s! */
    imu_scaled_data_t scaled;
    ok = imu_driver_read_calibrated(&driver, &scaled);
    TEST_ASSERT(ok == true, "Live calibrated read succeeds");
    TEST_ASSERT_FLOAT_NEAR(scaled.gyro_dps.x, 0.0f, 0.01f, "Gyro X bias cancelled to 0.0 deg/s");
    TEST_ASSERT_FLOAT_NEAR(scaled.gyro_dps.y, 0.0f, 0.01f, "Gyro Y bias cancelled to 0.0 deg/s");
    TEST_ASSERT_FLOAT_NEAR(scaled.accel_mps2.z, 9.80665f, 0.001f, "Accel Z untouched at 9.807 m/s^2");
}

static void test_bus_fault_during_reading(void) {
    TEST_CASE("Resilience to I2C Bus Fault During In-Flight Read");

    mock_imu_comm_reset();
    imu_driver_t driver;
    imu_driver_init(&driver);

    /* Simulate sudden I2C bus disconnection */
    mock_imu_comm_set_bus_healthy(false);

    imu_scaled_data_t scaled;
    bool ok = imu_driver_read_calibrated(&driver, &scaled);
    TEST_ASSERT(ok == false, "Read returns false upon I2C communication fault");
}

int main(void) {
    TEST_SUITE_START("IMU Driver (Inertial Sensor Coordinator & Calibrator)");

    test_initialization_and_whoami();
    test_calibration_and_bias_cancellation();
    test_bus_fault_during_reading();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
