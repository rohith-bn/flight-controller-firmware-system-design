/**
 * @file test_imu_converter.c
 * @brief Host Unit Tests for IMU Raw Count to Physical Unit Conversion
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "test_framework.h"
#include "imu_converter.h"
#include <string.h>

static void test_byte_decoding(void) {
    TEST_CASE("Big-Endian Byte Decoding to int16_t");

    /* Positive value: 0x1234 = 4660 */
    int16_t val1 = imu_converter_bytes_to_i16(0x12, 0x34);
    TEST_ASSERT_INT_EQUAL(val1, 4660, "Decode 0x12, 0x34 to +4660");

    /* Negative value: 0xFF, 0x9C = -100 */
    int16_t val2 = imu_converter_bytes_to_i16(0xFF, 0x9C);
    TEST_ASSERT_INT_EQUAL(val2, -100, "Decode 0xFF, 0x9C to -100");

    /* Zero: 0x00, 0x00 = 0 */
    int16_t val3 = imu_converter_bytes_to_i16(0x00, 0x00);
    TEST_ASSERT_INT_EQUAL(val3, 0, "Decode 0x00, 0x00 to 0");

    /* Positive maximum: 0x7F, 0xFF = 32767 */
    int16_t val4 = imu_converter_bytes_to_i16(0x7F, 0xFF);
    TEST_ASSERT_INT_EQUAL(val4, 32767, "Decode 0x7F, 0xFF to +32767 (INT16_MAX)");

    /* Negative minimum: 0x80, 0x00 = -32768 */
    int16_t val5 = imu_converter_bytes_to_i16(0x80, 0x00);
    TEST_ASSERT_INT_EQUAL(val5, -32768, "Decode 0x80, 0x00 to -32768 (INT16_MIN)");
}

static void test_accel_scaling(void) {
    TEST_CASE("Accelerometer Raw Counts to m/s^2");

    /* Zero counts -> 0.0 m/s^2 */
    float a0 = imu_converter_raw_to_mps2(0);
    TEST_ASSERT_FLOAT_NEAR(a0, 0.0f, 0.0001f, "0 counts -> 0.0 m/s^2");

    /* +1g (+16384 counts) -> +9.80665 m/s^2 */
    float a1 = imu_converter_raw_to_mps2(16384);
    TEST_ASSERT_FLOAT_NEAR(a1, 9.80665f, 0.001f, "+16384 counts -> +9.80665 m/s^2 (+1g)");

    /* -1g (-16384 counts) -> -9.80665 m/s^2 */
    float a2 = imu_converter_raw_to_mps2(-16384);
    TEST_ASSERT_FLOAT_NEAR(a2, -9.80665f, 0.001f, "-16384 counts -> -9.80665 m/s^2 (-1g)");

    /* +2g (+32767 counts) -> +19.6133 m/s^2 */
    float a3 = imu_converter_raw_to_mps2(32767);
    TEST_ASSERT_FLOAT_NEAR(a3, 19.6133f, 0.01f, "+32767 counts -> ~19.613 m/s^2 (Positive rail)");

    /* -2g (-32768 counts) -> -19.6139 m/s^2 */
    float a4 = imu_converter_raw_to_mps2(-32768);
    TEST_ASSERT_FLOAT_NEAR(a4, -19.6139f, 0.01f, "-32768 counts -> ~-19.614 m/s^2 (Negative rail)");
}

static void test_gyro_scaling(void) {
    TEST_CASE("Gyroscope Raw Counts to deg/s");

    /* Zero counts -> 0.0 deg/s */
    float g0 = imu_converter_raw_to_dps(0);
    TEST_ASSERT_FLOAT_NEAR(g0, 0.0f, 0.0001f, "0 counts -> 0.0 deg/s");

    /* +131 counts -> +1.0 deg/s */
    float g1 = imu_converter_raw_to_dps(131);
    TEST_ASSERT_FLOAT_NEAR(g1, 1.0f, 0.01f, "+131 counts -> +1.0 deg/s");

    /* -131 counts -> -1.0 deg/s */
    float g2 = imu_converter_raw_to_dps(-131);
    TEST_ASSERT_FLOAT_NEAR(g2, -1.0f, 0.01f, "-131 counts -> -1.0 deg/s");

    /* +1310 counts -> +10.0 deg/s */
    float g3 = imu_converter_raw_to_dps(1310);
    TEST_ASSERT_FLOAT_NEAR(g3, 10.0f, 0.01f, "+1310 counts -> +10.0 deg/s");

    /* Maximum positive range: +32750 counts -> ~250.0 deg/s */
    float g4 = imu_converter_raw_to_dps(32750);
    TEST_ASSERT_FLOAT_NEAR(g4, 250.0f, 0.1f, "+32750 counts -> ~250.0 deg/s");
}

static void test_temp_scaling(void) {
    TEST_CASE("Temperature Raw Counts to deg C");

    /* 0 counts -> 36.53 deg C */
    float t0 = imu_converter_raw_to_celsius(0);
    TEST_ASSERT_FLOAT_NEAR(t0, 36.53f, 0.01f, "0 counts -> 36.53 deg C");

    /* -340 counts -> 35.53 deg C */
    float t1 = imu_converter_raw_to_celsius(-340);
    TEST_ASSERT_FLOAT_NEAR(t1, 35.53f, 0.01f, "-340 counts -> 35.53 deg C");
}

static void test_frame_processing(void) {
    TEST_CASE("Full Frame Conversion Pipeline");

    imu_raw_data_t raw;
    raw.accel_raw.x = 0;
    raw.accel_raw.y = 0;
    raw.accel_raw.z = 16384; /* 1g down */
    raw.gyro_raw.x  = 131;   /* 1 deg/s roll */
    raw.gyro_raw.y  = -262;  /* -2 deg/s pitch */
    raw.gyro_raw.z  = 0;     /* 0 deg/s yaw */
    raw.temp_raw    = 0;

    imu_scaled_data_t scaled;
    bool ok = imu_converter_process_frame(&raw, &scaled);

    TEST_ASSERT(ok == true, "Frame conversion returns true on valid pointers");
    TEST_ASSERT_FLOAT_NEAR(scaled.accel_mps2.x, 0.0f, 0.001f, "Accel X is 0 m/s^2");
    TEST_ASSERT_FLOAT_NEAR(scaled.accel_mps2.y, 0.0f, 0.001f, "Accel Y is 0 m/s^2");
    TEST_ASSERT_FLOAT_NEAR(scaled.accel_mps2.z, 9.80665f, 0.001f, "Accel Z is 9.80665 m/s^2");

    TEST_ASSERT_FLOAT_NEAR(scaled.gyro_dps.x, 1.0f, 0.01f, "Gyro X is 1.0 deg/s");
    TEST_ASSERT_FLOAT_NEAR(scaled.gyro_dps.y, -2.0f, 0.01f, "Gyro Y is -2.0 deg/s");
    TEST_ASSERT_FLOAT_NEAR(scaled.gyro_dps.z, 0.0f, 0.01f, "Gyro Z is 0.0 deg/s");
    TEST_ASSERT_FLOAT_NEAR(scaled.temp_c, 36.53f, 0.01f, "Temp is 36.53 deg C");

    /* Defensive test: NULL pointer handling */
    TEST_ASSERT(imu_converter_process_frame(NULL, &scaled) == false, "NULL raw pointer returns false");
    TEST_ASSERT(imu_converter_process_frame(&raw, NULL) == false, "NULL scaled pointer returns false");
}

int main(void) {
    TEST_SUITE_START("IMU Converter (Stateless Unit Scaler)");

    test_byte_decoding();
    test_accel_scaling();
    test_gyro_scaling();
    test_temp_scaling();
    test_frame_processing();

    TEST_REPORT();
    return TEST_EXIT_CODE();
}
