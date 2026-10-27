/**
 * @file imu_converter.c
 * @brief Stateless Raw Count to Physical Engineering Unit Scaler Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "imu_converter.h"
#include <stddef.h>

int16_t imu_converter_bytes_to_i16(uint8_t msb, uint8_t lsb) {
    /* Reconstruct 16-bit signed integer using bitwise operations */
    uint16_t u16 = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return (int16_t)u16;
}

float imu_converter_raw_to_mps2(int16_t raw_counts) {
    /* a = (raw_counts / 16384.0) * 9.80665 m/s^2 */
    return ((float)raw_counts / QFC_MPU6050_ACCEL_LSB_PER_G) * QFC_GRAVITY_MSS;
}

float imu_converter_raw_to_dps(int16_t raw_counts) {
    /* omega = raw_counts / 131.0 deg/s */
    return (float)raw_counts / QFC_MPU6050_GYRO_LSB_PER_DPS;
}

float imu_converter_raw_to_celsius(int16_t raw_counts) {
    /* T = (raw_counts / 340.0) + 36.53 deg C */
    return ((float)raw_counts / QFC_MPU6050_TEMP_LSB_PER_C) + QFC_MPU6050_TEMP_OFFSET_LSB;
}

bool imu_converter_process_frame(const imu_raw_data_t *p_raw, imu_scaled_data_t *p_scaled) {
    if ((p_raw == NULL) || (p_scaled == NULL)) {
        return false;
    }

    /* Convert Accelerometer 3-Axis counts to m/s^2 */
    p_scaled->accel_mps2.x = imu_converter_raw_to_mps2(p_raw->accel_raw.x);
    p_scaled->accel_mps2.y = imu_converter_raw_to_mps2(p_raw->accel_raw.y);
    p_scaled->accel_mps2.z = imu_converter_raw_to_mps2(p_raw->accel_raw.z);

    /* Convert Gyroscope 3-Axis counts to deg/s */
    p_scaled->gyro_dps.x = imu_converter_raw_to_dps(p_raw->gyro_raw.x);
    p_scaled->gyro_dps.y = imu_converter_raw_to_dps(p_raw->gyro_raw.y);
    p_scaled->gyro_dps.z = imu_converter_raw_to_dps(p_raw->gyro_raw.z);

    /* Convert Temperature counts to Celsius */
    p_scaled->temp_c = imu_converter_raw_to_celsius(p_raw->temp_raw);

    return true;
}
