/**
 * @file imu_driver.c
 * @brief Inertial Sensor Subsystem Coordinator Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "imu_driver.h"
#include <stddef.h>

bool imu_driver_init(imu_driver_t *p_driver) {
    if (p_driver == NULL) {
        return false;
    }

    p_driver->b_initialized = false;
    p_driver->b_calibrated  = false;

    /* 1. Initialize physical or mock I2C bus peripheral */
    if (!imu_comm_init()) {
        return false;
    }

    /* 2. Device verification: check WHO_AM_I register (0x68) */
    if (!imu_comm_check_whoami(MPU6050_WHO_AM_I_DEFAULT_VAL)) {
        return false;
    }

    /* 3. Wake up sensor from sleep mode by writing 0x00 to PWR_MGMT_1 (0x6B) */
    if (!imu_comm_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00U)) {
        return false;
    }

    /* 4. Configure sample rate divider */
    if (!imu_comm_write_reg(MPU6050_REG_SMPLRT_DIV, 0x00U)) {
        return false;
    }

    /* 5. Initialize the zero-rate gyro calibrator */
    if (!gyro_calibrator_init(&p_driver->calibrator)) {
        return false;
    }

    p_driver->stored_bias.x_dps = 0.0f;
    p_driver->stored_bias.y_dps = 0.0f;
    p_driver->stored_bias.z_dps = 0.0f;

    p_driver->b_initialized = true;
    return true;
}

bool imu_driver_calibrate_step(imu_driver_t *p_driver, bool *p_b_complete) {
    if ((p_driver == NULL) || (p_b_complete == NULL) || (!p_driver->b_initialized)) {
        return false;
    }

    uint8_t buf[MPU6050_BURST_DATA_LEN];
    if (!imu_comm_read_burst(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_BURST_DATA_LEN)) {
        return false;
    }

    /* Extract big-endian raw gyro counts (bytes 8-13) */
    int16_t gx_raw = imu_converter_bytes_to_i16(buf[8], buf[9]);
    int16_t gy_raw = imu_converter_bytes_to_i16(buf[10], buf[11]);
    int16_t gz_raw = imu_converter_bytes_to_i16(buf[12], buf[13]);

    /* Convert to physical deg/s */
    vector3f_t sample = {
        .x = imu_converter_raw_to_dps(gx_raw),
        .y = imu_converter_raw_to_dps(gy_raw),
        .z = imu_converter_raw_to_dps(gz_raw)
    };

    /* Accumulate into calibrator */
    if (!gyro_calibrator_push_sample(&p_driver->calibrator, &sample)) {
        return false;
    }

    if (gyro_calibrator_is_complete(&p_driver->calibrator)) {
        (void)gyro_calibrator_get_bias(&p_driver->calibrator, &p_driver->stored_bias);
        p_driver->b_calibrated = true;
        *p_b_complete = true;
    } else {
        *p_b_complete = false;
    }

    return true;
}

bool imu_driver_read_calibrated(imu_driver_t *p_driver, imu_scaled_data_t *p_scaled) {
    if ((p_driver == NULL) || (p_scaled == NULL) || (!p_driver->b_initialized)) {
        return false;
    }

    uint8_t buf[MPU6050_BURST_DATA_LEN];
    if (!imu_comm_read_burst(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_BURST_DATA_LEN)) {
        return false;
    }

    /* Reconstruct raw measurement frame */
    imu_raw_data_t raw;
    raw.accel_raw.x = imu_converter_bytes_to_i16(buf[0], buf[1]);
    raw.accel_raw.y = imu_converter_bytes_to_i16(buf[2], buf[3]);
    raw.accel_raw.z = imu_converter_bytes_to_i16(buf[4], buf[5]);
    raw.temp_raw    = imu_converter_bytes_to_i16(buf[6], buf[7]);
    raw.gyro_raw.x  = imu_converter_bytes_to_i16(buf[8], buf[9]);
    raw.gyro_raw.y  = imu_converter_bytes_to_i16(buf[10], buf[11]);
    raw.gyro_raw.z  = imu_converter_bytes_to_i16(buf[12], buf[13]);

    /* Scale to engineering units */
    if (!imu_converter_process_frame(&raw, p_scaled)) {
        return false;
    }

    /* Real-time zero-rate bias cancellation */
    p_scaled->gyro_dps.x -= p_driver->stored_bias.x_dps;
    p_scaled->gyro_dps.y -= p_driver->stored_bias.y_dps;
    p_scaled->gyro_dps.z -= p_driver->stored_bias.z_dps;

    p_driver->last_sample = *p_scaled;
    return true;
}

bool imu_driver_get_bias(const imu_driver_t *p_driver, gyro_bias_t *p_bias) {
    if ((p_driver == NULL) || (p_bias == NULL) || (!p_driver->b_calibrated)) {
        return false;
    }

    *p_bias = p_driver->stored_bias;
    return true;
}

bool imu_driver_is_calibrated(const imu_driver_t *p_driver) {
    if (p_driver == NULL) {
        return false;
    }
    return p_driver->b_calibrated;
}
