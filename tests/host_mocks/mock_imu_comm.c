/**
 * @file mock_imu_comm.c
 * @brief Host Mock & Test Double for MPU6050 I2C Hardware Seam Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#include "mock_imu_comm.h"
#include <string.h>

static uint8_t  s_mock_whoami = MPU6050_WHO_AM_I_DEFAULT_VAL;
static bool     s_mock_bus_healthy = true;
static uint8_t  s_canned_burst_buffer[MPU6050_BURST_DATA_LEN] = {0};
static uint32_t s_burst_read_count = 0;
static uint32_t s_write_count = 0;

void mock_imu_comm_reset(void) {
    s_mock_whoami = MPU6050_WHO_AM_I_DEFAULT_VAL;
    s_mock_bus_healthy = true;
    (void)memset(s_canned_burst_buffer, 0, sizeof(s_canned_burst_buffer));
    s_burst_read_count = 0;
    s_write_count = 0;
}

void mock_imu_comm_set_whoami(uint8_t id) {
    s_mock_whoami = id;
}

void mock_imu_comm_set_bus_healthy(bool healthy) {
    s_mock_bus_healthy = healthy;
}

static void pack_be16(uint8_t *p_dest, int16_t val) {
    p_dest[0] = (uint8_t)(((uint16_t)val >> 8) & 0xFFU);
    p_dest[1] = (uint8_t)((uint16_t)val & 0xFFU);
}

void mock_imu_comm_set_canned_raw(int16_t ax, int16_t ay, int16_t az,
                                  int16_t gx, int16_t gy, int16_t gz,
                                  int16_t temp) {
    /* 14-byte format:
     * 0-1:   ACCEL_XOUT
     * 2-3:   ACCEL_YOUT
     * 4-5:   ACCEL_ZOUT
     * 6-7:   TEMP_OUT
     * 8-9:   GYRO_XOUT
     * 10-11: GYRO_YOUT
     * 12-13: GYRO_ZOUT
     */
    pack_be16(&s_canned_burst_buffer[0], ax);
    pack_be16(&s_canned_burst_buffer[2], ay);
    pack_be16(&s_canned_burst_buffer[4], az);
    pack_be16(&s_canned_burst_buffer[6], temp);
    pack_be16(&s_canned_burst_buffer[8], gx);
    pack_be16(&s_canned_burst_buffer[10], gy);
    pack_be16(&s_canned_burst_buffer[12], gz);
}

uint32_t mock_imu_comm_get_burst_read_count(void) {
    return s_burst_read_count;
}

uint32_t mock_imu_comm_get_write_count(void) {
    return s_write_count;
}

/* ========================================================================== */
/* HARDWARE INTERFACE IMPLEMENTATION (LINKED BY FIRMWARE)                     */
/* ========================================================================== */

bool imu_comm_init(void) {
    return s_mock_bus_healthy;
}

bool imu_comm_check_whoami(uint8_t expected_id) {
    if (!s_mock_bus_healthy) {
        return false;
    }
    return (s_mock_whoami == expected_id);
}

bool imu_comm_read_burst(uint8_t start_reg, uint8_t *p_buf, uint8_t len) {
    (void)start_reg;
    if (!s_mock_bus_healthy || (p_buf == NULL) || (len > MPU6050_BURST_DATA_LEN)) {
        return false;
    }
    (void)memcpy(p_buf, s_canned_burst_buffer, len);
    s_burst_read_count++;
    return true;
}

bool imu_comm_write_reg(uint8_t reg, uint8_t val) {
    (void)reg;
    (void)val;
    if (!s_mock_bus_healthy) {
        return false;
    }
    s_write_count++;
    return true;
}
