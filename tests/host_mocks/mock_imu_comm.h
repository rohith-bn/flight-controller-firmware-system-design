/**
 * @file mock_imu_comm.h
 * @brief Host Mock & Test Double for MPU6050 I2C Hardware Seam
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 */

#ifndef MOCK_IMU_COMM_H
#define MOCK_IMU_COMM_H

#include "imu_comm.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void mock_imu_comm_reset(void);
void mock_imu_comm_set_whoami(uint8_t id);
void mock_imu_comm_set_bus_healthy(bool healthy);
void mock_imu_comm_set_canned_raw(int16_t ax, int16_t ay, int16_t az,
                                  int16_t gx, int16_t gy, int16_t gz,
                                  int16_t temp);
uint32_t mock_imu_comm_get_burst_read_count(void);
uint32_t mock_imu_comm_get_write_count(void);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_IMU_COMM_H */
