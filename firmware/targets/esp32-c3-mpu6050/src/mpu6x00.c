#include "mpu6x00.h"

#define MPU_ADDRESS_LOW 0x68
#define MPU_ADDRESS_HIGH 0x69

#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_GYRO_XOUT_H 0x43

#define GYRO_LSB_PER_DPS 131.0f

static int16_t bytes_to_int16(uint8_t high, uint8_t low) {
  return (int16_t)(((uint16_t)high << 8) | low);
}