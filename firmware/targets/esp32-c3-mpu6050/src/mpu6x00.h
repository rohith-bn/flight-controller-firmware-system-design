#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float x_dps;
  float y_dps;
  float z_dps;
} GyroSampleDps;

typedef struct {
  void *context;

  bool (*write_register)(void *context, uint8_t device_address,
                         uint8_t register_address, uint8_t value);

  bool (*read_registers)(void *context, uint8_t device_address,
                         uint8_t start_register, uint8_t *buffer,
                         uint8_t length);
} I2cBus;

typedef struct {
  I2cBus bus;
  uint8_t address;
} Mpu6x00;

bool mpu6x00_begin(Mpu6x00 *imu);
bool mpu6x00_read_gyro(Mpu6x00 *imu, GyroSampleDps *sample);