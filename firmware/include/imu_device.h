#pragma once

#include "imu_types.h"

// The flight-control logic will depend on this interface, never on Wire/I2C.
class IImuDevice {
public:
  virtual ~IImuDevice() = default;
  virtual bool begin() = 0;
  // virtual bool readGyro(GyroSampleDps& sample) = 0;
  virtual bool readGyro(GyroSampleDps &sample, int16_t &rawX, int16_t &rawY,
                        int16_t &rawZ) = 0;
  virtual const char *lastError() const = 0;
};
