#pragma once

#include <stdint.h>

// Hardware-independent representation of one gyroscope measurement.
struct GyroSampleDps {
  float x;
  float y;
  float z;
};

// MPU6050/MPU6500 gyroscope output at the +/-250 degrees/second range.
constexpr float kMpuGyroLsbPerDps = 131.0f;

inline GyroSampleDps gyroRawToDps(int16_t x, int16_t y, int16_t z) {
  return {
      static_cast<float>(x) / kMpuGyroLsbPerDps,
      static_cast<float>(y) / kMpuGyroLsbPerDps,
      static_cast<float>(z) / kMpuGyroLsbPerDps,
  };
}
