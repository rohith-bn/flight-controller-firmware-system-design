#include <Arduino.h>
#include <Wire.h>

#include "imu_device.h"

namespace {

// Change only these two pins when moving the same test to another ESP32-C3
// board. Most ESP32-C3 SuperMini boards use GPIO8 (SDA) and GPIO9 (SCL).
constexpr int kI2cSdaPin = 8;
constexpr int kI2cSclPin = 9;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint32_t kPrintPeriodMs =
    200; // 50 = 20 samples/second., 200 = 5 samples/second

constexpr uint8_t kMpuAddressLow = 0x68;
constexpr uint8_t kMpuAddressHigh = 0x69;
constexpr uint8_t kRegisterPowerManagement1 = 0x6B;
constexpr uint8_t kRegisterGyroConfig = 0x1B;
constexpr uint8_t kRegisterWhoAmI = 0x75;
constexpr uint8_t kRegisterGyroXoutHigh = 0x43;

class Mpu6x00I2c final : public IImuDevice {
public:
  bool begin() override {
    Wire.begin(kI2cSdaPin, kI2cSclPin);
    Wire.setClock(kI2cClockHz);

    if (!findDevice()) {
      error_ = "MPU not found at 0x68 or 0x69";
      return false;
    }

    uint8_t whoAmI = 0;
    if (!readRegister(kRegisterWhoAmI, whoAmI)) {
      error_ = "Could not read WHO_AM_I";
      return false;
    }

    // MPU6050 commonly reports 0x68; MPU6500 commonly reports 0x70.
    Serial.printf("MPU detected at 0x%02X, WHO_AM_I = 0x%02X\n", address_,
                  whoAmI);

    if (!writeRegister(kRegisterPowerManagement1, 0x80)) { // reset
      error_ = "Could not reset MPU";
      return false;
    }
    delay(100);
    if (!writeRegister(kRegisterPowerManagement1,
                       0x01)) { // wake, use gyro X clock
      error_ = "Could not wake MPU";
      return false;
    }
    if (!writeRegister(kRegisterGyroConfig, 0x00)) { // +/-250 degrees/second
      error_ = "Could not configure gyro range";
      return false;
    }
    return true;
  }

  bool readGyro(GyroSampleDps &sample, int16_t &rawX, int16_t &rawY,
                int16_t &rawZ) override {
    Wire.beginTransmission(address_);
    Wire.write(kRegisterGyroXoutHigh);
    if (Wire.endTransmission(false) != 0) {
      error_ = "I2C register selection failed";
      return false;
    }

    constexpr uint8_t kBytesToRead = 6;
    const uint8_t received = Wire.requestFrom(address_, kBytesToRead);
    if (received != kBytesToRead) {
      error_ = "Incomplete gyro read";
      while (Wire.available())
        Wire.read();
      return false;
    }

    const int16_t x = readInt16();
    const int16_t y = readInt16();
    const int16_t z = readInt16();
    sample = gyroRawToDps(x, y, z);
    rawX = x;
    rawY = y;
    rawZ = z;
    return true;
  }

  const char *lastError() const override { return error_; }

private:
  bool findDevice() {
    for (const uint8_t candidate : {kMpuAddressLow, kMpuAddressHigh}) {
      Wire.beginTransmission(candidate);
      if (Wire.endTransmission() == 0) {
        address_ = candidate;
        return true;
      }
    }
    return false;
  }

  bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  bool readRegister(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(address_, uint8_t{1}) != 1) {
      return false;
    }
    value = Wire.read();
    return true;
  }

  static int16_t readInt16() {
    const uint8_t high = Wire.read();
    const uint8_t low = Wire.read();
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  }

  uint8_t address_ = 0;
  const char *error_ = "Not initialized";
};

Mpu6x00I2c imu;
uint32_t lastPrintMs = 0;

} // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nQFC-1 ESP32-C3 MPU gyro verification");
  Serial.printf("I2C: SDA=GPIO%d, SCL=GPIO%d\n", kI2cSdaPin, kI2cSclPin);

  if (!imu.begin()) {
    Serial.printf("STARTUP FAILED: %s\n", imu.lastError());
    Serial.println("Check 3.3V, GND, SDA/SCL pins, and the AD0 address pin.");
    while (true)
      delay(1000);
  }

  Serial.println(
      "Gyro ready. Move or shake the sensor; X/Y/Z values should change.");
}

void loop() {
  if (millis() - lastPrintMs < kPrintPeriodMs)
    return;
  lastPrintMs = millis();

  GyroSampleDps gyro{};
  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;

  if (!imu.readGyro(gyro, rawX, rawY, rawZ)) {
    Serial.printf("READ ERROR: %s\n", imu.lastError());
    return;
  }
  Serial.printf("raw: X=%d  Y=%d  Z=%d | ", rawX, rawY, rawZ);
  // Serial.printf("raw: X=%d  Y=%d  Z=%d | ", x, y, z);
  Serial.printf("gyro_dps_R: X=%8.2f  Y=%8.2f  Z=%8.2f\n", gyro.x, gyro.y,
                gyro.z);
}
