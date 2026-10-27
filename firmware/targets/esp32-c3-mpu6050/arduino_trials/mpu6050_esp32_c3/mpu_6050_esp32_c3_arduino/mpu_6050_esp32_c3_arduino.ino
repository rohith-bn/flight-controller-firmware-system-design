#include <Wire.h>

const int SDA_PIN = 8;
const int SCL_PIN = 9;

uint8_t mpuAddress = 0;

// MPU6050 / MPU6500 registers
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_XOUT_H = 0x43;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool findMpu() {
  const uint8_t addresses[] = {0x68, 0x69};

  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(addresses[i]);

    if (Wire.endTransmission() == 0) {
      mpuAddress = addresses[i];
      return true;
    }
  }

  return false;
}

int16_t readInt16() {
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  return (int16_t)((highByte << 8) | lowByte);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nESP32-C3 MPU gyro test");

  // ESP32-C3 I2C pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!findMpu()) {
    Serial.println("MPU not found.");
    Serial.println("Check VCC, GND, SDA, SCL wiring.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("MPU found at I2C address: 0x");
  Serial.println(mpuAddress, HEX);

  // Reset, then wake up the sensor
  writeRegister(PWR_MGMT_1, 0x80);
  delay(100);

  writeRegister(PWR_MGMT_1, 0x01);
  writeRegister(GYRO_CONFIG, 0x00);  // gyro range: +/-250 degrees/second

  Serial.println("Gyro ready. Move or shake the MPU.");
}

void loop() {
  // Point the MPU internal register cursor to gyro X high byte
  Wire.beginTransmission(mpuAddress);
  Wire.write(GYRO_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    Serial.println("I2C error.");
    delay(500);
    return;
  }

  // Read X, Y, Z: each axis uses two bytes
  if (Wire.requestFrom(mpuAddress, (uint8_t)6) != 6) {
    Serial.println("Gyro read failed.");
    delay(500);
    return;
  }

  int16_t rawX = readInt16();
  int16_t rawY = readInt16();
  int16_t rawZ = readInt16();

  // At +/-250 dps, MPU6050 uses 131 raw counts per degree/second
  float gyroX = rawX / 131.0;
  float gyroY = rawY / 131.0;
  float gyroZ = rawZ / 131.0;

  Serial.print("X: ");
  Serial.print(gyroX, 2);
  Serial.print(" dps | Y: ");
  Serial.print(gyroY, 2);
  Serial.print(" dps | Z: ");
  Serial.print(gyroZ, 2);
  Serial.println(" dps");

  delay(100);
}
