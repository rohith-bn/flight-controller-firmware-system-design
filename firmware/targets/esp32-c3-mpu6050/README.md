# ESP32-C3 MPU6050/MPU6500 gyro verification

This is the first QFC-1 hardware bench test. It verifies only that the IMU is detected and its three gyro axes change when the sensor moves.

## Wiring

Use 3.3 V logic and power.

| MPU module | ESP32-C3 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |

The selected pins suit most ESP32-C3 SuperMini boards. If your board labels different I2C pins, edit `kI2cSdaPin` and `kI2cSclPin` at the top of `src/main.cpp`.

## Run in VS Code

1. Install the **PlatformIO IDE** extension in VS Code.
2. Open the `firmware/targets/esp32-c3-mpu6050` folder.
3. Connect the ESP32-C3 by USB, then select **PlatformIO: Upload**.
4. Select **PlatformIO: Monitor** at 115200 baud.

Expected startup output:

```text
QFC-1 ESP32-C3 MPU gyro verification
I2C: SDA=GPIO8, SCL=GPIO9
MPU detected at 0x68, WHO_AM_I = 0x68
Gyro ready. Move or shake the sensor; X/Y/Z values should change.
gyro_dps: X=   -0.31  Y=    0.06  Z=    0.18
```

When you shake or rotate the board, the `X`, `Y`, and `Z` degrees-per-second readings should vary noticeably. At rest, small non-zero values are normal; gyro bias calibration comes later.

## Architecture

`src/main.cpp` is the ESP32-C3/I2C adapter. Its only output to the rest of the future flight controller is the `IImuDevice` interface and `GyroSampleDps` type in `firmware/include/`, so the sensing/control code can later be kept independent of the ESP32-C3 and replaced with another board implementation.
