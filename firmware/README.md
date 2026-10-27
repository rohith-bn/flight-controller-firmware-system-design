# Firmware

This directory will contain the QFC-1 embedded firmware implementation.

Planned layout:

- `include/`: public module headers.
- `src/`: hardware-independent firmware logic.
- `hal/`: HAL-facing adapter interfaces and platform integration points.

The implementation should keep control logic testable on a host machine wherever possible.

## First hardware bench test

The ready-to-upload ESP32-C3 MPU6050/MPU6500 gyro verification target is in
[`targets/esp32-c3-mpu6050`](targets/esp32-c3-mpu6050). It confirms IMU
identity and streams the three gyro axes to the serial console before any
control-loop work starts.
