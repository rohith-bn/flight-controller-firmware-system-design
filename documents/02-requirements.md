# Requirements

## Functional Requirements

### Startup And Calibration

- R-01: The firmware shall verify the MPU6050 identity during startup using the WHO_AM_I register.
- R-02: If IMU identity validation fails, the firmware shall enter FAULT.
- R-03: The firmware shall calculate gyroscope bias during startup while the board is stationary.
- R-04: After successful startup and calibration, the firmware shall enter DISARMED.

### Sensor Processing

- R-05: The firmware shall acquire raw accelerometer and gyroscope readings once per 1 ms control tick.
- R-06: The firmware shall convert accelerometer counts into physical units.
- R-07: The firmware shall convert gyroscope counts into physical units.
- R-08: The firmware shall subtract calibrated gyro bias from gyroscope readings.

### Attitude Estimation

- R-09: The firmware shall estimate roll angle.
- R-10: The firmware shall estimate pitch angle.
- R-11: The firmware shall use gyroscope data for short-term angle response.
- R-12: The firmware shall use accelerometer data for long-term drift correction.
- R-13: The initial estimation strategy shall be a complementary filter.

### Control

- R-14: The firmware shall maintain internal roll and pitch setpoints for bench validation.
- R-15: The firmware shall compute roll error from setpoint and estimated roll angle.
- R-16: The firmware shall compute pitch error from setpoint and estimated pitch angle.
- R-17: The firmware shall compute PID correction for the roll axis.
- R-18: The firmware shall compute PID correction for the pitch axis.
- R-19: The firmware shall apply integral windup limiting.

### Motor Mixing And Output

- R-20: The firmware shall mix roll and pitch corrections into four motor commands using Quad-X geometry.
- R-21: The firmware shall clamp every motor command before PWM output.
- R-22: The firmware shall write four PWM channels once per 1 ms tick while ARMED.
- R-23: The firmware shall force all motor outputs to zero while DISARMED.
- R-24: The firmware shall force all motor outputs to zero while FAULT.

### Safety State Machine

- R-25: The firmware shall support DISARMED, ARMED, and FAULT states.
- R-26: The firmware shall boot into a safe state and produce zero motor output on startup.
- R-27: The firmware shall transition from DISARMED to ARMED only through an explicit arm command.
- R-28: The firmware shall transition from ARMED to DISARMED through a disarm command.
- R-29: The firmware shall transition from any state to FAULT when a fault is detected.
- R-30: FAULT shall be one-way. The firmware shall not automatically recover from FAULT.

## Fault Requirements

- F-01: IMU identity mismatch shall trigger FAULT.
- F-02: I2C communication failure shall trigger FAULT.
- F-03: Sensor readings outside configured physical limits shall trigger FAULT.
- F-04: Estimated roll or pitch beyond safe bounds shall trigger FAULT.
- F-05: Motor command bounds violation before clamping shall trigger FAULT or be explicitly reported by the mixer contract.

## Timing Requirements

- T-01: The control pipeline shall be triggered by a 1 ms tick.
- T-02: The full control pipeline shall complete before the next 1 ms tick.
- T-03: No stage inside the 1 ms control pipeline shall use unbounded blocking.
- T-04: PID computation shall run in constant time.
- T-05: Motor mixing shall run in constant time.
- T-06: PWM output shall be bounded and deterministic.

## Design Constraints

- C-01: The firmware shall sit above the HAL boundary.
- C-02: The firmware shall not configure MCU clocks or peripheral registers directly.
- C-03: HAL services are assumed to be initialized before firmware entry.
- C-04: Runtime dynamic memory allocation shall not be used.
- C-05: The core firmware shall not depend on an RTOS.
- C-06: Hardware-independent modules shall be unit-testable on a host machine.

## Out Of Scope

- RC receiver input.
- Yaw control.
- Altitude estimation or hold.
- GPS or position control.
- Magnetometer or compass heading.
- Barometer integration.
- Kalman filter, EKF, Madgwick, or Mahony filters.
- UART logging or CLI.
- Wireless telemetry.
- Actual flight validation.

## Bench Validation Definition Of Done

- D-01: IMU identity validation passes on startup.
- D-02: Gyroscope bias calibration completes.
- D-03: Roll and pitch estimates respond correctly when the board is tilted by hand.
- D-04: PID outputs respond to angle error.
- D-05: Four PWM channels update once per 1 ms tick while ARMED.
- D-06: DISARMED produces zero motor output.
- D-07: FAULT produces zero motor output.
- D-08: A simulated fault transitions the system to FAULT and holds it there until reset.
