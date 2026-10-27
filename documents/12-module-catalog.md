# Module Catalog

## Purpose Of This Document

This document provides the definitive module catalog for QFC-1. It defines the implementation-level boundaries, public API contracts, internal state structures, design patterns, and collaborator mappings for all 15 firmware modules.

Each module corresponds to exactly one `.c` implementation file and one `.h` header file in the `firmware/` directory.

## Why This Follows The Runtime View

The previous document, [Runtime View](11-runtime-view.md), defined how code executes across time (the 1 ms super loop) and how behavioral design patterns structure system interactions. 

The Module Catalog translates that behavioral and runtime model into a concrete, static code architecture. It establishes strict file boundaries and dependency rules before writing implementation code.

---

## Master Module Inventory Table

| # | Module Name | Design Pattern Applied | Memory Model | HAL Dependency | Header / Source File |
| :---: | :--- | :--- | :--- | :---: | :--- |
| **01** | `flight_controller` | Object Pattern (Orchestrator) | Static volatile flag | Timer Tick | `flight_controller.h` / `.c` |
| **02** | `system_state_manager` | Opaque ADT + State Pattern | Static struct | None | `system_state_manager.h` / `.c` |
| **03** | `imu_driver` | Object Pattern + Containment | Static buffer struct | Via `imu_comm` | `imu_driver.h` / `.c` |
| **04** | `imu_comm` | Virtual API Pattern | Stateless executor | I2C API | `imu_comm.h` / `.c` |
| **05** | `imu_converter` | Object Pattern (Stateless Utility)| Pure math (zero state) | None | `imu_converter.h` / `.c` |
| **06** | `gyro_calibrator` | Object Pattern | Static sample buffer | None | `gyro_calibrator.h` / `.c` |
| **07** | `attitude_estimator` | Object Pattern + Containment | Static attitude struct | None | `attitude_estimator.h` / `.c` |
| **08** | `angle_computer` | Object Pattern (Stateless Utility)| Pure trigonometry | None | `angle_computer.h` / `.c` |
| **09** | `complementary_filter`| Object Pattern | Static filter state | None | `complementary_filter.h` / `.c` |
| **10** | `setpoint_manager` | Opaque ADT Pattern | Static setpoints | None | `setpoint_manager.h` / `.c` |
| **11** | `pid_controller` | Object Pattern + Containment | Stateless coordinator| None | `pid_controller.h` / `.c` |
| **12** | `roll_pid` | Strategy Pattern Instance | Static axis context | None | `pid_axis.h` / `.c` |
| **13** | `pitch_pid` | Strategy Pattern Instance | Static axis context | None | `pid_axis.h` / `.c` |
| **14** | `motor_mixer` | Object Pattern | Static command array | None | `motor_mixer.h` / `.c` |
| **15** | `pwm_driver` | Virtual API Pattern | Static channel map | PWM API | `pwm_driver.h` / `.c` |

---

## Detailed Module Catalog Entries

### MODULE 01: `flight_controller`
- **Role**: Top-level coordinator and 1 ms pipeline orchestrator.
- **Responsibilities**:
  - Own the main firmware entry point and startup sequence.
  - Coordinate pipeline order on every 1 ms tick:
    $$\text{IMU Read} \to \text{Attitude Update} \to \text{PID Compute} \to \text{Mixer} \to \text{Safety Gate} \to \text{PWM Write}$$
  - Receive the periodic `TMR-01` tick flag from the HAL timer ISR.
  - Guarantee that pipeline execution never overruns the 1 ms budget.
- **Internal State**:
  - `volatile bool g_tick_flag`: Signaled by hardware timer ISR.
  - `bool b_is_running`: Active execution flag.
- **Hardware Dependencies**: HAL 1 ms hardware timer callback.
- **Collaborators**: All modules (`imu_driver`, `attitude_estimator`, `pid_controller`, `motor_mixer`, `system_state_manager`, `pwm_driver`).

---

### MODULE 02: `system_state_manager`
- **Role**: Safety lifecycle gatekeeper.
- **Responsibilities**:
  - Own the system safety state machine (`DISARMED`, `ARMED`, `FAULT`).
  - Enforce valid transitions: `DISARMED <-> ARMED`, `ANY -> FAULT`.
  - Enforce one-way lock on `FAULT`: refuse all subsequent Arm commands.
  - Gate motor commands: pass mixed throttle in `ARMED`; force $0.0$ in `DISARMED` and `FAULT`.
  - Register and notify fault observers.
- **Internal State**:
  - `system_state_t current_state` (`STATE_DISARMED`, `STATE_ARMED`, `STATE_FAULT`).
  - `fault_callback_t fault_listeners[MAX_LISTENERS]`.
- **Hardware Dependencies**: None. Pure logical state machine.
- **Collaborators**: `flight_controller`, `pwm_driver`, fault sources.

---

### MODULE 03: `imu_driver`
- **Role**: Inertial sensor subsystem coordinator.
- **Responsibilities**:
  - Coordinate MPU6050 initialization and startup self-test.
  - Delegate raw I2C transactions to `imu_comm`.
  - Delegate integer-to-engineering unit conversion to `imu_converter`.
  - Subtract stored gyroscope bias offsets before returning readings.
  - Deliver calibrated data ($\text{m/s}^2$, $\text{deg/s}$) to `attitude_estimator`.
- **Internal State**:
  - `imu_sensor_data_t last_sample`: Current 3-axis accelerometer and gyro values.
  - `gyro_bias_t stored_bias`: Calibration offsets ($X, Y, Z$).
  - `bool b_initialized`: Initialization flag.
- **Hardware Dependencies**: MPU6050 register addresses via `imu_comm`.
- **Collaborators**: `imu_comm`, `imu_converter`, `gyro_calibrator`, `system_state_manager`.

---

### MODULE 04: `imu_comm`
- **Role**: HAL-facing I2C communication transceiver.
- **Responsibilities**:
  - Execute burst I2C reads (14 bytes) from MPU6050 registers (`0x3B` to `0x48`).
  - Read `WHO_AM_I` register (`0x75`) and verify expected value `0x68`.
  - Configure device power management and sample rate divider registers (`PWR_MGMT_1`, `SMPLRT_DIV`).
  - Catch and return I2C bus errors (timeouts, NACKs).
- **Internal State**: None. Pure stateless hardware protocol transceiver.
- **Hardware Dependencies**: HAL I2C driver APIs (`hal_i2c_read`, `hal_i2c_write`).
- **Collaborators**: `HAL`, `imu_driver`.

---

### MODULE 05: `imu_converter`
- **Role**: Stateless raw count to physical engineering unit scaler.
- **Responsibilities**:
  - Decode raw signed 16-bit big-endian register pairs into signed integers.
  - Scale accelerometer counts to $\text{m/s}^2$ ($16384 \text{ LSB}/g \to 9.80665 \text{ m/s}^2$).
  - Scale gyroscope counts to $\text{deg/s}$ ($131.0 \text{ LSB}/(^\circ/\text{s})$).
- **Internal State**: None. Pure stateless mathematical transform.
- **Hardware Dependencies**: None. Pure ISO C arithmetic.
- **Collaborators**: `imu_driver`.

---

### MODULE 06: `gyro_calibrator`
- **Role**: Startup zero-rate bias characterization.
- **Responsibilities**:
  - Collect $N = 512$ consecutive gyroscope samples at rest during boot.
  - Accumulate 64-bit sums per axis and compute mean zero-rate offsets.
  - Validate that resting variance does not exceed physical vibration limits.
  - Signal completion or failure to `system_state_manager`.
- **Internal State**:
  - `uint16_t sample_count`.
  - `double sum_x, sum_y, sum_z`.
  - `gyro_bias_t calculated_bias`.
- **Hardware Dependencies**: None.
- **Collaborators**: `imu_driver`, `system_state_manager`.

---

### MODULE 07: `attitude_estimator`
- **Role**: Sensor fusion subsystem coordinator.
- **Responsibilities**:
  - Coordinate orientation estimation on every 1 ms tick.
  - Delegate accelerometer vector geometry to `angle_computer`.
  - Delegate rate integration and sensor fusion to `complementary_filter`.
  - Monitor estimated angles for unrecoverable divergence ($> \pm 90^\circ$) and signal fault.
- **Internal State**:
  - `attitude_estimate_t current_attitude` (Roll, Pitch in degrees).
- **Hardware Dependencies**: None. Pure computation.
- **Collaborators**: `angle_computer`, `complementary_filter`, `pid_controller`, `system_state_manager`.

---

### MODULE 08: `angle_computer`
- **Role**: Stateless accelerometer geometric orientation calculator.
- **Responsibilities**:
  - Compute static geometric roll angle: $\phi = \text{atan2f}(a_y, a_z) \times \frac{180.0}{\pi}$.
  - Compute static geometric pitch angle: $\theta = \text{atan2f}(-a_x, \sqrt{a_y^2 + a_z^2}) \times \frac{180.0}{\pi}$.
- **Internal State**: None. Pure stateless trigonometric utility.
- **Hardware Dependencies**: C standard math library (`math.h`).
- **Collaborators**: `attitude_estimator`.

---

### MODULE 09: `complementary_filter`
- **Role**: Discrete-time orientation filter.
- **Responsibilities**:
  - Maintain persistent roll and pitch angle estimates across ticks.
  - Integrate angular velocity: $\theta_{\text{gyro}} = \theta_{\text{prev}} + \omega \cdot \Delta t$.
  - Fuse with accelerometer angle: $\theta = \alpha \cdot \theta_{\text{gyro}} + (1 - \alpha) \cdot \theta_{\text{accel}}$.
  - Use $\alpha = 0.98$ and $\Delta t = 0.001 \text{ s}$.
- **Internal State**:
  - `float roll_deg`, `float pitch_deg`.
  - `float alpha`, `float dt`.
- **Hardware Dependencies**: None. Pure mathematical filter.
- **Collaborators**: `attitude_estimator`.

---

### MODULE 10: `setpoint_manager`
- **Role**: Target orientation data owner.
- **Responsibilities**:
  - Own target orientation setpoints (default bench hover: $0.0^\circ$ roll, $0.0^\circ$ pitch).
  - Expose read accessors to `pid_controller`.
  - Provide an isolated API seam for future pilot RC receiver inputs.
- **Internal State**:
  - `float roll_setpoint_deg`.
  - `float pitch_setpoint_deg`.
- **Hardware Dependencies**: None. Pure data ADT.
- **Collaborators**: `pid_controller`.

---

### MODULE 11: `pid_controller`
- **Role**: Closed-loop attitude stabilization coordinator.
- **Responsibilities**:
  - Act as coordinator for dual-axis closed-loop attitude stabilization.
  - Query current angles from `attitude_estimator` and targets from `setpoint_manager`.
  - Delegate roll error computation to `roll_pid`.
  - Delegate pitch error computation to `pitch_pid`.
  - Deliver dynamic correction demands to `motor_mixer`.
- **Internal State**: None at coordinator level (state is encapsulated in axis instances).
- **Hardware Dependencies**: None.
- **Collaborators**: `attitude_estimator`, `setpoint_manager`, `roll_pid`, `pitch_pid`, `motor_mixer`.

---

### MODULES 12 & 13: `roll_pid` and `pitch_pid`
- **Role**: Independent axis PID control strategy instances.
- **Responsibilities**:
  - Compute error: $e(t) = \theta_{\text{target}} - \theta_{\text{actual}}$.
  - Compute proportional term: $P = K_p \cdot e(t)$.
  - Accumulate integral term: $I(t) = I(t-1) + K_i \cdot e(t) \cdot \Delta t$.
  - Apply anti-windup clamping to prevent integrator runaway: $I = \text{clamp}(I, -\text{LIMIT}, \text{LIMIT})$.
  - Compute derivative term: $D = K_d \cdot \frac{e(t) - e(t-1)}{\Delta t}$.
  - Return total demand: $\text{Output} = P + I + D$.
- **Internal State**:
  - `float integral_accumulator`.
  - `float prev_error`.
  - `float kp, ki, kd, windup_limit`.
- **Hardware Dependencies**: None. Pure mathematical control law.
- **Collaborators**: `pid_controller`.

---

### MODULE 14: `motor_mixer`
- **Role**: Quad-X geometry mixer and output safety clamper.
- **Responsibilities**:
  - Combine base throttle with roll and pitch demands across Quad-X geometry:
    $$M_1 = T + R - P, \quad M_2 = T - R - P, \quad M_3 = T + R + P, \quad M_4 = T - R + P$$
  - Clamp all outputs within safe operating bounds: $[\text{MOTOR\_MIN}, \text{MOTOR\_MAX}]$.
  - Detect pre-clamp bounds violations indicating controller runaway and assert fault signal.
- **Internal State**:
  - `float motor_commands[4]`.
  - `float base_throttle`.
- **Hardware Dependencies**: None. Pure mixing arithmetic.
- **Collaborators**: `pid_controller`, `system_state_manager`, `pwm_driver`.

---

### MODULE 15: `pwm_driver`
- **Role**: HAL-facing timer compare actuator.
- **Responsibilities**:
  - Map normalized motor commands ($[0.0, 1.0]$) to PWM duty cycle compare counts.
  - Write duty cycles to 4 independent timer compare channels via the HAL.
  - Implement `pwm_driver_zero_all()` to immediately set all timer registers to zero upon `FaultDetected`.
- **Internal State**:
  - `uint32_t channel_map[4]`: Maps logical motor index to HAL PWM channel IDs.
- **Hardware Dependencies**: HAL PWM APIs (`hal_pwm_set_duty`, `hal_pwm_stop`).
- **Collaborators**: `HAL`, `motor_mixer`, `system_state_manager`.

---

## Next Step

With all 15 modules specified, the next document, [TDD Test Strategy](13-test-strategy.md), defines the host-based Dual-Target testing framework, test double designs, and boundary assertion rules.
