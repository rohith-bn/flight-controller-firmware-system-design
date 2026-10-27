# Object Model Decomposition

## Purpose Of This Document

This document defines the Level 2 Object Model decomposition for QFC-1. It analyzes each of the 9 candidate objects established in [CRC Cards](08-crc-cards.md) against single responsibility and testability criteria, decomposing entities that conflate hardware communication with pure mathematical computation.

The result is a concrete, implementation-ready structural hierarchy of 15 modular units that directly map to C header and source files.

## Why This Follows CRC Cards

The CRC cards clarified what each candidate object is responsible for. However, inspecting those responsibilities reveals an important architectural tension:
- Some objects bundle **hardware I/O protocol mechanics** with **pure algorithmic mathematics**.
- Some objects bundle **a generic control algorithm** with **independent axis state management**.

If left undecomposed, testing an angle calculation or unit conversion on the host would require mocking an entire I2C bus transaction. 

Decomposition separates pure mathematical logic from hardware-dependent seams, enabling **zero-mock host unit testing**.

## Decomposition Criteria

Each candidate object is evaluated against one guiding question:
> *"Is this object doing too many unrelated things, or does it conflate hardware communication with pure computation?"*

If an object contains distinct concerns that can be developed, tested, or reused independently, it is decomposed into sub-modules. If its responsibilities are tightly bound to a single cohesive concern, it is preserved without decomposition.

---

## Decomposition Analysis

### 1. `IMUDriver` -> **DECOMPOSE**
- **Analysis**: Carries two fundamentally distinct concerns:
  1. *Hardware Communication*: Executing I2C bus transactions to the MPU6050, verifying the `WHO_AM_I` register, and handling bus error conditions (timeouts, NACKs).
  2. *Unit Conversion*: Decoding signed two's complement 16-bit register pairs and scaling raw integer counts into physical engineering units ($\text{m/s}^2$ for acceleration, $\text{deg/s}$ for angular velocity).
- **Architectural Rationale**: Communication is hardware-facing and requires a HAL interface. Conversion is pure mathematics with zero side effects. Keeping them together forces unit conversion tests to mock I2C.
- **Decomposed Structure**:
  ```text
  IMUDriver
  ├── IMUComm
  │     Owns: I2C transactions, WHO_AM_I validation, raw register reads, error detection.
  └── IMUConverter
        Owns: Raw count to engineering unit conversion for accelerometer and gyroscope.
  ```

### 2. `GyroCalibrator` -> **NO DECOMPOSITION NEEDED**
- **Analysis**: Owns a single cohesive concern: collecting stationary gyro samples during startup, calculating an arithmetic mean offset, and storing the result. All operations are tightly bound to the initialization lifecycle.

### 3. `AttitudeEstimator` -> **DECOMPOSE**
- **Analysis**: Carries two distinct mathematical concerns:
  1. *Stateless Geometry*: Computing instantaneous roll and pitch angles from accelerometer force vectors using trigonometric projections (`atan2f`).
  2. *Stateful Sensor Fusion*: Propagating gyro angular rates across time, managing the $\alpha$ filter weight, and blending the estimates.
- **Architectural Rationale**: Accelerometer angle computation is completely stateless (same input vector always produces the exact same angle). The complementary filter owns persistent state that evolves over time. Separating them allows geometry to be tested with simple lookup tables and the filter to be tested with dynamic waveforms.
- **Decomposed Structure**:
  ```text
  AttitudeEstimator
  ├── AngleComputer
  │     Owns: Trigonometric roll and pitch calculation from accelerometer vectors (stateless).
  └── ComplementaryFilter
        Owns: Persistent filter state, alpha weighting, gyro integration, and fused outputs.
  ```

### 4. `SetpointManager` -> **NO DECOMPOSITION NEEDED**
- **Analysis**: Owns a single cohesive concern: maintaining and exposing target roll and pitch setpoints. It acts as an Abstract Data Type (ADT) data owner.

### 5. `PIDController` -> **DECOMPOSE**
- **Analysis**: Carries two distinct operational concerns:
  1. *Generic Control Algorithm*: Computing P, I, and D terms with integral anti-windup clamping.
  2. *Independent Axis State Management*: Roll and Pitch operate on independent sensor inputs, have distinct aerodynamic moments of inertia, require independent gain tuning ($K_p, K_i, K_d$), and track independent integrator states.
- **Architectural Rationale**: Decomposing by axis eliminates code duplication while allowing Roll and Pitch to be tuned, tested, and state-isolated independently.
- **Decomposed Structure**:
  ```text
  PIDController
  ├── RollPID
  │     Owns: Roll error calculation, roll P/I/D terms, windup limits, roll correction demand.
  └── PitchPID
        Owns: Pitch error calculation, pitch P/I/D terms, windup limits, pitch correction demand.
  ```

### 6. `MotorMixer` -> **NO DECOMPOSITION NEEDED**
- **Analysis**: Owns a single cohesive concern: mapping pitch, roll, and throttle demands into Quad-X motor commands and enforcing output safety clamping. Mixing and clamping cannot be separated safely.

### 7. `PWMDriver` -> **NO DECOMPOSITION NEEDED**
- **Analysis**: Owns a single cohesive concern: mapping normalized motor throttle percentages ($[0.0, 1.0]$) to hardware timer compare register values and delegating writes to the HAL.

### 8. `SystemStateManager` -> **NO DECOMPOSITION NEEDED**
- **Analysis**: Owns a single cohesive concern: governing the system lifecycle state machine (`DISARMED`, `ARMED`, `FAULT`), handling state transitions, and gating motor actuation.

### 9. `FlightController` -> **NO DECOMPOSITION NEEDED**
- **Analysis**: Owns a single cohesive concern: orchestrating the sequential execution of all pipeline stages within the deterministic 1 ms tick budget.

---

## Level 2 Object Hierarchy

```text
FlightController (Master 1ms Orchestrator)
│
├── SystemStateManager (Safety & Lifecycle Gate)
│
├── IMUDriver (Sensor Acquisition Subsystem)
│   ├── IMUComm (HAL-facing I2C Transceiver)
│   └── IMUConverter (Pure Mathematical Count-to-Unit Scaler)
│
├── GyroCalibrator (Startup Zero-Rate Bias Characterization)
│
├── AttitudeEstimator (Orientation Fusion Subsystem)
│   ├── AngleComputer (Stateless Accelerometer Trigonometry)
│   └── ComplementaryFilter (Stateful Rate Integration & Alpha Fusion)
│
├── SetpointManager (Target Orientation Data Owner)
│
├── PIDController (Closed-Loop Correction Subsystem)
│   ├── RollPID (Roll Axis Control Strategy Instance)
│   └── PitchPID (Pitch Axis Control Strategy Instance)
│
├── MotorMixer (Quad-X Geometry Mixer & Safety Clamper)
│
└── PWMDriver (HAL-facing Timer Compare Actuator)
```

---

## Architectural Mapping to Implementation Modules

Decomposing the 9 candidate objects yields **15 concrete implementation modules**:

| Module Name | Architectural Category | Primary Role | HAL Dependency |
| :--- | :--- | :--- | :---: |
| `flight_controller` | Orchestration | Master pipeline sequencer | Yes (Timer Tick) |
| `system_state_manager`| Safety Gate | Tri-state machine & output gate | None |
| `imu_driver` | Orchestration | Sensor subsystem coordinator | None |
| `imu_comm` | Hardware Seam | I2C protocol transceiver | Yes (I2C) |
| `imu_converter` | Pure Math | Signed count to engineering unit scaler | None |
| `gyro_calibrator` | Pure Logic | Sample accumulation & bias calculator | None |
| `attitude_estimator` | Orchestration | Estimation subsystem coordinator | None |
| `angle_computer` | Pure Math | Stateless accelerometer trigonometry | None |
| `complementary_filter`| Pure Math | Discrete-time rate & angle fusion | None |
| `setpoint_manager` | Pure Data | Target orientation data owner | None |
| `pid_controller` | Orchestration | Dual-axis control coordinator | None |
| `roll_pid` | Pure Math | Roll axis PID strategy instance | None |
| `pitch_pid` | Pure Math | Pitch axis PID strategy instance | None |
| `motor_mixer` | Pure Math | Quad-X matrix mixer & safety clamper | None |
| `pwm_driver` | Hardware Seam | Timer compare value generator | Yes (PWM) |

---

## Next Step

With structural module boundaries finalized, the next document, [Event Model](10-event-model.md), identifies the external, timer, and internal stimuli that drive the system, modeling startup, normal cyclic execution, and fault flows.
