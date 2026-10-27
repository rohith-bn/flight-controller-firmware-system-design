# CRC Cards

## Purpose Of This Document

This document defines the Class-Responsibility-Collaborator (CRC) card set for QFC-1. It refines the 9 conceptual candidate objects established in [Object Model](07-object-model.md) by defining exact behavioral boundaries, explicit obligations, and directional collaboration contracts.

The goal is to eliminate ambiguity in ownership before decomposing objects into implementation modules.

## Why This Follows The Object Model

The previous document identified the 9 core candidate objects of QFC-1. However, identifying an object name is not enough to begin implementation. We must answer three structural questions for each entity:

1. **What is its identity?** (Class / Object name)
2. **What does it own?** (Responsibilities — behaviors, calculations, and state)
3. **Who does it need help from?** (Collaborators — other objects it calls or receives data from)

CRC cards provide this bridge without introducing C syntax, pointers, or premature header layout.

## CRC Modeling Rules For QFC-1

1. **Single Responsibility**: Each responsibility belongs to exactly one object. An object must never carry unrelated obligations.
2. **Delegation Over Monoliths**: Objects delegate specialized work rather than doing everything internally.
3. **Directional Collaboration**: Every collaborator relationship is purposeful and directional. Circular dependencies are prohibited.
4. **Hardware Boundary Isolation**: The Hardware Abstraction Layer (HAL) appears as a collaborator only for hardware-facing objects (`PWMDriver` and `FlightController`). Pure algorithmic objects have zero HAL dependency.
5. **Centralized Safety Ownership**: `SystemStateManager` is the sole owner of safety state decisions. No subordinate module may independently decide to enable or disable motors.

---

## The QFC-1 CRC Card Set

### CRC CARD 1: `IMUDriver`

```text
+-------------------------------------------------------------------------------+
| Class: IMUDriver                                                              |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Initialize the MPU6050 sensor on boot           | - GyroCalibrator          |
| - Verify device identity via WHO_AM_I register    |   (provides bias offset   |
| - Read raw 3-axis accelerometer data every tick   |    to subtract from gyro) |
| - Read raw 3-axis gyroscope data every tick       | - SystemStateManager      |
| - Apply gyroscope bias correction to raw readings |   (receives I2C fault and |
| - Convert raw integer counts to engineering units |    init failure signals)  |
|   (m/s^2 for accel, deg/s for gyro)               | - FlightController        |
| - Detect and report I2C communication failures    |   (calls read every tick) |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 2: `GyroCalibrator`

```text
+-------------------------------------------------------------------------------+
| Class: GyroCalibrator                                                         |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Request raw gyroscope samples from IMUDriver    | - IMUDriver               |
|   during startup calibration window               |   (source of raw samples) |
| - Accumulate N stationary samples at rest         | - SystemStateManager      |
| - Compute average zero-rate bias offset per axis  |   (signals outcome:       |
| - Store computed bias values for runtime use      |    success -> DISARMED,   |
| - Expose bias values for IMUDriver consumption    |    failure -> FAULT)      |
| - Validate resting noise variance                 |                           |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 3: `AttitudeEstimator`

```text
+-------------------------------------------------------------------------------+
| Class: AttitudeEstimator                                                      |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Receive calibrated accel and gyro data per tick | - IMUDriver               |
| - Compute roll angle from accel using atan2f      |   (source of sensor data) |
| - Compute pitch angle from accel using atan2f     | - PIDController           |
| - Integrate gyro angular rate forward in time     |   (consumer of estimated  |
| - Apply complementary filter fusion (alpha=0.98)  |    roll & pitch angles)   |
| - Output stable, drift-free roll and pitch angles | - SystemStateManager      |
| - Detect estimator divergence beyond safe bounds  |   (receives divergence    |
|   (> +/- 90 degrees) and report fault             |    fault notification)    |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 4: `SetpointManager`

```text
+-------------------------------------------------------------------------------+
| Class: SetpointManager                                                        |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Own desired roll angle (bench default: 0.0 deg) | - PIDController           |
| - Own desired pitch angle (bench default: 0.0 deg)|   (queries target angles  |
| - Provide roll setpoint on demand                 |    for error calculation) |
| - Provide pitch setpoint on demand                |                           |
| - Expose update API for future RC receiver input  |                           |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 5: `PIDController`

```text
+-------------------------------------------------------------------------------+
| Class: PIDController                                                         |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Receive estimated angles from AttitudeEstimator | - AttitudeEstimator       |
| - Receive target setpoints from SetpointManager   |   (source of actual angle)|
| - Compute angle error for roll and pitch axes     | - SetpointManager         |
| - Compute proportional term per axis (Kp * error) |   (source of target angle)|
| - Accumulate integral term (Ki * error * dt)      | - MotorMixer              |
| - Apply anti-windup saturation limits on integral |   (consumer of corrective |
| - Compute derivative term (Kd * delta_error / dt) |    torque demands)        |
| - Sum P + I + D into net correction demand        |                           |
| - Output roll and pitch correction demands        |                           |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 6: `MotorMixer`

```text
+-------------------------------------------------------------------------------+
| Class: MotorMixer                                                             |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Receive roll & pitch correction demands         | - PIDController           |
| - Receive base throttle reference value           |   (source of corrections) |
| - Apply Quad-X geometric mixing equations:        | - PWMDriver               |
|   M1 (FL) = Throttle + Roll - Pitch               |   (consumer of motor      |
|   M2 (FR) = Throttle - Roll - Pitch               |    throttle commands)     |
|   M3 (RL) = Throttle + Roll + Pitch               | - SystemStateManager      |
|   M4 (RR) = Throttle - Roll + Pitch               |   (receives pre-clamp     |
| - Clamp motor outputs within [MOTOR_MIN, MAX]     |    bounds violations;     |
| - Detect pre-clamp bounds violations as fault     |    enforces zero-output)  |
| - Output 4 individual motor throttle values       |                           |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 7: `PWMDriver`

```text
+-------------------------------------------------------------------------------+
| Class: PWMDriver                                                              |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Receive 4 motor throttle values from MotorMixer | - MotorMixer              |
| - Map throttle [0.0, 1.0] to PWM compare counts   |   (source of throttle)    |
| - Write duty cycles to Channel 1 (Front-Left)     | - HAL                     |
| - Write duty cycles to Channel 2 (Front-Right)    |   (target interface for   |
| - Write duty cycles to Channel 3 (Rear-Left)      |    timer compare writes)  |
| - Write duty cycles to Channel 4 (Rear-Right)     |                           |
| - Execute zero_all() immediately on fault signal  |                           |
| - Delegate actual hardware writes to the HAL      |                           |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 8: `SystemStateManager`

```text
+-------------------------------------------------------------------------------+
| Class: SystemStateManager                                                     |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Initialize system safety state to DISARMED      | - FlightController        |
| - Process ArmCommand: DISARMED -> ARMED           |   (source of operator     |
| - Process DisarmCommand: ARMED -> DISARMED        |    arm/disarm triggers)   |
| - Process FaultSignal from any module: -> FAULT   | - IMUDriver               |
| - Enforce one-way latching (refuse transitions    |   (I2C & init faults)     |
|   out of FAULT state)                             | - GyroCalibrator          |
| - Enforce zero motor output in DISARMED & FAULT   |   (calibration faults)    |
| - Gate motor commands before PWM generation       | - AttitudeEstimator       |
| - Expose current system state to all modules      |   (divergence faults)     |
| - Register and notify fault observers             | - MotorMixer              |
|                                                   |   (bounds violation faults|
|                                                   |    & output gating)       |
|                                                   | - PWMDriver               |
|                                                   |   (zero-output enforcer)  |
+---------------------------------------------------+---------------------------+
```

### CRC CARD 9: `FlightController`

```text
+-------------------------------------------------------------------------------+
| Class: FlightController                                                       |
+---------------------------------------------------+---------------------------+
| Responsibilities:                                 | Collaborators:            |
| - Receive periodic 1ms tick signal from HAL       | - HAL                     |
| - Execute startup orchestrator on boot:           |   (tick source)           |
|   -> IMUDriver.init()                             | - IMUDriver               |
|   -> GyroCalibrator.run()                         | - AttitudeEstimator       |
|   -> Transition to DISARMED                       | - PIDController           |
| - Sequence the full 1ms pipeline every tick:      | - MotorMixer              |
|   1. IMUDriver.read()                             | - SystemStateManager      |
|   2. AttitudeEstimator.update()                   | - PWMDriver               |
|   3. PIDController.compute()                      |                           |
|   4. MotorMixer.mix()                             |                           |
|   5. SystemStateManager.gate()                    |                           |
|   6. PWMDriver.write()                            |                           |
| - Guarantee no pipeline stage overruns tick budget|                           |
| - Forward operator commands to SystemStateManager |                           |
+---------------------------------------------------+---------------------------+
```

---

## Collaboration Validation

The CRC model satisfies four strict architectural invariants:
1. **Directional Flow**: The control pipeline moves strictly forward ($1 \to 2 \to 3 \to 4 \to 5 \to 6$). There are zero recursive callbacks or circular dependencies.
2. **Decoupled Safety Enforcer**: `SystemStateManager` is the exclusive authority on state transitions and output gating. No sensor driver or control algorithm can independently turn on or shut down motors.
3. **Single Master Sequencer**: `FlightController` is the sole pipeline dispatcher. Subordinate objects never call each other out of sequence.
4. **Clean HAL Isolation**: The HAL appears as a collaborator only for `PWMDriver` (actuation) and `FlightController` (timer tick). Pure business logic objects have zero connection to silicon.

---

## Next Step

With object responsibilities and collaborator contracts firmly established, the next document, [Object Model Decomposition](09-object-model-decomposition.md), breaks these 9 candidate objects into fine-grained, implementation-ready modules, cleanly separating pure mathematics from hardware communication.
