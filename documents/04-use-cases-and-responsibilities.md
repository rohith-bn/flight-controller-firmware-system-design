# Use Cases And Responsibilities

## Purpose Of This Document

This document captures the major use cases that drive QFC-1 behavior and derives the system's grouped functional responsibilities from those use cases. It sits between system context and later structural design steps such as candidate objects, functional modules, and runtime modeling.

## Why This Follows System Context

System context defined where QFC-1 begins and ends, which actors interact with it, and what assumptions shape its environment. The next design question is behavioral:

What does this bounded system actually do in response to its actors, timing triggers, and fault conditions?

Use cases answer that question from the outside. Responsibilities then group those behaviors into coherent obligations that the system must own before we decide how to structure the internals.

## Use Cases

### UC-01: System Startup And Self-Check

**Primary actor:** Developer / Operator  
**Trigger:** Power-on or hardware reset  
**Precondition:** HAL is initialized and hardware connections are available  
**Goal:** Bring the system into a safe, known state before normal operation

**Main flow:**

1. Firmware begins execution after system startup.
2. The IMU is accessed through the HAL.
3. The firmware verifies IMU identity using the expected startup check.
4. The firmware performs gyroscope bias calibration while the board is stationary.
5. Initial internal state is prepared for normal operation.
6. The system enters `DISARMED`.

**Success result:** The system is alive, calibrated, safe, and ready for controlled activation.  
**Failure result:** Any startup failure transitions the system to `FAULT`.

### UC-02: Arm Command Received

**Primary actor:** Developer / Operator  
**Trigger:** Explicit arm action  
**Precondition:** System is in `DISARMED` and no active fault exists  
**Goal:** Enable active motor-output behavior under controlled conditions

**Main flow:**

1. The operator requests arm.
2. The firmware checks that the current state permits arming.
3. The system transitions from `DISARMED` to `ARMED`.
4. Motor-output behavior becomes eligible during the normal control loop.

**Success result:** The system enters `ARMED`.  
**Failure result:** Invalid arming conditions do not activate motor output and may lead to fault handling if required.

### UC-03: Normal Stabilization Loop

**Primary actors:** IMU Hardware, Physical Environment  
**Trigger:** 1 ms periodic timing tick  
**Precondition:** System is initialized and in a valid operating state  
**Goal:** Continuously stabilize roll and pitch within the deterministic execution budget

**Main flow:**

1. Raw accelerometer and gyroscope data are read through the HAL.
2. Sensor values are converted into engineering units.
3. Gyroscope bias is accounted for in the processed readings.
4. Roll and pitch are estimated using the chosen estimation strategy.
5. Estimated attitude is compared with internal setpoints.
6. Roll and pitch corrective outputs are computed.
7. Control outputs are translated into four motor commands.
8. Motor commands are clamped to safe bounds.
9. PWM outputs are updated if the system state allows active output.
10. The loop returns before the next 1 ms tick.

**Success result:** The system performs one bounded stabilization cycle and repeats.  
**Failure result:** Any invalid sensor, estimation, timing, or command condition leads to fault handling.

### UC-04: Fault Detection And Shutdown

**Primary actor:** IMU Hardware or internal safety logic  
**Trigger:** Fault condition detected  
**Precondition:** System is in any state  
**Goal:** Force the system into a safe non-recovering state

**Main flow:**

1. A fault is detected, such as IMU identity mismatch, I2C failure, estimator divergence, or unsafe command condition.
2. The firmware immediately forces motor outputs to zero.
3. The system transitions to `FAULT`.
4. The system remains in `FAULT` until hardware reset.

**Success result:** Unsafe behavior is terminated and the system becomes safe.  
**Failure result:** There is no alternate success path; safety behavior must be unconditional.

### UC-05: Disarm

**Primary actor:** Developer / Operator  
**Trigger:** Explicit disarm action  
**Precondition:** System is in `ARMED`  
**Goal:** Return the system to a safe non-actuating mode without stopping internal processing

**Main flow:**

1. The operator requests disarm.
2. The system transitions from `ARMED` to `DISARMED`.
3. Motor outputs are forced to zero.
4. Internal estimation and control-related processing may continue, but output actuation is suppressed.

**Success result:** The system returns to `DISARMED`.  
**Failure result:** If disarm handling cannot guarantee zero output, the system must fail safe.

## Behavioral Patterns Across Use Cases

Across the use cases, a few recurring patterns become visible:

- Startup behavior is cautious and validation-driven.
- Steady-state behavior is timer-driven and deterministic.
- Safety behavior overrides all other goals.
- Human actions affect mode transitions rather than low-level control behavior.
- Normal operation and fault handling are not symmetric; `FAULT` is intentionally one-way.

These patterns matter because they begin to reveal the real shape of the system before any internal structure is chosen.

## Derived Functional Responsibilities

The formal requirements document already captures many detailed obligations. The purpose of this section is not to repeat those requirements line by line, but to group them into coherent higher-level responsibilities that the system must own.

### 1. Sensor Acquisition Responsibility

QFC-1 must acquire the inertial data required for stabilization on every control tick through the trusted hardware abstraction boundary.

This includes:

- Accessing accelerometer and gyroscope readings
- Using the HAL correctly for sensor interaction
- Maintaining bounded interaction with the sensing path

### 2. Startup Validation And Calibration Responsibility

QFC-1 must verify that the sensing path is valid before active operation begins and must establish the initial calibration needed for meaningful control.

This includes:

- IMU identity validation
- Startup readiness checks
- Gyroscope bias calibration
- Safe transition into initial operating state

### 3. Signal Conditioning Responsibility

QFC-1 must convert raw measurements into meaningful engineering values suitable for estimation and control.

This includes:

- Converting sensor counts into engineering units
- Applying calibration offsets
- Preparing sensor data for downstream processing

### 4. Attitude Estimation Responsibility

QFC-1 must estimate the current roll and pitch state of the airframe from noisy inertial measurements.

This includes:

- Combining accelerometer and gyroscope information
- Producing roll and pitch estimates
- Maintaining an estimate stable enough for closed-loop control

### 5. Setpoint And Control Responsibility

QFC-1 must compare current estimated attitude with desired attitude and compute corrective demands for the controlled axes.

This includes:

- Maintaining bench-validation setpoints
- Computing roll and pitch error
- Running PID-based correction logic
- Managing integral windup limits

### 6. Motor Command Generation Responsibility

QFC-1 must translate control outputs into actuation-ready motor commands appropriate for the quadcopter geometry.

This includes:

- Quad-X command mixing
- Safe clamping of command values
- Preparation of PWM-ready outputs

### 7. Actuation Responsibility

QFC-1 must present valid output commands through the HAL in a form the ESC and motor system can use.

This includes:

- Updating PWM outputs
- Ensuring outputs are emitted only when allowed by system state
- Preserving bounded timing in the actuation path

### 8. Safety And State Management Responsibility

QFC-1 must enforce safe operating modes and guarantee safe output behavior under all conditions.

This includes:

- Managing `DISARMED`, `ARMED`, and `FAULT`
- Enforcing zero output in `DISARMED`
- Enforcing zero output in `FAULT`
- Supporting explicit arm/disarm transitions
- Preventing automatic recovery from `FAULT`

### 9. Fault Detection And Response Responsibility

QFC-1 must recognize unsafe or invalid conditions and convert them into an immediate and deterministic safety response.

This includes:

- Detecting startup and runtime faults
- Detecting invalid sensor or estimation conditions
- Detecting unsafe command conditions
- Forcing motor shutdown and entering `FAULT`

### 10. Control-Loop Orchestration Responsibility

QFC-1 must coordinate all major pipeline stages within the 1 ms execution budget.

This includes:

- Sequencing sensing, estimation, control, mixing, and actuation
- Respecting deterministic timing constraints
- Avoiding blocking behavior in the steady-state path
- Preserving a stable runtime structure around the periodic tick

## Why Responsibilities Matter Before Objects Or Modules

Responsibilities are the bridge between required behavior and internal structure.

At this stage, we are not yet deciding whether a responsibility should become:

- an object
- a functional module
- a helper routine
- a state-oriented subsystem

Instead, we are identifying the major obligations the system must own. That makes later structural decisions more disciplined because candidate objects or modules will emerge from responsibilities rather than being invented too early.

## What This Enables Next

With the major use cases and grouped responsibilities now identified, the next design steps become more concrete:

- compare possible design approaches for structuring the system
- explore candidate objects versus functional modules
- assign responsibilities to internal structure more intentionally
- prepare for event modeling and runtime analysis

This document moves the project from context definition into behavioral design. It answers not just what surrounds the system, but what the system must consistently do within that world.
