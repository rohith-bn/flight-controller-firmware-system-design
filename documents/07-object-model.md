# Object Model

## Purpose Of This Document

This document defines the conceptual object model for QFC-1. It formalizes the major object identities that emerge from the system responsibilities and the earlier comparison between object-oriented reasoning and module-oriented implementation structure.

The goal is not to finalize code-level structure yet. The goal is to identify the key conceptual elements of the system, explain why they deserve object identity, and show how they relate to each other at a design level.

## Why This Follows Candidate Objects Vs Functional Modules

The previous document clarified an important distinction:

- object thinking is useful for reasoning about responsibility and collaboration
- module thinking is useful for practical firmware implementation

With that distinction in place, the next step is to formalize the conceptual object view of the system before refining it further through CRC cards and later decomposition.

## What This Object Model Represents

This object model is a conceptual design artifact.

It represents:

- the major named objects in the system
- the high-level responsibilities of each
- the relationships between them

It does not yet represent:

- final source-file boundaries
- exact function signatures
- runtime sequencing details
- detailed collaborator contracts

Those later concerns will build on this model, not replace it.

## Object Identification Criteria

An element is treated as an object in this model when it has most of the following qualities:

- a coherent responsibility center
- a meaningful role in the behavior of the system
- recognizable collaborations with other parts
- conceptual value beyond being just a helper or low-level utility

This is why not every implementation unit becomes an object, and not every object must eventually become a rich class-like construct in code.

## Candidate Object Set For QFC-1

The current conceptual object set for QFC-1 is:

- `IMUDriver`
- `GyroCalibrator`
- `AttitudeEstimator`
- `SetpointManager`
- `PIDController`
- `MotorMixer`
- `PWMDriver`
- `SystemStateManager`
- `FlightController`

Together, these objects represent the major behavioral and coordination elements required to model QFC-1 conceptually.

## Object Summaries

### IMUDriver

`IMUDriver` represents the sensing-access object responsible for obtaining raw inertial data through the trusted hardware boundary.

At the conceptual level, it owns:

- IMU identity validation
- raw sensor access
- controlled interaction with the HAL sensing path

### GyroCalibrator

`GyroCalibrator` represents the startup calibration logic required to determine and preserve gyroscope bias information.

At the conceptual level, it owns:

- startup calibration behavior
- bias computation
- readiness of gyroscope correction data

### AttitudeEstimator

`AttitudeEstimator` represents the object responsible for transforming processed inertial information into roll and pitch estimates.

At the conceptual level, it owns:

- interpretation of inertial data
- attitude estimation logic
- production of stable roll and pitch estimates for downstream control

### SetpointManager

`SetpointManager` represents the object that owns desired operating targets for the current system scope.

At the conceptual level, it owns:

- bench-validation roll and pitch setpoints
- access to desired attitude values

In this project, it is intentionally simple, but it still has enough identity to be modeled explicitly.

### PIDController

`PIDController` represents the object responsible for computing corrective demands from attitude error.

At the conceptual level, it owns:

- roll and pitch control computation
- PID behavior
- integral windup management

### MotorMixer

`MotorMixer` represents the object that translates control outputs into four motor command values suitable for the quadcopter geometry.

At the conceptual level, it owns:

- Quad-X mixing logic
- command combination
- safe preparation of motor-level outputs before final actuation

### PWMDriver

`PWMDriver` represents the actuation-output object responsible for expressing final motor commands through the HAL.

At the conceptual level, it owns:

- PWM output intent
- interaction with the output path
- safe presentation of final motor commands

### SystemStateManager

`SystemStateManager` represents the safety-state object responsible for governing valid operating modes and safe output rules.

At the conceptual level, it owns:

- `DISARMED`, `ARMED`, and `FAULT`
- state transitions
- enforcement of state-dependent output permissions

### FlightController

`FlightController` represents the top-level coordinating object of QFC-1.

At the conceptual level, it owns:

- startup coordination
- control-loop orchestration
- sequencing of major pipeline stages
- integration of control, state, and actuation flow

It is not the owner of all detailed behavior, but it is the coordinator of the overall system behavior.

## Relationships Between Objects

The object relationships can be understood at a high level as follows:

- `FlightController` coordinates the overall execution flow.
- `IMUDriver` provides raw inertial data.
- `GyroCalibrator` provides bias information needed for meaningful sensor interpretation.
- `AttitudeEstimator` depends on processed inertial inputs and calibration context.
- `SetpointManager` provides desired target values.
- `PIDController` depends on estimated attitude and desired setpoints.
- `MotorMixer` depends on controller outputs and turns them into motor command values.
- `SystemStateManager` determines whether motor output is allowed and how safety state affects system behavior.
- `PWMDriver` represents the final actuation path used to express valid motor commands outward.

These relationships describe collaboration, not yet exact call-level implementation.

## Central And Supporting Objects

Some objects are more central to the conceptual behavior of the system, while others primarily support that behavior.

### Central Objects

- `FlightController`
- `AttitudeEstimator`
- `PIDController`
- `SystemStateManager`

These are central because they dominate the control, coordination, and safety meaning of the system.

### Supporting Objects

- `IMUDriver`
- `GyroCalibrator`
- `SetpointManager`
- `MotorMixer`
- `PWMDriver`

These are supporting because they provide enabling behavior, transformation, or boundary interaction for the central control flow.

This distinction is useful because it helps avoid treating every object as equally central when thinking about later design refinements.

## Object Model Summary Table

| Object | Primary role | Design reason |
|---|---|---|
| `IMUDriver` | Sensor access | Owns trusted sensing boundary interaction |
| `GyroCalibrator` | Startup calibration | Owns gyro-bias preparation |
| `AttitudeEstimator` | State estimation | Owns roll/pitch estimation logic |
| `SetpointManager` | Target management | Owns desired roll/pitch values |
| `PIDController` | Control computation | Owns corrective-demand generation |
| `MotorMixer` | Command translation | Owns Quad-X motor command generation |
| `PWMDriver` | Output actuation | Owns final PWM-oriented output expression |
| `SystemStateManager` | Safety state | Owns modes and output-enabling rules |
| `FlightController` | Coordination | Owns top-level sequencing and orchestration |

## Limits Of The Current Object Model

This model is intentionally high level.

It does not yet answer:

- exactly which objects collaborate in which scenarios
- which collaborators are direct versus indirect
- which objects may later split into smaller units
- how the final C module layout should mirror or simplify the model

Those are the next questions, and they belong in the CRC and decomposition steps.

## What This Enables Next

With the candidate object model now defined, the next steps become clearer:

- analyze each object through CRC-style responsibility and collaborator thinking
- refine the collaboration network between objects
- decide which parts of the object model should remain conceptually rich and which should later collapse into simpler implementation modules

This document stabilizes the conceptual object view of QFC-1. It gives the project a named structural vocabulary before deeper collaboration and decomposition work begins.
