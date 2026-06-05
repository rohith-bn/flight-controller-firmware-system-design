# QFC-1: Flight Controller Firmware - A System Design Approach

QFC-1 is a learning-focused embedded firmware project for a quadcopter flight controller. The goal is not only to write firmware, but to demonstrate a system design approach to embedded software: clear boundaries, explicit requirements, deterministic timing, safety-first state management, modular decomposition, and test-driven development.

The firmware stabilizes a quadcopter airframe on the roll and pitch axes by repeatedly running a sense-estimate-control-actuate loop within a fixed 1 ms cycle.

## Naming Note

`flight-controller-firmware-system-design` is the repository name for the overall learning project.

`QFC-1` is the case-study system name used inside the documentation. It stands for **Quadcopter Flight Controller - 1** and refers to the specific firmware system being analyzed, designed, and later implemented through this repo.

## Project Intent

This repository is structured as both:

- An embedded firmware implementation.
- A system design case study for real-time control firmware.

The documentation is treated as a first-class artifact. Each design decision should be traceable from problem statement to requirements, architecture, module boundaries, tests, and final firmware behavior.

## Core Problem

A quadcopter airframe is physically unstable. Without continuous correction, disturbances such as gravity imbalance, vibration, and air movement can cause the frame to tilt and diverge.

QFC-1 answers one focused question:

How can firmware continuously measure roll and pitch, estimate the true attitude from noisy sensor data, compute correction demands, and apply those corrections to four motors fast enough to keep the airframe stable?

## Scope

In scope:

- MPU6050 IMU startup and validation.
- Gyroscope bias calibration.
- Accelerometer and gyroscope unit conversion.
- Roll and pitch estimation using a complementary filter.
- Roll and pitch PID control.
- Quad-X motor mixing.
- PWM output through a hardware abstraction layer.
- DISARMED, ARMED, and FAULT system states.
- Deterministic 1 ms control loop.
- Bench validation and unit testing.

Out of scope:

- Actual flight validation.
- RC receiver input.
- Yaw control.
- Altitude hold.
- GPS, barometer, magnetometer, telemetry, or logging.
- RTOS scheduling.
- Runtime dynamic memory allocation.

## System Design Focus

The project emphasizes the following embedded system design skills:

- Defining system boundaries and external actors.
- Translating use cases into firmware responsibilities.
- Designing safety behavior before control behavior.
- Separating hardware-independent logic from HAL-dependent code.
- Building deterministic runtime flow around a hardware timer tick.
- Decomposing firmware into testable modules.
- Using TDD for pure logic and controlled seams for hardware-facing modules.

## Initial Repository Structure

```text
documents/
  01-problem-statement.md
  02-requirements.md
  02.1-requirement-extraction.md
  03-system-context.md
  03.1-system-context-diagram.md
  04-use-cases-and-responsibilities.md
  04.1-use-case-diagram.md
  05-design-approach.md
  06-candidate-objects-vs-functional-modules.md
firmware/
  include/
  src/
  hal/
tests/
tools/
```

## Documentation

- [Problem Statement](documents/01-problem-statement.md)
- [Requirements](documents/02-requirements.md)
- [Requirement Extraction Notes](documents/02.1-requirement-extraction.md)
- [System Context](documents/03-system-context.md)
- [System Context Diagram](documents/03.1-system-context-diagram.md)
- [Use Cases And Responsibilities](documents/04-use-cases-and-responsibilities.md)
- [Use Case Diagram](documents/04.1-use-case-diagram.md)
- [Design Approach](documents/05-design-approach.md)
- [Candidate Objects Vs Functional Modules](documents/06-candidate-objects-vs-functional-modules.md)

Planned design-document flow:

- `07-object-model.md`
- `08-crc-cards.md`
- `09-object-model-decomposition.md`
- `10-event-model.md`
- `11-runtime-view.md`

## Firmware Architecture Snapshot

The control pipeline is expected to run once per 1 ms tick:

```text
IMU read
  -> unit conversion
  -> attitude estimation
  -> setpoint comparison
  -> PID control
  -> motor mixing
  -> command clamping
  -> PWM output
```

Motor outputs are suppressed unless the system is explicitly in the ARMED state. FAULT is a one-way safety state that forces all motor outputs to zero until hardware reset.

## Development Status

Status: early design phase in progress.

Current focus:

1. Finalize the design-document flow through system context, use cases, responsibilities, object thinking, event modeling, and runtime view.
2. Keep the documentation aligned with the broader goal of learning and explaining embedded system design methods through the QFC-1 case study.
3. Use the completed design set as the basis for later implementation planning and coding.

## Definition Of Done

The project is complete when bench validation demonstrates that:

- IMU identity validation passes on startup.
- Gyro bias calibration completes.
- Roll and pitch estimates respond correctly when the board is tilted by hand.
- PID outputs respond to angle error.
- Four PWM channels update every 1 ms.
- Simulated faults zero all motor outputs and lock the system in FAULT.
- DISARMED and FAULT never produce motor output.
