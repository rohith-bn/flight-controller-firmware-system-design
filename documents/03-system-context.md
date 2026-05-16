# System Context

## Purpose Of This Document

This document defines the system context for QFC-1. It explains where the system begins and ends, which external actors interact with it, what crosses the system boundary, and which assumptions shape the environment in which the firmware operates.

## System Overview

QFC-1 is a learning-focused embedded firmware system for roll and pitch stabilization of a quadcopter airframe. It reads inertial measurements, estimates attitude, computes corrective control outputs, and drives four motor commands within a deterministic 1 ms control loop.

The purpose of this document is not to define internal module structure yet. Its job is to clarify the system as a bounded unit before moving into use cases, responsibilities, and implementation structure.

## System Boundary

### Inside The System Boundary

The QFC-1 firmware is responsible for:

- Reading accelerometer and gyroscope data through the HAL
- Validating IMU identity during startup
- Performing gyroscope bias calibration
- Converting raw sensor counts into engineering units
- Estimating roll and pitch angles
- Maintaining internal setpoints for bench validation
- Computing control corrections for roll and pitch
- Mixing control outputs into four motor commands
- Clamping motor commands before output
- Driving PWM outputs through the HAL
- Managing DISARMED, ARMED, and FAULT states
- Detecting faults and enforcing safe shutdown behavior
- Orchestrating the 1 ms control loop

### Outside The System Boundary

The following are external to QFC-1:

- IMU hardware (MPU6050)
- ESC and motor hardware
- HAL implementation
- MCU clock configuration
- Peripheral register configuration
- Physical airframe dynamics
- Operator actions such as power-on, reset, arm, and disarm
- Any behavior related to full aircraft flight beyond bench validation

## External Actors

### IMU Hardware

The MPU6050 provides raw accelerometer and gyroscope measurements. QFC-1 depends on it for sensing the physical state of the airframe.

### HAL

The hardware abstraction layer provides the trusted services used by the firmware to interact with hardware, including I2C access, PWM output, and the 1 ms timing trigger.

### ESC And Motor Assembly

The ESC and motor system receives PWM duty-cycle commands from QFC-1 and converts them into motor actuation.

### Physical Environment

Gravity, vibration, and angular disturbances affect the airframe and are reflected in IMU measurements. The firmware reacts to this environment but does not model or control the entire aircraft.

### Developer / Operator

The developer or operator powers the system, performs bench validation, and may issue arm or disarm actions. Human intervention is also required after a FAULT condition.

## Inputs And Outputs

### Inputs

- Raw accelerometer readings from the IMU
- Raw gyroscope readings from the IMU
- IMU identity response during startup
- 1 ms timing tick from the HAL
- Operator arm or disarm intent
- System startup or hardware reset

### Outputs

- PWM duty-cycle commands to four ESC channels
- Forced zero-output behavior in DISARMED state
- Forced zero-output behavior in FAULT state
- Internal fault state transition when unsafe conditions are detected

## Interaction Summary

QFC-1 reads sensor data from the IMU through the HAL, processes that data inside a fixed-time control loop, and sends motor commands outward through PWM channels. The physical environment continuously affects the sensed state of the airframe, while the operator controls startup and arming-related actions. The firmware itself remains bounded to stabilization logic and safety enforcement.

## Assumptions And Trust Boundaries

QFC-1 makes the following assumptions:

- The HAL is already initialized before firmware execution begins
- HAL services are correct, trusted, and sufficiently bounded for timing needs
- The IMU is available and connected as expected
- The board is stationary during startup calibration
- Bench validation is the intended operating context for this project
- All required control-loop work can be completed within the 1 ms execution window

The most important trust boundary is the HAL. QFC-1 depends on HAL correctness but does not reimplement or validate HAL internals.

## Explicit Exclusions

The following are intentionally excluded from the current system context:

- Yaw-axis stabilization
- Altitude estimation or hold
- GPS or navigation behavior
- Barometer or magnetometer integration
- RC receiver parsing
- Wireless telemetry
- CLI or logging infrastructure in the core firmware
- RTOS-based scheduling
- Runtime dynamic memory allocation
- Real flight validation

## What This Enables Next

With system context defined, the next design steps become clearer:

- Identify the major use cases that drive system behavior
- Derive functional responsibilities from those use cases
- Decide how responsibilities should be grouped into candidate objects or implementation modules
- Refine runtime behavior, especially around timing, safety, and fault handling

System context does not tell us how to design the internals in detail. It tells us what world the system lives in, what it owns, and what it depends on. That clarity is what makes the next design step meaningful.
