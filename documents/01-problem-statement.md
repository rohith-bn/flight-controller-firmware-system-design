# Problem Statement

## System Purpose

QFC-1 is a deterministic embedded firmware system for stabilizing a quadcopter airframe on the roll and pitch axes.

The firmware continuously:

- Acquires inertial measurements from an IMU.
- Estimates roll and pitch angles.
- Computes corrective control outputs.
- Mixes those corrections into four motor commands.
- Enforces safety rules before any motor output is produced.

The system is intentionally narrow. It is designed for bench validation and learning, not for autonomous flight.

## Problem

A quadcopter airframe is unstable by nature. If it is disturbed, it does not passively return to level. It needs fast, continuous correction.

The firmware problem is:

How can the system measure attitude, estimate the true roll and pitch angles from noisy sensor data, compute corrections, and update four motors inside a fixed 1 ms timing budget?

This problem is interesting because it combines:

- Sensor acquisition.
- Signal conversion.
- Sensor fusion.
- Feedback control.
- Motor command distribution.
- Safety-state enforcement.
- Real-time execution constraints.

## System Boundary

Inside the firmware boundary:

- IMU data acquisition through a HAL interface.
- MPU6050 identity validation.
- Gyroscope bias calibration.
- Raw sensor conversion into engineering units.
- Roll and pitch attitude estimation.
- Roll and pitch setpoint handling.
- PID control.
- Quad-X motor mixing.
- Motor command clamping.
- PWM output through a HAL interface.
- Safety state management.
- Fault detection and shutdown behavior.

Outside the firmware boundary:

- MCU clock and peripheral configuration.
- Low-level I2C driver implementation.
- Low-level PWM timer implementation.
- ESC and motor physics.
- Actual aircraft flight.
- Operator hardware reset.

## Actors

### Developer / Operator

Powers the system, observes bench behavior, and may issue arm or disarm commands during validation.

### IMU Hardware

Provides raw accelerometer and gyroscope readings. The initial target IMU is the MPU6050.

### HAL

Provides trusted hardware access services such as I2C reads/writes, PWM writes, and a 1 ms tick signal.

### ESC And Motor Assembly

Receives PWM commands and drives motor output. The firmware must never send unsafe motor commands.

### Physical Environment

Applies gravity, vibration, and angular disturbance to the airframe.

## Non-Goals

QFC-1 does not attempt to solve full drone flight. It does not include navigation, altitude hold, yaw control, RC input, wireless telemetry, or autonomous behavior.

The point of the project is to design and implement a focused real-time firmware subsystem well.
