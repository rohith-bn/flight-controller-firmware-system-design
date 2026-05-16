# Problem Statement

## Purpose Of This Document

This document defines the problem QFC-1 exists to solve. It focuses on why the system is needed, what outcome matters, and what scope is intentionally chosen for this learning-focused project.

## System Purpose

QFC-1 is a deterministic embedded firmware system for stabilizing a quadcopter airframe on the roll and pitch axes.

The firmware continuously:

- Acquires inertial measurements from an IMU.
- Estimates roll and pitch angles.
- Computes corrective control outputs.
- Mixes those corrections into four motor commands.
- Enforces safety rules before any motor output is produced.

The system is intentionally narrow. It is designed for bench validation and learning, not for autonomous flight.

## Problem Definition

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

## Definition Of Success

At a high level, this project is successful if the system can:

- Keep the design focused on roll and pitch stabilization rather than full drone flight.
- Define a clear, deterministic control problem that can be implemented within a 1 ms execution cycle.
- Establish a safety-first firmware concept in which unsafe conditions never result in uncontrolled motor output.
- Provide enough clarity to move into system context, use cases, responsibilities, and implementation without inventing the system blindly.

## Intended Scope

This project is intentionally scoped as a learning-oriented embedded firmware case study.

Within that scope, the system is intended to:

- Estimate roll and pitch from IMU data.
- Compute corrective control outputs.
- Translate those corrections into four motor commands.
- Enforce safe operating behavior through explicit system states.
- Be validated on the bench rather than through real flight.

The project is not intended to solve the entire drone problem. It is meant to isolate one meaningful embedded control problem and design it well.

## Non-Goals

QFC-1 does not attempt to solve full drone flight. It does not include navigation, altitude hold, yaw control, RC input, wireless telemetry, or autonomous behavior. It also does not attempt to explore every possible control or estimation strategy. The point is to design and implement one focused real-time firmware subsystem well.

The point of the project is to design and implement a focused real-time firmware subsystem well.
