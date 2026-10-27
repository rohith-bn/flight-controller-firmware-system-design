# TDD Test Strategy

## Purpose Of This Document

This document defines the Test-Driven Development (TDD) strategy and verification framework for QFC-1. It specifies the Dual-Targeting testing model, categorizes modules by test double requirements, and outlines the exact test cases and assertion bounds for host-based verification before flashing target silicon.

The goal is to prove software correctness, algorithmic precision, and safety interlocks with fast, automated test execution on the developer machine.

## Why This Follows The Module Catalog

The previous document, [Module Catalog](12-module-catalog.md), established strict module boundaries and dependencies. Testing is not an afterthought added after code is written; in embedded systems, testability dictates code structure.

By formalizing the test strategy before writing C code, we guarantee that every module has a clear testing seam and that pure mathematics can be tested on the host with **zero mocks**.

---

## Foundational Architecture: The Dual-Target Strategy

Testing embedded software exclusively on target hardware evaluation boards introduces friction: slow flashing cycles, complex debugging probes, and difficulty simulating edge-case sensor failures. QFC-1 addresses this through a **Dual-Target Strategy**:

```text
+--------------------------------------------------------------------------------+
|                         DUAL-TARGET VERIFICATION MODEL                         |
+-----------------------------------------------+--------------------------------+
|           HOST TEST SUITE (Fast Loop)         |   TARGET TEST SUITE (Slow Loop)|
+-----------------------------------------------+--------------------------------+
| - Runs on host developer machine (Windows/x64)| - Cross-compiled for ARM/ESP32 |
| - Native C compiled via GCC/Clang             | - Flashed to target evaluation |
| - Millisecond execution (< 100 ms total suite)|   board                        |
| - Fast Red-Green-Refactor TDD microcycle      | - Validates hardware timing,   |
| - Catches 95% of logic, math, and state bugs  |   endianness, FPU precision,   |
| - Uses test doubles for hardware boundaries   |   and actual peripheral silicon|
+-----------------------------------------------+--------------------------------+
```

---

## The Three Module Categories

Every module in QFC-1 is assigned to one of three test categories, dictating its test double requirements:

```text
[Category A: Pure Logic]       --> Test on Host with ZERO MOCKS
  (imu_converter, angle_computer, complementary_filter, roll_pid, pitch_pid, motor_mixer, setpoint_manager, gyro_calibrator)

[Category B: Orchestration]    --> Test on Host with LIGHTWEIGHT STUBS & SPIES
  (imu_driver, attitude_estimator, pid_controller, system_state_manager, flight_controller)

[Category C: Hardware Boundary] --> Test on Host with IN-MEMORY FAKES; Verify on Target
  (imu_comm, pwm_driver)
```

---

## TDD Microcycle Protocol (Red - Green - Refactor)

Every line of production C code is developed using the strict three-phase TDD microcycle:

1. **RED**: Write a failing unit test asserting a single requirement or mathematical equation. Compile and run on host. *The test must fail for the expected reason*.
2. **GREEN**: Write the minimal production C code necessary to satisfy the test. *No speculative features, wrappers, or unused helpers*.
3. **REFACTOR**: Clean up naming, remove duplication, and verify MISRA-C compliance. *All unit tests must remain green*.

---

## Per-Module Test Scope and Test Double Matrix

### 1. Category A: Pure Logic Modules (Zero Mocks)

#### `imu_converter` (Stateless Pure Math)
- **Scope**:
  - Raw count $0 \implies 0.0 \text{ m/s}^2$ and $0.0 \text{ deg/s}$.
  - Full scale acceleration: $\pm 16384 \text{ LSB} \implies \pm 9.807 \text{ m/s}^2$ ($\pm 1g$).
  - Full scale rotation: $\pm 131 \text{ LSB} \implies \pm 1.0 \text{ deg/s}$.
  - Signed two's complement boundary values (maximum positive `0x7FFF`, maximum negative `0x8000`).
- **Test Doubles**: None.
- **Host Strategy**: Assert outputs within float epsilon tolerance ($\pm 0.001$). Use 0-1-N test patterns.

#### `angle_computer` (Stateless Trigonometry)
- **Scope**:
  - Board flat ($a_x = 0, a_y = 0, a_z = 1g$) $\implies \text{Roll} = 0.0^\circ, \text{Pitch} = 0.0^\circ$.
  - Pure roll tilt: $90^\circ$ roll ($a_x = 0, a_y = 1g, a_z = 0$) $\implies \text{Roll} = 90.0^\circ$.
  - Pure pitch tilt: $45^\circ$ pitch $\implies \text{Pitch} = 45.0^\circ$.
  - Symmetry verification: $+45^\circ$ and $-45^\circ$ yield equal magnitude with opposite sign.
- **Test Doubles**: None. Pure call to `atan2f`.
- **Host Strategy**: Feed known acceleration vectors; assert angle output within $\pm 0.1^\circ$.

#### `complementary_filter` (Stateful Mathematical Fusion)
- **Scope**:
  - Post-init: angles initialize to $0.0^\circ$.
  - Pure gyroscope rate over $N$ ticks with $\alpha = 1.0 \implies$ pure integration (drift test).
  - Pure accelerometer angle with $\alpha = 0.0 \implies$ instantaneous tracking.
  - Standard tuning ($\alpha = 0.98$): dynamic disturbance rejected; converges to true steady-state angle over 500–1000 ticks.
- **Test Doubles**: None. Inputs are plain floats.
- **Host Strategy**: Simulate $N$ ticks of synthetic gyro and accelerometer waveforms in a tight loop.

#### `roll_pid` / `pitch_pid` (Stateful Control Law)
- **Scope**:
  - Proportional term: zero error $\implies P = 0$; positive error $\implies P > 0$; scales linearly with $K_p$.
  - Integral term: constant error accumulates over time at rate $K_i \cdot e \cdot \Delta t$; stops accumulating when hitting `windup_limit`.
  - Derivative term: constant error $\implies D = 0$; rapid ramp $\implies D$ resists change.
  - Axis isolation: exercising `roll_pid` has zero effect on `pitch_pid` state.
- **Test Doubles**: None.
- **Host Strategy**: Execute isolated step-response inputs; verify individual P, I, D mathematical terms.

#### `motor_mixer` (Quad-X Geometry & Clamping)
- **Scope**:
  - Pure throttle: base throttle $T = 0.5$, zero roll/pitch $\implies M_1 = M_2 = M_3 = M_4 = 0.5$.
  - Pure roll: positive roll demand increases left motors ($M_1, M_3$) and decreases right motors ($M_2, M_4$).
  - Pure pitch: positive pitch demand increases rear motors ($M_3, M_4$) and decreases front motors ($M_1, M_2$).
  - Clamping: demands exceeding `MOTOR_MAX` ($1.0$) are clamped to $1.0$; demands below `MOTOR_MIN` ($0.0$) are clamped to $0.0$.
  - Bounds violation: severe out-of-bounds demands trigger a fault callback before clamping.
- **Test Doubles**: Test Spy on fault callback.
- **Host Strategy**: Boundary value analysis testing inputs at limit, one below, and one above.

---

### 2. Category B: Orchestration Modules (Stubs & Spies)

#### `system_state_manager` (Safety Critical State Machine)
- **Scope**:
  - Initial state on boot is `DISARMED`.
  - Valid transitions: `DISARMED` + `ArmCommand` $\implies \text{ARMED}$; `ARMED` + `DisarmCommand` $\implies \text{DISARMED}$.
  - Fault handling: Any state + `FaultSignal` $\implies \text{FAULT}$.
  - One-way latch: In `FAULT`, `ArmCommand` and `DisarmCommand` are strictly rejected.
  - Output gating: Motor commands are zeroed in `DISARMED` and `FAULT`.
- **Test Doubles**:
  - Test Spy on `pwm_driver_zero_all()` to assert immediate execution on fault.
  - Test Dummy for fault signal generation.
- **Host Strategy**: Exhaustive state transition table testing.

#### `attitude_estimator` & `pid_controller` (Coordinators)
- **Scope**:
  - Correct delegation to underlying sub-modules.
  - Accurate data propagation from input struct to output struct.
  - Divergence monitoring ($> \pm 90^\circ$ fires fault to `SystemStateManager`).
- **Test Doubles**:
  - Link-time stubs for upstream data providers (`imu_driver`, `setpoint_manager`).
  - Real instances of pure math sub-modules (`complementary_filter`, `pid_axis`).

---

### 3. Category C: Hardware Boundary Modules (Fakes & Target Testing)

#### `imu_comm` (I2C Protocol Seam)
- **Scope**:
  - Correct I2C slave address (`0x68`) and register addresses (`0x3B`–`0x48`, `0x75`) requested.
  - `WHO_AM_I` returns `0x68`.
  - HAL timeout or NACK errors are caught and converted to error codes.
- **Test Doubles**:
  - **Test Fake**: Lightweight in-memory array simulating MPU6050 hardware registers on the host.
- **Target Strategy**: Read live MPU6050 hardware on evaluation board; verify stable sensor readings.

#### `pwm_driver` (Timer Compare Seam)
- **Scope**:
  - Throttle $0.0 \implies$ minimum pulse count ($1000 \mu\text{s}$).
  - Throttle $1.0 \implies$ maximum pulse count ($2000 \mu\text{s}$).
  - `pwm_driver_zero_all()` writes $0$ counts to all 4 channels immediately.
- **Test Doubles**:
  - **Test Spy**: Records channel IDs and duty cycle counts passed to HAL.
- **Target Strategy**: Connect 4 oscilloscope/logic analyzer channels to PWM output pins; verify $1000 \text{ Hz}$ update frequency and exact pulse widths.

---

## Next Step

With testing strategy and verification seams established, the final pre-implementation document, [Decision Log](14-decision-log.md), documents the formal Architectural Decision Records (ADRs) that explain and justify every core trade-off made across the project.
