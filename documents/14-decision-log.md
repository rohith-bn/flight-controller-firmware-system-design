# Architectural Decision Log

## Purpose Of This Document

This document records the formal Architectural Decision Records (ADRs) for QFC-1. It documents the context, alternatives evaluated, technical trade-offs, and resulting consequences for all foundational design choices made across documents `01` through `13`.

In production engineering, the code explains *what* the system does; the ADR log explains *why* the system was built that way.

---

## Master ADR Registry

| ADR ID | Decision Title | Status | Primary Impact |
| :---: | :--- | :---: | :--- |
| **ADR-01** | Hybrid Architecture: OOAD for Reasoning, Modular C for Code | `ACCEPTED` | High-level modularity without C++ heap or vtable bloat |
| **ADR-02** | ISR-Triggered Super Loop over Preemptive RTOS | `ACCEPTED` | Hard real-time determinism within 1 ms execution budget |
| **ADR-03** | One-Way Latching FAULT Safety State (No Auto-Recovery) | `ACCEPTED` | Uncompromising physical airframe safety |
| **ADR-04** | Complementary Filter over Extended Kalman Filter (EKF) | `ACCEPTED` | Deterministic floating-point budget and intuitive tuning |
| **ADR-05** | Category A Pure Logic Isolation for Zero-Mock Host TDD | `ACCEPTED` | Eliminates hardware friction during daily development loop |
| **ADR-06** | Strategy Pattern for Dual-Axis PID Control | `ACCEPTED` | Zero code duplication with isolated roll/pitch state |
| **ADR-07** | Strict Static Allocation and Zero Dynamic Heap Memory | `ACCEPTED` | Eliminates memory fragmentation and heap exhaustion |

---

## Detailed Architectural Decision Records

### ADR-01: Hybrid Architecture: OOAD for Reasoning, Modular C for Code

- **Context**: Embedded systems must balance rigorous architectural abstraction with resource-constrained execution. Pure object-oriented languages (C++) often introduce runtime overhead (virtual table pointer lookups, dynamic memory allocation) that complicate real-time safety. Pure procedural C often degrades into tightly-coupled, untestable spaghetti code.
- **Decision**: Adopt a Hybrid Design Approach:
  1. Use Object-Oriented Analysis and Design (OOAD) and CRC cards during the conceptual design phase to allocate responsibilities, establish boundaries, and prevent a "God module."
  2. Implement the resulting objects as static, modular ISO C99 modules (`.c` + `.h`), using static structures for encapsulation.
- **Alternatives Considered**:
  - *Full C++ OOP*: Rejected due to potential runtime overhead, non-deterministic exception handling, and compiler variation across low-cost MCU toolchains.
  - *Ad-hoc Procedural C*: Rejected due to poor testability and high risk of circular dependencies.
- **Consequences**:
  - **Positive**: Clean architectural boundaries, high cohesion, zero runtime vtable overhead, 100% portable across standard C compilers.
  - **Negative**: Requires discipline to maintain object encapsulation manually using C idioms (opaque pointers, static structs).

---

### ADR-02: ISR-Triggered Super Loop over Preemptive RTOS

- **Context**: The firmware must execute a closed-loop control pipeline (Sense -> Estimate -> Control -> Mix -> Actuate) within a fixed 1 ms (1000 Hz) period.
- **Decision**: Reject FreeRTOS and implement an **ISR-Triggered Deterministic Super Loop with Inline State Machine**. A hardware timer ISR does one atomic operation: sets a `volatile bool g_tick_flag`. The main loop polls and clears this flag, executing the 5 pipeline stages sequentially.
- **Alternatives Considered**:
  - *Preemptive RTOS (FreeRTOS / Zephyr)*: Rejected because the pipeline is strictly sequential (stage $N$ requires the output of stage $N-1$). Context switching overhead, stack frame preservation, and scheduler jitter reduce the available computational budget without providing task isolation benefits.
  - *Event Queue Dispatcher*: Rejected because all events are consumed in the exact millisecond they are produced; queuing introduces queue-overflow risks and dynamic latency.
- **Consequences**:
  - **Positive**: Absolute execution determinism, zero context-switch latency, minimal interrupt footprint (< 10 CPU cycles), tiny RAM footprint (runs comfortably in < 2 KB SRAM).
  - **Negative**: Long-running background operations (e.g., future flash logging) must be carefully sliced so as not to block the 1 ms tick.

---

### ADR-03: One-Way Latching FAULT Safety State (No Auto-Recovery)

- **Context**: In real-time physical control systems with high-speed spinning propellers, sensor dropouts, mathematical divergence, or communication loss represent catastrophic risks.
- **Decision**: The `FAULT` state is designed as an **unconditional, one-way latching state**. The moment a fault is detected (I2C timeout, `WHO_AM_I` mismatch, angle $> \pm 90^\circ$, pre-clamp bounds violation):
  1. All 4 motor PWM outputs are instantly driven to $0.0$ within the exact same 1 ms cycle.
  2. The system transitions to `FAULT` and refuses all subsequent Arm commands.
  3. Recovery requires a physical power cycle or hardware reset button.
- **Alternatives Considered**:
  - *Automatic Soft Reboot / In-Flight Recovery*: Rejected. Attempting to recover and re-spin motors while an airframe is in an undefined or inverted attitude causes physical injury and hardware destruction.
- **Consequences**:
  - **Positive**: Total safety guarantee. Zero risk of runaway motors in an undefined state.
  - **Negative**: Any transient glitch shuts down the system (requires robust hardware wiring and noise filtering on bench).

---

### ADR-04: Complementary Filter over Extended Kalman Filter (EKF)

- **Context**: Sensor fusion must combine high-frequency angular rates from the gyroscope with long-term gravity vectors from the accelerometer.
- **Decision**: Implement a discrete-time **Complementary Filter** ($\alpha = 0.98, \Delta t = 0.001\text{s}$) rather than an Extended Kalman Filter (EKF).
- **Alternatives Considered**:
  - *Extended Kalman Filter (EKF)*: Rejected for baseline roll/pitch stabilization due to high computational complexity (matrix inversions, covariance propagation, trigonometric Jacobians) which consumes valuable CPU cycles.
  - *Madgwick / Mahony Filter*: Viable alternative, but complementary filter provides identical horizontal stabilization with fewer floating-point operations and single-parameter transparency.
- **Consequences**:
  - **Positive**: Low computational overhead (< 30 float operations), deterministic execution time, intuitive tuning via $\alpha$, easy to test on host.
  - **Negative**: Does not compute full 3D attitude quaternions or yaw heading (aligned with baseline scope).

---

### ADR-05: Category A Pure Logic Isolation for Zero-Mock Host TDD

- **Context**: Testing embedded software exclusively on physical target boards creates friction and slows development.
- **Decision**: Decompose all algorithmic modules so that mathematical logic is completely decoupled from hardware communication. Modules in Category A (`imu_converter`, `angle_computer`, `complementary_filter`, `pid_axis`, `motor_mixer`, `setpoint_manager`, `gyro_calibrator`) contain **zero HAL calls and zero hardware headers**.
- **Alternatives Considered**:
  - *Hardware-in-the-Loop Only Testing*: Rejected due to slow compile-flash-debug cycles.
  - *Mock-Heavy Host Testing*: Rejected because mocking hardware buses (I2C transactions) to test basic trigonometry makes unit tests brittle and bloated.
- **Consequences**:
  - **Positive**: 95% of firmware logic is tested on the host PC in milliseconds with zero mocks.
  - **Negative**: Requires strict discipline during decomposition to prevent HAL includes from leaking into math headers.

---

### ADR-06: Strategy Pattern for Dual-Axis PID Control

- **Context**: Roll and pitch axes require identical PID control laws, but maintain independent integrator states, previous errors, and gain configurations ($K_p, K_i, K_d$).
- **Decision**: Implement a reusable `pid_axis_t` Strategy struct and compute function. `PIDController` instantiates two independent instances (`roll_pid` and `pitch_pid`).
- **Alternatives Considered**:
  - *Duplicated Code (`roll_pid.c` and `pitch_pid.c`)*: Rejected due to maintenance violation.
  - *Monolithic Dual-Axis PID*: Rejected because coupling both axes into one function prevents isolated unit testing and independent axis tuning.
- **Consequences**:
  - **Positive**: Zero code duplication, independent tuning, full axis state isolation.
  - **Negative**: Requires passing pointer to axis context struct on every computation.

---

### ADR-07: Strict Static Allocation and Zero Dynamic Heap Memory

- **Context**: Embedded firmware running continuously for hours must guarantee that it will never run out of memory or suffer from heap fragmentation.
- **Decision**: Completely ban `malloc()`, `free()`, and dynamic runtime memory allocation. All structs, buffers, arrays, and variables must be statically allocated at compile time in `.data` or `.bss`.
- **Alternatives Considered**:
  - *Dynamic Allocation at Startup*: Rejected. Even one-time allocation requires linking the heap manager and risks runtime memory faults.
- **Consequences**:
  - **Positive**: Zero heap fragmentation, deterministic memory usage, compile-time memory footprint verification via `size` or linker map files.
  - **Negative**: Buffer sizes (e.g., calibration sample count $N=512$) are fixed at compile time.

---

## Conclusion of Phase 1 (System Design)

With the completion of documents `01` through `14`, the entire system design journey of QFC-1 is fully formalized. Every requirement is traced to an architectural responsibility, every responsibility is mapped to a modular C entity, every runtime interaction is budgeted, and every design trade-off is recorded.

The system is now completely specified and ready for **Phase 2: Host TDD Firmware Implementation**.
