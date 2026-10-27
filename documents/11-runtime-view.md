# Runtime View

## Purpose Of This Document

This document defines the Runtime View for QFC-1. It selects the software execution model, applies classical behavioral design patterns in pure ISO C99, and specifies the static runtime data structures that govern execution across the 1 ms hard real-time cycle.

The goal is to define how code executes in time and memory without being coupled to a specific RTOS or silicon peripheral register map.

## Why This Follows The Event Model

The previous document, [System Event Model](10-event-model.md), identified what stimuli trigger system behavior. The Runtime View answers the operational engineering questions:
- What execution environment schedules these events?
- How is the tight 1 ms (1000 Hz) deadline guaranteed?
- How are state transitions, multi-axis control, and fault notifications handled in code?
- How is memory allocated and accessed safely without dynamic heap allocation?

---

## Event Characteristics Analysis

Before choosing an execution architecture, system events are evaluated across four runtime criteria:

| Metric | System Characteristic | Architectural Implication |
| :--- | :--- | :--- |
| **Frequency** | Exactly one dominant high-rate event (`TMR-01` at 1000 Hz). Operator commands and faults operate near 0 Hz. | The system does not require multi-rate scheduling; a single fixed-rate cycle governs all critical behavior. |
| **Timing** | Hard real-time deadline: Sense, estimate, control, mix, and actuate must complete within $< 1000 \mu\text{s}$. | Zero tolerance for scheduler preemption latency, thread context switching overhead, or blocking I/O. |
| **Interrupt Sensitivity**| The timer tick must be captured reliably without drift. | The timer ISR must be minimal (asserting a flag and returning in nanoseconds). |
| **Concurrency** | The 5 pipeline stages are strictly sequential: $A \to B \to C \to D \to E$. | **Zero concurrency requirement**. Stages never execute in parallel; queues add no value. |

---

## Runtime Execution Pattern Selection

### Evaluated Alternatives
- **Run/Reset Architecture**: *Rejected*. Destroys integrator state, complementary filter memory, and system safety state across iterations.
- **Event/Message Queue Dispatcher**: *Rejected*. Event queues introduce heap/ring-buffer overhead, potential queue-overflow failure modes, and non-deterministic latency. Events are consumed in the exact millisecond they are produced.
- **RTOS Preemptive Multitasking (FreeRTOS)**: *Rejected*. Context switching overhead, multiple stack allocations in RAM ($512$–$2048$ bytes per task), and mutex contention steal valuable execution budget from a purely sequential control pipeline.
- **Cooperative Fibers / Coroutines**: *Rejected*. Explicit yielding adds state machine bloat without performance gain.

### Selected Pattern: ISR-Triggered Deterministic Super Loop with Inline State Machine

```text
[Hardware Timer (1000 Hz)]
           │
           │ (Fires every 1 ms)
           v
  +-----------------+
  |    Timer ISR    |  --> Executes in < 10 CPU cycles:
  |                 |      g_tick_flag = true;
  +-----------------+
           │
           v
  +-----------------+
  |   Super Loop    |  --> Polled in main():
  |     main()      |      if (g_tick_flag) {
  +-----------------+         g_tick_flag = false;
           │                  run_1ms_pipeline();
           │               }
           v
  [Sequential Pipeline Execution]
  IMUDriver -> AttitudeEstimator -> PIDController -> MotorMixer -> SafetyGate -> PWMDriver
```

### Architectural Benefits
1. **Absolute Determinism**: The timer tick is never missed; pipeline execution starts within nanoseconds of the tick flag assertion.
2. **Minimal Interrupt Footprint**: The ISR executes in $< 10$ clock cycles, eliminating interrupt nesting hazards and priority inversion.
3. **Single Shared Stack**: Eliminates multiple thread stacks; entire firmware runs comfortably in $< 2 \text{ KB}$ of SRAM.
4. **Portability**: Runs identically on ARM Cortex-M4 (STM32), RISC-V (ESP32-C3), or a desktop host test harness.

---

## Behavioral Design Patterns Applied in Pure C

To achieve modularity and prevent sprawling `switch` or `if/else` ladders, three classical design patterns are implemented in ISO C99:

### Pattern 1: State Pattern (System Safety Lifecycle)
- **Applied to**: `SystemStateManager`.
- **Problem**: Gating motor output, handling commands, and enforcing safety with sprawling `switch` statements across multiple files leads to edge-case bugs and race conditions.
- **Solution**: Encapsulate state-specific behavior in dedicated handler sets:
  | Event | `DISARMED` State | `ARMED` State | `FAULT` State |
  | :--- | :--- | :--- | :--- |
  | `ArmCommand` | Transition to `ARMED` | Ignored (already armed) | Rejected (cannot leave FAULT) |
  | `DisarmCommand`| Ignored (already disarmed)| Transition to `DISARMED` | Rejected (cannot leave FAULT) |
  | `FaultDetected`| Transition to `FAULT` | Transition to `FAULT` & kill PWM | Ignored (already latched) |
  | `MotorGate` | Return `0.0` (suppress) | Pass mixed motor commands | Return `0.0` (suppress) |

### Pattern 2: Strategy Pattern (Dual-Axis PID Control)
- **Applied to**: `PIDController` (`RollPID` and `PitchPID`).
- **Problem**: Roll and pitch axes execute the exact same mathematical PID control law but require completely independent state storage (integrator accumulator, previous error) and independent gain tuning ($K_p, K_i, K_d$).
- **Solution**: Encapsulate the PID algorithm in a single, reusable function that operates on an axis-specific context structure:
  ```c
  typedef struct {
      float kp;
      float ki;
      float kd;
      float integral;
      float prev_error;
      float windup_limit;
  } pid_axis_t;

  float pid_axis_compute(pid_axis_t *axis, float error, float dt);
  ```
- **Benefits**: Zero code duplication. Independent tuning and testing for Roll and Pitch with zero mocks.

### Pattern 3: Observer / Callback Pattern (Fault Notification Pathway)
- **Applied to**: Fault notification routing.
- **Problem**: Multiple modules (`imu_comm`, `attitude_estimator`, `motor_mixer`) can discover catastrophic conditions. Direct hardcoded coupling from each module to `SystemStateManager` creates tight circular dependencies.
- **Solution**: A decoupled callback interface where `SystemStateManager` registers its fault ingestion handler as a callback function pointer:
  ```c
  typedef void (*fault_callback_t)(fault_code_t code);
  ```
- **Benefits**: Many-to-one decoupled fault routing. Fault sources do not need to know the internal implementation of the state manager.

---

## Runtime Data Structures and Memory Budget

All data structures are allocated statically at compile time in `.data` or `.bss` sections. **Zero dynamic heap allocation (`malloc`/`free`) is permitted.**

### Structure 1: Volatile Tick Flag
Shared between the hardware timer ISR and the main super loop:
```c
volatile bool g_tick_flag = false;
```
*Requirement*: Declared `volatile` so the compiler never optimizes reads into a CPU register.

### Structure 2: Static Structs for Module States
Persistent state across ticks is stored in static structures:
```c
/* Complementary Filter persistent state */
typedef struct {
    float roll_angle;
    float pitch_angle;
    float alpha;
    float dt;
} complementary_filter_t;

/* System State Manager persistent state */
typedef struct {
    system_state_t current_state;
    fault_callback_t fault_observer;
} system_state_manager_t;
```

### Structure 3: Fixed Motor Command Array
Commands are passed between `MotorMixer` and `PWMDriver` via fixed-size static arrays:
```c
float motor_commands[4];
/* Index mapping:
 * motor_commands[0] -> Front Left  (FL)
 * motor_commands[1] -> Front Right (FR)
 * motor_commands[2] -> Rear Left   (RL)
 * motor_commands[3] -> Rear Right  (RR)
 */
```

---

## Master Dispatch Execution Blueprint

```c
int main(void) {
    /* 1. HAL Silicon Initialization */
    hal_init();
    
    /* 2. Startup Orchestration */
    flight_controller_startup();
    
    /* 3. Enable 1ms Periodic Hardware Timer */
    hal_timer_start_1ms();

    /* 4. Super Loop */
    while (1) {
        if (g_tick_flag) {
            g_tick_flag = false;
            
            /* Execute 1ms Control Pipeline */
            imu_driver_read(&sensor_data);
            attitude_estimator_update(&sensor_data, &attitude);
            pid_controller_compute(&attitude, &setpoints, &corrections);
            motor_mixer_mix(&corrections, base_throttle, raw_motor_commands);
            
            /* Safety State Gate */
            system_state_manager_gate(raw_motor_commands, final_motor_commands);
            
            /* Actuation */
            pwm_driver_write(final_motor_commands);
        }
        
        /* Check asynchronous operator commands outside tick gate */
        handle_operator_commands();
    }
}
```

---

## Next Step

With runtime patterns, data structures, and dispatch sequence defined, the next document, [Module Catalog](12-module-catalog.md), provides the definitive technical specification for all 15 implementation modules.
