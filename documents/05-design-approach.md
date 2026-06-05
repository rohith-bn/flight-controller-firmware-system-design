# Design Approach

## Purpose Of This Document

This document examines the major design approaches that could be used to structure QFC-1 and explains which combination is most appropriate for this project. Its role is not to define the final object model or final module boundaries yet. Instead, it provides the reasoning that will guide those later structural decisions.

## Why This Follows Use Cases And Responsibilities

System context defined the world around QFC-1. Use cases and responsibilities defined how the system behaves and what obligations it must own. The next design question is structural:

How should those responsibilities be organized so that the system is understandable, implementable, testable, and compatible with its real-time and safety constraints?

This is where design approach matters. Different design lenses emphasize different aspects of the same system.

## The Design Question

QFC-1 is not just any software problem. It is:

- an embedded firmware problem
- a real-time control problem
- a safety-sensitive problem
- a bounded learning case study

That means the design approach cannot be chosen only for elegance or familiarity. It must support:

- deterministic execution
- clear responsibility ownership
- safe state handling
- hardware abstraction
- later implementation and testing

## Candidate Design Approaches

The most relevant design approaches for QFC-1 are:

- Functional decomposition
- Object-oriented analysis and design
- Event-driven design
- State-oriented design
- Runtime-oriented design
- Module-oriented design

Each of these looks at the same system from a different angle.

## Functional Decomposition

Functional decomposition structures the system around what it does.

For QFC-1, that naturally suggests a pipeline such as:

- sense
- convert
- estimate
- control
- mix
- actuate
- fault-handle

### Why it fits

- The stabilization loop is naturally sequential.
- The control path is easy to describe as a flow of transformations.
- It maps well to a 1 ms deterministic cycle.
- It is intuitive for embedded firmware and control systems.

### Limits if used alone

- It can become overly procedural.
- Responsibility boundaries may become weak if everything is treated as one long flow.
- Safety and mode behavior may not stand out strongly enough.

### Practical value for QFC-1

Functional decomposition is very useful for understanding the control pipeline and later for runtime modeling. It should definitely be part of the design lens, but it is not enough on its own.

## Object-Oriented Analysis And Design

OOAD structures the system around cooperating elements with defined responsibilities and collaborators.

For QFC-1, likely candidates might include elements such as:

- IMU driver
- gyro calibrator
- attitude estimator
- PID controller
- motor mixer
- PWM driver
- state manager
- flight controller

### Why it fits

- Responsibilities from the previous document can be assigned clearly.
- It improves communication and reasoning about who owns what.
- It works well with CRC cards and object-model exploration.
- It helps separate coordination from detailed logic.

### Limits if used alone

- It can become artificial if every behavior is forced into class-like structure.
- Embedded systems do not automatically benefit from deep object hierarchies.
- Timing behavior and event flow can become less visible if the design becomes too static.

### Practical value for QFC-1

OOAD is valuable as a thinking tool for assigning responsibility and clarifying collaboration. It is especially useful in the next few docs, but it should remain grounded in the real constraints of embedded firmware.

## Event-Driven Design

Event-driven design structures the system around triggers and reactions.

For QFC-1, important events include:

- power on
- periodic 1 ms tick
- arm command
- disarm command
- fault detection

### Why it fits

- The system is clearly driven by discrete events.
- The control loop is activated by a periodic trigger rather than free-running computation.
- Fault behavior is event-sensitive and must interrupt normal flow cleanly.

### Limits if used alone

- It does not by itself define good internal responsibility boundaries.
- It can describe when things happen without explaining how functionality should be packaged.

### Practical value for QFC-1

Event-driven thinking is essential for later event modeling and runtime analysis. It is a strong behavioral lens, but not a complete structural approach by itself.

## State-Oriented Design

State-oriented design emphasizes system modes and valid transitions between them.

For QFC-1, the safety-related states are explicit:

- `DISARMED`
- `ARMED`
- `FAULT`

### Why it fits

- Safety behavior depends strongly on state.
- Output rules differ by operating mode.
- `FAULT` is intentionally one-way.
- Human actions such as arm and disarm are state transitions more than data-processing steps.

### Limits if used alone

- It explains mode behavior well, but not the full internal data-processing pipeline.
- It is not enough to define estimation, control, and actuation responsibilities.

### Practical value for QFC-1

State-oriented design is essential for safety behavior and later runtime/state-machine analysis. It should be one of the central lenses in the final design.

## Runtime-Oriented Design

Runtime-oriented design focuses on execution timing, sequencing, and bounded behavior over time.

For QFC-1, this means asking:

- what happens at startup?
- what happens on every 1 ms tick?
- what happens when a fault occurs?
- what must never block?

### Why it fits

- QFC-1 is heavily constrained by timing.
- The 1 ms budget is architecturally important.
- Execution phases such as startup, steady-state, and fault handling differ significantly.
- Bounded runtime behavior matters as much as functional correctness.

### Limits if used alone

- It says a lot about execution behavior, but less about static design structure.
- It must be combined with other views to define modules or objects cleanly.

### Practical value for QFC-1

Runtime-oriented thinking is critical. It should shape the later event model and runtime-view documents and also influence how implementation boundaries are chosen.

## Module-Oriented Design

Module-oriented design organizes the system into implementation-ready units with stable boundaries and interfaces.

For QFC-1, examples might include:

- imu driver
- imu converter
- gyro calibrator
- attitude estimator
- pid controller
- motor mixer
- pwm driver
- state manager

### Why it fits

- It is directly useful for implementation planning.
- It supports testing and replacement of individual units.
- It helps separate hardware-dependent and hardware-independent logic.
- It aligns well with embedded firmware development practice.

### Limits if used alone

- If introduced too early, it can become guesswork.
- Module boundaries are more meaningful after behavior and responsibilities are understood.

### Practical value for QFC-1

Module-oriented design is the bridge to implementation. It should be informed by the earlier approaches rather than replacing them.

## Comparative Evaluation

Each approach contributes something different to QFC-1:

- **Functional decomposition** is best for understanding the control pipeline.
- **OOAD** is best for responsibility ownership and collaboration thinking.
- **Event-driven design** is best for trigger-based behavior.
- **State-oriented design** is best for safety modes and valid transitions.
- **Runtime-oriented design** is best for timing and execution behavior.
- **Module-oriented design** is best for implementation and testing boundaries.

No single approach captures the full system well enough on its own.

## Chosen Design Approach For QFC-1

QFC-1 should use a hybrid design approach.

The intended combination is:

- **Functional decomposition** to understand the stabilization pipeline
- **State-oriented design** to model safety behavior
- **Event-driven thinking** to model triggers such as startup, tick, arm, disarm, and fault
- **OOAD** to explore responsibility ownership and collaboration
- **Runtime-oriented thinking** to preserve deterministic timing
- **Module-oriented design** to translate the model into implementation-ready structure

This hybrid is not complexity for its own sake. It reflects the fact that QFC-1 has multiple design pressures:

- it must process data
- it must respond to events
- it must enforce state-dependent safety behavior
- it must run within timing constraints
- it must eventually be coded and tested in a practical way

## Why A Single Lens Is Not Enough

If QFC-1 were viewed only as a functional pipeline, safety and state logic could become secondary.

If it were viewed only through OOAD, execution timing and event flow could become less obvious.

If it were viewed only through states, internal data processing would remain underexplained.

If it were viewed only as modules, those module boundaries might be chosen too early and too mechanically.

The project therefore benefits from using multiple design lenses in sequence rather than forcing one viewpoint to explain everything.

## What This Enables Next

With the overall design approach now chosen, the next step becomes more focused:

- compare candidate objects with functional modules
- decide which responsibilities are best expressed as collaborators and which are best expressed as implementation units
- begin shaping the static internal structure of QFC-1

This document does not finalize the structure. It clarifies how the structure will be chosen.
