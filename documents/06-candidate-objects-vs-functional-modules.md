# Candidate Objects Vs Functional Modules

## Purpose Of This Document

This document examines how the responsibilities identified for QFC-1 might be expressed structurally and explores a key design question in embedded systems:

Should the system be thought of primarily in terms of objects, functional modules, or a combination of both?

The goal is not yet to freeze the final structure. The goal is to understand how responsibility ownership at the design level can become practical implementation structure later.

## Why This Follows Design Approach

The previous document established that QFC-1 should use a hybrid design approach rather than forcing one lens to explain everything. That still leaves an important structural question open:

When moving from responsibilities toward implementation, how should those responsibilities be packaged?

This is where many embedded designers hesitate. The design may be object-oriented in reasoning, but the implementation may still be simpler and stronger when expressed as modules. This document is where that tension is examined directly.

## The Structural Design Question

At this stage, several questions naturally arise:

- Should every major responsibility become an object?
- Should the system simply be decomposed into C-style modules?
- Is OOAD actually useful for embedded firmware?
- How do responsibilities become code structure without becoming artificial?

These are not academic questions. They directly affect:

- clarity of design
- ease of implementation
- testability
- hardware abstraction
- maintainability
- suitability for a real-time embedded environment

## Two Different Kinds Of Structure

Before comparing objects and modules, it helps to distinguish between two different structural views.

### Conceptual Structure

Conceptual structure is how we reason about the system.

It answers questions such as:

- Who owns this responsibility?
- Which element collaborates with which?
- What are the major conceptual parts of the system?

This view is useful during analysis and design because it helps clarify responsibility, collaboration, and meaning.

### Implementation Structure

Implementation structure is how the code is actually organized.

It answers questions such as:

- Which source files should exist?
- Which interfaces should be public?
- Which code depends on the HAL?
- Which units should be independently testable?

This view is useful during coding and testing because it helps define practical boundaries for firmware implementation.

### Why This Distinction Matters

A conceptually rich object model does not always imply an object-heavy implementation.

In embedded systems, it is common for:

- the conceptual model to be more expressive
- the actual code structure to be more module-oriented

That is not a contradiction. It is often the most practical outcome.

## What Object Thinking Offers

Object thinking asks:

- what are the major collaborating elements?
- what responsibility belongs to each one?
- what should each element know or control?

Applied to QFC-1, object thinking naturally suggests elements such as:

- `IMUDriver`
- `GyroCalibrator`
- `AttitudeEstimator`
- `PIDController`
- `MotorMixer`
- `PWMDriver`
- `SystemStateManager`
- `FlightController`

### Why This Is Useful

Object thinking helps because:

- responsibilities are easier to assign
- collaborations become easier to discuss
- CRC analysis becomes possible
- the system becomes easier to explain at the design level

For example, it is easier to ask:

> Should attitude estimation own angle fusion responsibility?

than to ask:

> Which random utility function file should hold this behavior?

That is a real advantage during design.

## What Module Thinking Offers

Module thinking asks:

- what implementation units should exist?
- what code should change together?
- what interface boundaries improve testing and maintainability?

Applied to QFC-1, likely modules might include:

- `imu_driver`
- `imu_converter`
- `gyro_calibrator`
- `attitude_estimator`
- `pid_controller`
- `motor_mixer`
- `pwm_driver`
- `system_state_manager`
- `flight_controller`

### Why This Is Useful

Module thinking helps because:

- it aligns well with embedded firmware practice
- it produces buildable, testable code units
- it naturally separates hardware-facing and hardware-independent logic
- it avoids forcing unnecessary object machinery into implementation

For a C-based firmware project, modules are often the most natural implementation units.

## So Should Everything Become An Object?

No.

That would usually be a mistake for a project like QFC-1.

### Why not?

- Not every responsibility has strong conceptual identity.
- Some responsibilities are really service-like or conversion-like.
- Some hardware-facing code is better expressed as a straightforward module boundary.
- Overusing object-style structure can create unnecessary indirection.

For example:

- `AttitudeEstimator` has a strong conceptual identity.
- `IMUConverter` may be better viewed as a functional/helper-oriented unit.
- `PWMDriver` is often more naturally a hardware-facing module than a rich conceptual object.

So object thinking is valuable, but object inflation is not.

## So Should Everything Just Be A C Module?

Also no.

That would be simpler in one sense, but it can weaken the design reasoning.

### Why not?

- Responsibilities become easier to blur.
- Collaboration between conceptual parts becomes less visible.
- Design explanation becomes less disciplined.
- CRC-style reasoning becomes harder to apply.

If everything is only seen as a module from the start, the system can drift toward:

- implementation-first structure
- weaker conceptual ownership
- less thoughtful responsibility allocation

That would reduce some of the learning value of this project.

## Is OOAD Useful For Embedded Firmware?

Yes, but not in the naive sense of "everything must be a class."

OOAD is useful in QFC-1 because it helps answer:

- who owns estimation?
- who owns state behavior?
- who owns control logic?
- who coordinates the overall loop?

That kind of thinking is valuable even if the final implementation is mostly modular C code.

So the right conclusion is not:

- OOAD everywhere

or

- OOAD never

The better conclusion is:

- use OOAD to reason about responsibility and collaboration
- use implementation modules to express that reasoning practically

## Candidate Structural Elements In QFC-1

From the current responsibilities, the major structural candidates are:

- sensor access
- startup validation
- calibration
- signal conversion
- attitude estimation
- setpoint handling
- PID control
- motor mixing
- PWM output
- state management
- fault detection and response
- loop orchestration

The next question is which of these are best viewed conceptually as objects and which are better expressed practically as modules.

## Responsibilities That Naturally Suggest Objects

Some responsibilities have a strong conceptual center and clear collaboration role.

These are good object candidates:

- `AttitudeEstimator`
- `PIDController`
- `MotorMixer`
- `SystemStateManager`
- `FlightController`

### Why these fit object thinking

- Each owns a coherent responsibility.
- Each collaborates meaningfully with other parts.
- Each has an identity beyond just “a file with functions.”
- Each can be reasoned about in CRC-style analysis.

For example:

- the estimator collaborates with sensing and control
- the controller collaborates with setpoints and actuation
- the state manager collaborates with all output-enabling logic

That makes them strong conceptual objects.

## Responsibilities That Naturally Suggest Modules

Some responsibilities are better expressed as implementation modules or service-like units.

These are good module candidates:

- `imu_driver`
- `imu_converter`
- `pwm_driver`
- hardware abstraction adapters
- low-level validation helpers

### Why these fit module thinking

- They are closely tied to interfaces and implementation concerns.
- They are often hardware-facing or transformation-oriented.
- Their value comes more from stable boundaries than rich conceptual identity.

For example:

- `imu_driver` is largely about hardware interaction
- `pwm_driver` is about output integration
- `imu_converter` is about deterministic transformation of raw values

These do not need to be over-romanticized into richer objects than they really are.

## A Hybrid Structural Direction For QFC-1

The strongest direction for QFC-1 is a hybrid:

- use object-oriented thinking at the conceptual level
- use module-oriented structure at the implementation level

In practice, that means:

- reason about the system through named collaborating conceptual elements
- implement those ideas through practical firmware modules with clean interfaces

This gives the project the best of both approaches:

- strong responsibility clarity
- practical embedded implementation
- easier testing
- easier explanation in articles and design notes

## What This Could Look Like In Practice

A realistic outcome for QFC-1 might look like this:

### Conceptual design view

- `AttitudeEstimator`
- `PIDController`
- `MotorMixer`
- `SystemStateManager`
- `FlightController`

### Implementation view

- `attitude_estimator.c/.h`
- `pid_controller.c/.h`
- `motor_mixer.c/.h`
- `system_state_manager.c/.h`
- `flight_controller.c/.h`
- `imu_driver.c/.h`
- `imu_converter.c/.h`
- `pwm_driver.c/.h`

In other words:

- some conceptual objects may map neatly to modules
- some implementation modules may exist without needing strong object identity

That is realistic and healthy.

## Risks Of Overusing Either Side

### If the design becomes too object-heavy

Risks include:

- unnecessary indirection
- artificial abstractions
- awkward embedded implementation
- hiding timing-critical behavior behind excessive structure

### If the design becomes too module-only

Risks include:

- weaker responsibility ownership
- less clarity about collaboration
- less useful object-model and CRC analysis
- implementation structure chosen too early

The goal is not to avoid both risks entirely, but to stay aware of them while choosing structure.

## Current Direction Chosen For QFC-1

The current direction for QFC-1 is:

- use object-oriented analysis to reason about major conceptual responsibilities
- use CRC-style thinking to refine responsibility ownership and collaboration
- use practical firmware modules to implement the resulting design cleanly

This means:

- the design model may be richer than the final code shape
- the final code shape may be simpler and more module-driven than the conceptual model

That is a deliberate choice, not a compromise born from confusion.

## What This Enables Next

With this distinction now clear, the next design steps become more focused:

- define a candidate object model
- refine collaboration thinking using CRC cards
- decompose the object model into implementation-ready structure

This document is the bridge between behavioral design and structural design. It answers how responsibilities may become structure without forcing a simplistic one-to-one mapping too early.
