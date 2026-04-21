# Firmware

This directory will contain the QFC-1 embedded firmware implementation.

Planned layout:

- `include/`: public module headers.
- `src/`: hardware-independent firmware logic.
- `hal/`: HAL-facing adapter interfaces and platform integration points.

The implementation should keep control logic testable on a host machine wherever possible.
