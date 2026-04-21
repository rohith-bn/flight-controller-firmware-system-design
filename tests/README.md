# Tests

This directory will contain host-buildable tests for QFC-1 firmware modules.

Initial test focus:

- Unit conversion.
- Angle computation.
- Complementary filter behavior.
- PID controller behavior.
- Motor mixing and clamping.
- System state transitions.

Hardware-facing modules should be tested through fakes or mocks at the HAL boundary.
