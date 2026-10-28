# Buddy 2 host tests

These tests compile the hardware-independent encoder and control calculations
with the host compiler. They are intentionally deterministic and run without a
Pico or RTOS:

```sh
make -C tests/motion test
```

The task, snapshot, USB-isolation and GPIO-IRQ harnesses also exercise production
code with mocked RTOS/hardware access. The IRQ checks cover unchanged-level
interrupts, single-phase chatter, forward/reverse decoding, invalid transitions
and separated GPIO groups. They do not model analogue noise or ISR latency.
The task harness includes the 2026-10-28 599-count / 14063-us fault replay.
The motor harness verifies that PWM initialisation, compare-register and brake
failures are surfaced as hardware errors and do not leave a requested drive
state published as active.

The target build profiles exercise the same public units and interfaces. Host
tests do not prove GPIO electrical levels, motor current, PWM polarity, or
Robo-Pico wiring; those are covered by the staged bench gates in
`docs/project/STAGE_1_GUIDE.md` and the Buddy 2 plan.
