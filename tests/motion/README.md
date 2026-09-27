# Buddy 2 host tests

These tests compile the hardware-independent encoder and control calculations
with the host compiler. They are intentionally deterministic and run without a
Pico or RTOS:

```sh
make -C tests/motion test
```

The target build profiles exercise the same public units and interfaces. Host
tests do not prove GPIO electrical levels, motor current, PWM polarity, or
Robo-Pico wiring; those are covered by the staged bench gates in
`docs/project/STAGE_1_GUIDE.md` and the Buddy 2 plan.
