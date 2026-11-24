# Buddy 2 host tests

Run from the repository root with GNU Make and a host C compiler:

~~~sh
make -C tests/motion test
~~~

The Makefile uses C11, -Wall, -Wextra, -Werror and optimization.
Seven executables cover control calculations, Motion-loop execution,
packing, encoder snapshots, USB isolation, GPIO IRQ decoding and motor writes.
RTOS and register access are mocked where needed.

IRQ checks include unchanged-level interrupts, single-phase chatter,
forward/reverse decoding, invalid transitions and separated GPIO groups.
The Motion harness includes the 599-count / 14063-us fault replay, braking,
acknowledgement and brake-write failure.
Motor tests verify that initialization and PWM/brake errors propagate.

These checks do not model analogue noise, ISR latency, physical braking,
motor current, supply voltage or actual wiring.
See [Motion](../../docs/project/BUDDY2_MOTION.md) for current configuration
and [recorded observations](../../evidence/motion/FINAL_MOTION_VERIFICATION.md)
for the separate historical hardware evidence.
