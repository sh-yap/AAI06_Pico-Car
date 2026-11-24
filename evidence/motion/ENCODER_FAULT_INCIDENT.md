# Encoder timing incident

The 28 October 2026 investigation found that the RP2040 BSP did not explicitly
initialize the watchdog TICK generator after configuring CLK_REF.
Motion uses the hardware timer to associate encoder changes with elapsed time.
An inconsistent time reference can make genuine encoder pulses appear to
exceed a plausible wheel speed.

## Source correction

`kernel/sysdepend/pico_rp2040/cpu_clock.c` now writes
`CPU_CLOCK_WATCHDOG_TICK_ENABLE | XOSC_MHz` to the watchdog TICK register after
reference-clock setup. This establishes the intended microsecond timer
reference each boot rather than depending on earlier hardware state.
It configures timing; it does not enable a reset watchdog for task health.

The existing encoder plausibility check was retained. The Motion tests
include the recorded 599-count / 14063-us fault replay and reject that
implausible sample. Earlier electrical-noise explanations were hypotheses,
not an established diagnosis.

## Recorded post-fix result

The first rebuilt USB run reported a 300 mm target, 301 mm encoder travel,
left/right counts 4220/4233, speeds 105/105 mm/s, invalid transitions 0/0
and fault zero. The same image completed on powerbank VIN without the
earlier encoder fault groups. Reported physical travel was approximately
350 mm; encoder target completion therefore did not establish floor-distance
accuracy. The separate distance scale is recorded in `motion_config.c`.

These are historical observations from the original incident record.
They support the initialization fix without proving that every possible
encoder, clock or power fault has been eliminated.
