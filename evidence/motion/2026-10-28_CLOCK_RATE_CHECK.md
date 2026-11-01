# Motion clock-rate check - 2026-10-28

## Follow-up: missing BSP initialization found and corrected

Comparison with the teammate-provided `cpu_clock.c` found that the repository
never explicitly programmed the RP2040 watchdog `TICK` register. That tick is
also the hardware timer reference. The supplied file enables the generator and
sets its divider to `XOSC_MHz` after selecting the 12 MHz crystal for
`CLK_REF`, producing the required 1 us tick on every boot.

The same definitions and register write are now present in
`kernel/sysdepend/pico_rp2040/cpu_clock.c`. The current repository file matches
the supplied file. Both `APP=motion_clock_test` and
`APP=motion_closed_loop_button_test` rebuilt with no emitted compiler warnings
or errors.

After this change, the button test passed on computer USB with counts
4220/4233, speed 105/105 mm/s, encoder travel 301/300 mm and `fault=0`. The
same image then ran successfully from powerbank through VIN without the former
multi-group encoder-fault beeps. The tester observed about 350 mm of physical
travel, so timer/fault behaviour is provisionally corrected while physical
distance calibration remains open.

The encoder counts come from GPIO interrupts. The RP2040 microsecond timer is
used to measure elapsed time for speed and count plausibility. A slow timer
could make valid wheel movement appear too fast, so a motor-free clock image
compared it with the RTOS timebase.

## User-reported computer-USB result

`APP=motion_clock_test BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc` produced
seven consecutive lines, sequences 8 through 14:

- `raw_us`: 1,009,600 to 1,009,678 for each one-second RTOS delay;
- `os_ms`: 1,010 for each corresponding interval;
- mean raw interval: 1,009,632 us, only 368 us below the 1,010,000 us RTOS
  interval (about 0.036% difference);
- `tick_cycles=12`, `running=1`, `enabled=1`, `pause=0`, `ref_div=256` on every
  line. A divider register value of 256 means 1.0 in 8.8 fixed-point form.

The raw timer was running at essentially the same rate as the RTOS clock
under computer USB. The watchdog tick was configured for 12 cycles from the
12 MHz reference clock despite no explicit setup in the BSP at that time.
The later comparison identified this missing BSP initialization. The new write
makes that valid state deterministic for computer-USB and cold external-power
boots.

## Correction to the earlier suspicion

The old button report prints `prev_us` and `sample_us` after casting the
64-bit timer values to 32 bits. The printed value `sample_us=436680` was
therefore **not** a reliable time since boot: the displayed low word wraps
every 4,294.967 seconds (about 71 minutes 35 seconds). Comparing it with the
one-second pre-run delay as an absolute uptime was incorrect. The difference
between `prev_us=422617` and `sample_us=436680` remains 14,063 us for that
sample, because both were captured close together.

The failed run still recorded 599 new left counts and 599 new encoder IRQ
samples in that reported interval. A completely stopped encoder counter or
lost encoder IRQs do not explain those numbers. The source of the rapid
phase transitions, or a fault-time-only clock anomaly, remains unconfirmed.

At the time of the original audit, no Motion threshold or platform clock
configuration had been changed. That statement is now superseded: the BSP
watchdog-tick initialization was added and the first post-fix USB and VIN runs
passed. The earlier 599 GPIO IRQ/count samples remain valid historical data;
an incorrect elapsed-time reference can make real counts accumulated over a
different physical interval appear impossible.
