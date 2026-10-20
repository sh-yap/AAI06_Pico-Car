# Buddy 2 motion software

Buddy 2 is the sole owner of the two drive motors. It accepts non-blocking
velocity, distance and encoder-based turn commands from the vehicle controller;
Buddy 3, Buddy 4 and Buddy 5 report intent or recommendations and never write
motor PWM directly.

## Build profiles

The existing micro T-Kernel build is used; no second Pico SDK project is
needed. `APP` selects exactly one `usermain()` entry and `BOARD` selects the
hardware resources:

```text
APP=demo                  BOARD=pico
APP=motion_sim_test       BOARD=pico
APP=motion_motor_test     BOARD=robo_pico_robot
APP=motion_encoder_test   BOARD=robo_pico_robot
APP=motion_closed_loop_test BOARD=robo_pico_robot
```

For example, from `build_make`:

```powershell
make APP=motion_closed_loop_test BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc `
  PICO_SDK_PATH=C:/Users/yapsu/.pico-sdk/sdk/2.3.1 `
  SHELL=C:/msys64/usr/bin/sh.exe -j8
```

The image name contains the app/board suffix. `.build_profile`
also includes APP, BOARD, SMP, console and WiFi gates; changing any of these
invalidates the shared object tree instead of incorrectly reporting that there
is nothing to build.

## Public interface

Include `app_program/motion_task.h` and `motion_types.h`. Public distances and
speeds are integer millimetres and millimetres/second; angles are millidegrees
and timeouts are milliseconds. `motion_submit()` is non-blocking, and
`motion_stop()` brakes immediately through a dedicated event path. Invalid,
uninitialised and full-queue conditions are returned explicitly.

The pure calculations in `motion_control.c` have no RTOS or hardware
dependencies. Run their deterministic host tests with:

```sh
make -C tests/motion test
```

## Provisional Robo-Pico resources

| Resource | Assignment |
| --- | --- |
| Motor 1 A/B | GP8 / GP9 |
| Motor 2 A/B | GP10 / GP11 |
| Left encoder A/B | GP16 / GP17 |
| Right encoder A/B | GP4 / GP5 |
| Proposed I2C1 | GP2 / GP3 |
| PWM | 10 kHz, TOP 12499, divider 1 |

Encoder voltage/output type, motor voltage/current and PH2.0-6P orientation
must be verified before applying power. GP16/GP17 are dedicated to the left
encoder in the robot profile; the status LED is on GP19 and the ultrasonic
pair is GP6/GP7. The `robo_pico_robot` profile releases
PWM reset and avoids assigning GP8/GP9 to startup I2C0; the normal `pico`
startup remains unchanged.

## Verification order

1. Run the existing `APP=demo BOARD=pico` baseline.
2. Run host control tests.
3. With wheels raised and current limits confirmed, run the motor test and
   record polarity, brake/coast behaviour and deadband.
4. Run the encoder test with a logic analyser and record counts/revolution and
   invalid-transition rate.
5. Tune speed PID, then distance and turn calibration on the complete car.
6. Exercise stop, timeout, stall, disconnected-encoder and queue-full faults.

Do not claim final-car accuracy until the calibrated tests are repeated after
the other subsystems and the final vehicle mechanics are assembled. Store CSV
records using the fields in `docs/project/PROJECT_GATES.md` under
`evidence/motion/`.

For powered encoder-noise diagnosis, `APP=motion_motor_test` reports encoder
counts, per-wheel invalid transitions, their total and the GPIO IRQ count at
the end of each three-second run. Commands `1`/`2` exercise only the left motor
and `3`/`4` only the right motor; each drive command resets the diagnostic
counters before applying PWM. Output is intentionally paused while moving
because the USB `tm_printf()` path masks local interrupts and can otherwise
create false invalid-transition measurements.

`APP=motion_closed_loop_test` follows the same rule. It stores up to 40 status
samples statically at 10 Hz while motion is active, brakes, and only then dumps
the captured samples over USB. During capture only the stop key is accepted.
Its safety-test keys are `o` for a 500 ms command timeout, `i` for rejection of
an out-of-range command, and `q` for queue saturation using zero-speed commands.
The queue test explicitly brakes before and after the submissions.

Initial unloaded feed-forward calibration uses separate affine fits for each
wheel. The left fit is `59 + 0.3105 * speed_mm_s` permille and the right fit is
`57 + 0.2939 * speed_mm_s`, with the 80-permille deadband and 900-permille
maximum still enforced. These are bench starting values and must be refitted
under final vehicle load and battery power.

Above the 100 mm/s calibration knee, an additional slope of `0.3906` PWM
permille per mm/s is applied on the left and `0.4688` on the right. This keeps
the passing 100 mm/s calibration unchanged while compensating for the
non-linear response measured during the initial 200 mm/s run.

Two unloaded -100 mm/s trials settled the left wheel around 5-6% below target,
while the right wheel met the target. A provisional 6-permille left-reverse
feed-forward boost is therefore applied without changing the forward
calibration. This directional trim must be checked again under final-car load.

Stall timing is maintained separately for each wheel. If either commanded
wheel has a target magnitude of at least 80 mm/s but remains below 20 mm/s for
500 ms, both motors brake and status reports `MOTION_FAULT_STALL`. This catches
a single jammed wheel or disconnected encoder even while the other wheel moves.
After the hardware problem is corrected, a new valid command clears the old
non-fatal fault and starts with reset PID, slew, startup and stall state.
