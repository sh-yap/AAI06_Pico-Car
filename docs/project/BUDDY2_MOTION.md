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
APP=motion_stage4_test    BOARD=robo_pico_robot
APP=motion_stage5_test    BOARD=robo_pico_robot
APP=motion_stage6_test    BOARD=robo_pico_robot
APP=motion_stage7_test    BOARD=robo_pico_robot
APP=motion_route_test     BOARD=robo_pico_robot
APP=motion_closed_loop_button_test BOARD=robo_pico_robot
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

`APP=motion_stage4_test` is the powerbank-only loaded-characterisation image.
It does not require WiFi or MQTT. Robo-Pico GP20 repeats the selected bounded
one-second run; GP21 stops an active run, advances the test while idle, or
resets the sweep to forward 150 permille when held for at least one second.
The run deadline uses the RP2040 microsecond clock rather than accumulated
RTOS delays. If USB serial is present, an exact elapsed-time/count report is
printed only after both motors have been braked; GP20 still triggers motion.

`APP=motion_stage5_test` is the powerbank-only PID image. It contains one
required operating point at a time, in forward and reverse 500 mm cases. The
50 and 80 mm/s points are complete in both directions. GP20 starts and GP21
stops a run or toggles direction while idle.

`APP=motion_stage6_test` is the powerbank-only distance-calibration image. It
uses 50 mm/s only and selects the completed 500 mm forward case, an optional
1500 mm forward long-drift case, and the 100 mm reverse case. GP20 starts a
case and GP21 stops an active case; while idle, GP21 advances the case and a
long hold resets to 500 mm forward.
After braking, the serial report includes the target, encoder-derived travel,
left/right encoder counts, invalid transitions, sequence ID, elapsed time,
mean speed and peak speed. It uses the active loaded Q10 1167 (1.140) scale.
GP19 is only an optional external LED output; the Robo-Pico has no visible
onboard status LED assigned to `BOARD_LED_PIN`. If an external LED and resistor
are fitted to GP19, 1-9 flashes after completion represent approximately
10-90 mm/s average encoder speed. This indication is not required for testing.

`APP=motion_stage7_test` is the powerbank-only turn image. GP20 starts the
selected encoder turn at 80 mm/s. GP21 selects left 90, right 90 or U-turn
while idle, and immediately stops while running. One, two and three selection
beeps identify those cases. One result beep means complete; two means operator
stop, and fault result codes remain 3-8.

`APP=motion_route_test` is the powerbank-only combined route image. One GP20
press runs 500 mm forward, 90 degrees left, 500 mm forward, 90 degrees right,
1000 mm forward, a 180-degree right U-turn, then 1500 mm forward. Straight
segments use 50 mm/s and turns use 80 mm/s. GP21 is an immediate stop at every
stage, including the one-second start delay and the short gaps between
commands. One final result beep means the entire route completed; two means
operator stop and fault result codes remain 3-8. The route can be started
again with GP20 after completion.

`APP=motion_closed_loop_button_test` is the powerbank-only straightness image.
GP20 submits a 300 mm forward move at 100 mm/s to the encoder/PID Motion task;
GP21 requests an immediate stop. A one-beep completion indicates the Motion
task reached the encoder distance target; two beeps indicate an operator stop.
Fault codes are three timeout, four stall, five encoder, six invalid command,
seven hardware/configuration, and eight command rejection.

The RP2040 BSP now explicitly initializes the watchdog `TICK` register after
configuring `CLK_REF`. This tick is the hardware timer reference used by
Motion's elapsed-time and encoder plausibility calculations. The first rebuilt
USB run completed at 301/300 mm with counts 4220/4233 and `fault=0`; the same
image then completed from powerbank through VIN without the earlier encoder
fault groups. Approximate physical travel was 350 mm, so the power-dependent
fault is provisionally cleared while distance calibration remains open.

## Public interface

Include `app_program/motion_task.h` and `motion_types.h`. Public distances and
speeds are integer millimetres and millimetres/second; angles are millidegrees
and timeouts are milliseconds. `motion_submit()` is non-blocking, and
`motion_stop()` requests a bounded brake through a dedicated Motion-task event
path; PWM remains owned by the Motion task. Invalid,
uninitialised and full-queue conditions are returned explicitly.
Positive linear speed is forward; positive angular rate and positive turn
angle are vehicle-right. Timeout, stall and encoder faults remain latched until
an explicit `motion_stop()` acknowledgement; a hardware-configuration fault
requires reinitialisation.

The pure calculations in `motion_control.c` have no RTOS or hardware
dependencies. Run their deterministic host tests with:

```sh
make -C tests/motion test
```

The host suite also exercises the real Motion loop, encoder ISR/snapshot,
button-test isolation and motor-driver error paths with mocked hardware. PWM
or brake write failures become a latched hardware fault instead of being
silently reported as successful.

## Provisional Robo-Pico resources

| Resource | Assignment |
| --- | --- |
| Motor 1 A/B | GP8 / GP9 |
| Motor 2 A/B | GP10 / GP11 |
| Left encoder A/B | GP16 / GP17 |
| Right encoder A/B | GP4 / GP5 |
| Proposed I2C1 | GP2 / GP3 |
| PWM | 10 kHz, TOP 12499, divider 1 |

The 2026-10-27 marked manual calibration measured 2535.7 left and 2528.6 right
counts per wheel revolution, rounded in the configuration to 2536 and 2529.
This calibrates encoder counts, not loaded distance accuracy or tyre slip.
The 2026-10-28 loaded floor test measured 570 mm in 10.39 s for a 500 mm,
50 mm/s command. After Stage 5 cleared for the supported 50/80 mm/s scope,
Q10 1167 (1.140) became the active Stage 6 loaded-distance correction.

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
Keys `a`/`d` request a three-second left/right angular-velocity curve; `l`,
`t` and `u` request -90, +90 and 180 degree encoder turns respectively.
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
