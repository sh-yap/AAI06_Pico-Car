# Buddy 2 Motion Catch-Up Plan

This is the working plan for `buddy2-motion-catchup`. Complete and clear one
stage at a time. Do not tune later stages around a failure in an earlier stage.

The plan is based on the repository at commit `259ed1e` (the merge of
`test_integrate` into `main`) and compares it with the last commit on the old
`buddy2-motion` branch, `c75e8b6`. Hardware behaviour described below is not
considered verified until Buddy 2 repeats the test and saves the evidence.

## Progress tracker

| Stage | Purpose | Status | Evidence / commit |
| --- | --- | --- | --- |
| 0 | Freeze the current baseline and agree the contract | In progress | Contract received; WiFi build/limits pending |
| 1 | Re-check wiring, motors and encoders safely | In progress; functional checks pass, electrical/power checks pending | `evidence/motion/2026-10-27_MANUAL_ENCODER_CALIBRATION.md` |
| 2 | Fix motion correctness and concurrency defects | Software regression, raised-wheel manoeuvres, timeout, acknowledgement, recovery, reversal and stop-priority pass; five corrected stop-latency runs measured 204 us-10.690 ms, team limit remains | `evidence/motion/2026-10-27_SOFTWARE_VERIFICATION.md`, `evidence/motion/2026-10-27_RAISED_WHEEL_CLOSED_LOOP.md`, `evidence/motion/2026-10-27_RAISED_WHEEL_MANOEUVRE_DEFECT.md` |
| 3 | Recalibrate encoder geometry with the complete car | Cleared for progression; CPR and loaded effective diameters are documented, powered encoder-noise verification continues with Stage 4 | `evidence/motion/2026-10-27_STAGE3_LOADED_GEOMETRY.md` |
| 4 | Characterise the motors under the final load | In progress; button firmware built, physical sweep pending | `evidence/motion/2026-10-28_STAGE4_LOADED_MOTOR_RESULTS.md` |
| 5 | Tune loaded closed-loop wheel-speed control | Button-controlled 100 mm/s straight test built; physical run pending | `evidence/motion/2026-10-28_CLOSED_LOOP_STRAIGHT_TEST.md` |
| 6 | Tune distance and straight-line motion | Not started | |
| 7 | Tune left, right and U-turns | Not started | |
| 8 | Prove stop and fault behaviour | Not started | |
| 9 | Integrate with the current team software | Not started | |
| 10 | Run final regression and close the Motion gate | Not started | |

Change a stage to **Cleared** only when all of its exit checks pass and its
evidence is saved. Record the commit and evidence path in the last column.

## Non-negotiable project rules

- Buddy 2 is the only subsystem that owns the drive motors and wheel encoders.
- Buddy 3 and Buddy 5 report navigation information or recommendations. The
  Vehicle Controller decides, then requests motion through `motion_task.h`.
- Keep distances in millimetres, speeds in millimetres/second, public angles
  in millidegrees, and timeouts in milliseconds.
- Commands must be non-blocking. Motion must report completion or a specific
  failure instead of making the Vehicle Controller wait blindly.
- Keep ISRs short, never print from an encoder ISR, and keep motor hardware
  access serialised.
- Do not change the lecturer's kernel unless an application-level solution is
  not possible and the change has been agreed by the team.
- Do not claim final-car calibration using raised-wheel or unloaded results.

## Current baseline

### Already present

- Motor PWM driver, quadrature encoder driver, control calculations and RTOS
  Motion task.
- Non-blocking velocity, distance, turn and urgent-stop API.
- Stand-alone motor, encoder, simulation and closed-loop test applications.
- Host tests for the hardware-independent calculations.
- Vehicle Controller integration for drive, stop, reverse and turns.
- Motion telemetry through the communications subsystem.

### Current hardware assignment

| Resource | Current assignment |
| --- | --- |
| Left motor A/B | GP8 / GP9 |
| Right motor A/B | GP10 / GP11 |
| Left encoder physical A/B | GP16 / GP17; decoded as GP17 / GP16 |
| Right encoder A/B | GP4 / GP5 |
| Encoder GPIO IRQ | IO_IRQ_BANK0 on Motion's processor |
| PWM | 10 kHz, TOP 12499, divider 1 |

GP6/GP7 now belong to the ultrasonic subsystem and GP19 is the status LED.
The encoder change in `c00a4f5` moved the left encoder to GP16/GP17 and updated
the driver to handle encoder pins in multiple GPIO interrupt banks.

### Calibration values currently in the code

These are starting values, not final values for the heavier car:

| Item | Left | Right | Source / warning |
| --- | ---: | ---: | --- |
| Counts per wheel revolution | 2536 | 2529 | Mean of two marked ten-revolution manual trials, rounded |
| Wheel diameter | 65 mm | 65 mm | Provisional physical measurement |
| Feed-forward offset | 120 permille | 120 permille | Raised from unloaded 59/57 after a loaded track test |
| Base feed-forward slope, Q10 | 318 | 301 | Unloaded fit |
| Extra slope above 100 mm/s, Q10 | 400 | 480 | Initial 200 mm/s result |
| Reverse boost | 6 permille | 0 permille | Two unloaded reverse trials |

Other provisional values are a 145 mm track width, 80-permille deadband,
350-permille start-up boost for 400 ms, maximum 900 permille, speed PID
`Kp = 0.40`, `Ki = 1.20`, `Kd = 0`, and a 1000 ms stall limit. The 350/400 ms
start-up settings, increased feed-forward offsets and faster PID were added on
`test_integrate` after the power-bank load caused repeated starts to stall.
They must be measured again rather than accepted as final calibration.

### Baseline defects identified before the Stage 2 correction pass

This list describes the inherited baseline. The corresponding Stage 2
checkboxes below record which defects have now been corrected and tested.

1. Distance and turn progress use encoder totals from power-on. A manoeuvre
   after earlier driving can complete immediately or use the wrong distance.
2. Left turns use a negative target but positive absolute wheel travel, so a
   negative-angle turn cannot meet its completion condition and times out.
3. Every accepted velocity update resets PID, slew, start-up and stall state.
   The current Vehicle Controller reduces resends as a workaround, but Motion
   should distinguish an update from a new or reversing command.
4. `motion_stop()` and the Motion loop can race with a newly submitted command.
   The Vehicle Controller currently waits 40 ms before a manoeuvre so that the
   stop-side queue flush does not discard it.
5. Motion status fields and command-state flags are accessed from both SMP
   processors without protecting the complete state update. `volatile` alone
   is not cross-core synchronisation.
6. Positive-angle meaning was not consistent between velocity mixing and the
   discrete-turn path. The team accepted positive-right as the target contract,
   while the baseline VC sent a compensating negative angular rate. This
   required a coordinated Motion + VC change.
7. Velocity validation checks linear speed but not angular rate or the derived
   per-wheel targets. Distance speed/distance sign combinations and negative
   turn-speed inputs also need explicit rules.
8. Speed calculation assumes every loop took exactly 20 ms even if WiFi or
   another task delayed it. The loop also delays relative to its finish time,
   so execution time adds drift.
9. Impossible encoder deltas and excessive invalid transitions are reported
   but not rejected or converted into an encoder fault.
10. Straight correction compares raw left/right counts even though each wheel
    has a different counts-per-revolution value. It should compare normalised
    wheel distance or speed relative to the current command origin.
11. `motion_stop()` can write PWM from the VC task on processor 2 while the
    Motion task writes PWM on processor 1. Motor access must be serialised.
12. `motion_encoder_get_snapshot()` uses `DI()`, which only masks interrupts on
    the calling processor. Its safety currently depends on which processor
    calls it; the snapshot contract must be made explicitly SMP-safe.

## Stage 0 - Baseline and interface agreement

Goal: make the current branch reproducible and freeze what the team expects
from Motion before changing control code.

### Working order

Do only these steps for now:

1. Reproduce the current software baseline. This is complete locally; see the
   verification record below.
2. Copy the proposed Motion contract below into the team discussion and settle
   every unchecked contract item. Record any changed decision here before
   changing code.
3. The group has supplied the exact `vc_car` WiFi/MQTT flags. Create the
   ignored local configuration files from the repository examples without
   committing credentials, then build that exact profile.
4. Agree numerical acceptance limits. Use measurements or assignment criteria;
   do not select limits only because the current code happens to meet them.
5. Save the final build/test output and team decisions under
   `evidence/motion/`, fill the Stage 0 clearance note, and only then start
   Stage 1.

### Tasks

- [x] Confirm the branch is `buddy2-motion-catchup` and record the starting
  commit in this document (`259ed1e`).
- [x] Confirm the source working tree was clean before creating this plan.
- [x] Run `make -C tests/motion test`.
- [x] Run `make -C tests/vc test` because the Vehicle Controller consumes the
  Motion API.
- [x] Build these profiles with no compiler errors or new warnings:
  - [x] `APP=motion_motor_test BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc`
  - [x] `APP=motion_encoder_test BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc`
  - [x] `APP=motion_closed_loop_test BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc`
  - [x] `APP=vc_car BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc` without WiFi,
    as a local whole-application compatibility build.
  - [x] The exact group profile below, using local ignored configuration files;
    credentials are never committed.
- [x] Agree and write down the Motion contract with the Vehicle Controller:
  - [x] Positive linear speed is vehicle-forward.
  - [x] Positive angular rate and positive turn angle are the agreed right-turn
    convention. Motion and the VC were changed together in the Stage 2 pass.
  - [x] `TURN_ANGLE.wheel_speed_mm_s` is a positive magnitude.
  - [x] Distance and speed signs must agree, except zero distance is rejected
    or explicitly defined as immediate completion.
  - [x] A new same-mode velocity command updates/replaces the setpoint without
    resetting PID; starting, reversing and changing mode may reset it.
  - [x] Stop has priority; a command accepted after a stop must not be deleted
    by an older stop-side queue flush.
  - [x] Stall/encoder faults require the controller's stop acknowledgement
    before retry; hardware-configuration faults require reinitialisation.

- [x] Exact group WiFi/MQTT build command received:
  `make APP=vc_car BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc WIFI=cyw43
  WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
  PICO_SDK_PATH=<your SDK path> -j8`.
- [x] Confirm `APP=vc_car` is present on this catch-up branch: the branch is
  already at `259ed1e`, where `origin/main` merged `test_integrate` in PR #7.
  The teammate's “not on main yet” note described the state before that merge.
- [x] Create `config/wifi_credentials.h` and `config/mqtt_config.h` from the
  `.example.h` files. The exact team command uses DHCP, so
  `config/network_config.h` is not required unless a later static-IP profile
  is selected. Keep all credentials and broker details uncommitted.

Team-approved contract and current implementation state:

| Topic | Recommended decision |
| --- | --- |
| Linear sign | Positive is vehicle-forward; negative is reverse |
| Angular sign | Positive is the agreed right/clockwise vehicle turn; Motion mixing and the VC now implement this together, pending physical direction verification |
| Turn speed | Always a positive magnitude; the angle selects left/right |
| Distance sign | Distance and speed must have the same sign; zero-distance commands are rejected |
| Velocity updates | Latest accepted setpoint replaces the old setpoint without resetting PID unless starting, reversing or changing mode |
| Stop | Stop has priority and reports `STOPPED`; a command accepted after that stop must not be removed by an old queue flush |
| Fault recovery | Timeout, stall and encoder faults brake and latch until an explicit controller stop/acknowledgement plus a valid retry; hardware-configuration faults require reinitialisation |

### Group floor-test observations to carry into Motion work

These values were supplied as `test_integrate` observations, not final
calibration. Preserve them as starting evidence and retune them on the loaded
car rather than treating them as accepted constants:

| Observation | Temporary value used by `test_integrate` |
| --- | --- |
| Standstill starts stalled 4/5 times | `startup_pwm_permille`: 200 -> 350; boost: 200 -> 400 ms |
| Loaded 200 mm/s needed more power than predicted | Feed-forward offsets: 59/57 -> 120/120 permille |
| Loaded speed control was too slow | PID: `Kp` 0.15 -> 0.40; `Ki` 0.30 -> 1.20 |
| Stall detection was too early for the loaded start | Stall limit: 500 -> 1000 ms |

- [ ] Agree measurable acceptance limits with the team. Do not silently invent
  them during tuning. Fill in the table below.

| Metric | Agreed limit |
| --- | --- |
| Wheel-speed steady-state error | TBC |
| Wheel-speed overshoot | TBC |
| 500/1000/1500 mm distance error | TBC |
| Lateral drift | TBC |
| 90/180 degree turn error | TBC |
| Stop response time and stopping distance | TBC |
| Maximum invalid encoder transition rate | TBC |
| Minimum tested battery voltage | TBC |

### Local verification record - 2026-10-27

Environment: GNU Make 4.4.1, Arm GNU Toolchain 15.2.Rel1
(`arm-none-eabi-gcc` 15.2.1), Pico SDK 2.3.1, SMP enabled and USB CDC console.

| Check | Result | ELF text/data/bss bytes |
| --- | --- | ---: |
| Motion host tests | 47 passed | N/A |
| Vehicle Controller host tests | 150 passed, 0 failed | N/A |
| `motion_motor_test` | Build passed; 0 emitted warnings/errors | 95076 / 152 / 32304 |
| `motion_encoder_test` | Build passed; 0 emitted warnings/errors | 92992 / 152 / 32304 |
| `motion_closed_loop_test` | Build passed; 0 emitted warnings/errors; key `e` timestamped stop diagnostic added | 98888 / 156 / 35580 |
| `vc_car`, no WiFi | Build passed; 0 emitted warnings/errors | 93292 / 152 / 36400 |
| `vc_car`, exact WiFi/MQTT profile | Build passed; binary MQTT; UF2 generated | 365757 / 2708 / 69580 |

The exact WiFi/MQTT profile was built successfully after creating the ignored
local headers and setting their guard flags. The generated UF2 was not flashed
and no hardware WiFi, MQTT, or vehicle behaviour was verified during this
check. A `network_config.h` file remains unnecessary for the DHCP profile and
is only needed for a static-IP build.

### Exit check

- [x] Baseline and post-correction tests/builds pass and their summaries are
  saved under `evidence/motion/`.
- [x] The controller/Motion contract is agreed; the implementation gaps remain
  explicitly listed for Stage 2.
- [ ] Acceptance limits are agreed and recorded.
- [ ] No hardware result is claimed in this stage.

## Stage 1 - Safe motor and encoder re-check

Goal: verify that the merged pin/resource changes match the actual car before
running closed-loop code.

### Safety setup

- [ ] Record motor rated voltage/current and encoder supply/output voltage.
- [ ] Confirm the encoder outputs are safe for the RP2040 3.3 V GPIO domain
  and whether pull-ups are required.
- [ ] Confirm both PH2.0-6P connector orientations and continuity.
- [ ] Confirm the power-bank and motor supply arrangement, polarity, fuse or
  current limit, and a physical way to remove power immediately.
- [ ] Raise and secure the driven wheels for the first powered tests.
- [ ] Keep the floor area clear before any loaded test.

### Motor and encoder checks

- [x] Flash `motion_motor_test`; start at low duty and test one wheel at a time.
- [x] Verify left forward/reverse and right forward/reverse physical polarity.
- [ ] Verify `BRAKE` and `COAST` behaviour.
- [x] Verify the three-second safety brake for every drive command.
- [x] Confirm forward wheel motion produces positive encoder counts on both
  wheels and reverse produces negative counts.
- [x] Confirm motor noise does not produce an IRQ storm or a high invalid
  transition count.
- [x] Flash `motion_encoder_test`; manually rotate each wheel and confirm only
  the correct side changes.
- [x] Repeat ten manual revolutions per wheel and save every count, not only
  the mean.
- [ ] If available, compare A/B signals with a logic analyser and confirm valid
  quadrature phase order and 3.3 V levels.

### Exit check

- [ ] Pins, polarity, signs and electrical levels are confirmed.
- [ ] Counts/revolution spread and invalid-transition rate are recorded.
- [ ] Evidence is saved under `evidence/motion/` with the firmware commit,
  tester, setup, battery voltage, expected result and observed result.

## Stage 2 - Correctness and SMP hardening

Goal: make commands correct and deterministic before tuning the plant.

### Implementation pass completed locally - 2026-10-27

The application-side correction pass is implemented in the working tree and
passes 47 Motion host checks, 150 Vehicle Controller host checks, all Motion
firmware profiles, and both integrated `vc_car` profiles. The following items are now
implemented: per-command encoder origins, signed distance/turn completion,
normalised straight correction, velocity-update PID preservation, command
validation, measured loop timing with an absolute schedule, encoder-delta
plausibility checks, serialized stop/PWM ownership, complete status locking,
an SMP-safe encoder snapshot, motor-driver error handling, fault latching, and
positive-right velocity/discrete-turn targets changed together with the VC.
The verification summary is saved as
`evidence/motion/2026-10-27_SOFTWARE_VERIFICATION.md`.
The raised-wheel velocity/sign check is saved as
`evidence/motion/2026-10-27_RAISED_WHEEL_CLOSED_LOOP.md`: forward, reverse,
left-curve and right-curve signs passed with no Motion faults or invalid
encoder transitions. Raised-wheel discrete turns and repeated distance tests
also pass. Stop latency and the remaining command-order regressions are still
required, so this stage is not cleared yet.

A subsequent raised-wheel manoeuvre run exposed two additional defects: new
distance/turn commands did not update the Motion task's active command type,
and the diagnostic logger accepted a stale terminal state from the preceding
sequence. Both are corrected in the working tree and the failed run is
documented in
`evidence/motion/2026-10-27_RAISED_WHEEL_MANOEUVRE_DEFECT.md`. Hardware retest
of the corrected left turn, right turn, U-turn and repeated distance paths
passed. One non-repeatable slow right-wheel start remains recorded for loaded
motor characterisation.

### Command progress and sign fixes

- [x] Store left/right encoder origins when a distance or turn command starts;
  calculate command progress from the delta to those origins. Do not depend on
  the lifetime encoder total being zero.
- [x] Keep speed-estimator previous counts separate from manoeuvre origins.
- [x] Use magnitude for turn completion while retaining the angle sign only
  for left/right direction.
- [x] Calculate straight progress from normalised per-wheel distance, not raw
  count equality.
- [x] Define and validate distance/speed sign combinations.
- [x] Require turn wheel speed to be a positive magnitude.
- [x] Change Motion and VC together so positive-right handling is identical
  for angular velocity and discrete turns, with host regression coverage for
  both target-mixing paths. Raised-wheel testing physically confirmed the
  steering direction; loaded-floor behaviour remains to be checked.

### Command, stop and shared-state fixes

- [x] Treat a same-direction `SET_VELOCITY` as a setpoint update without
  resetting PID/integral/start-up state. Reset when starting from rest,
  reversing direction, changing command mode, stopping or faulting.
- [x] Give stop deterministic priority without flushing a command submitted
  after that stop. Remove the need for the VC's 40 ms timing workaround only
  after this behaviour has a regression test.
- [x] Ensure only one serialised execution context writes the PWM hardware, or
  document and protect any exceptional immediate-brake path.
- [x] Protect a complete status snapshot with the mutex; do not update most
  fields outside the lock and then lock only the timestamp.
- [x] Replace cross-core `volatile` coordination with RTOS synchronisation,
  critical sections or suitable atomics.
- [x] Make `motion_encoder_get_snapshot()` safe when called from processor 2;
  `DI()` only masks interrupts on the caller's processor. Use a documented
  snapshot/critical-section ownership strategy rather than relying on the
  current call location.
- [ ] Keep the emergency-stop response bounded while ensuring the VC cannot
  write PWM concurrently with the Motion task. Measure the resulting stop
  latency after the serialization change. The closed-loop diagnostic now
  provides key `e`, records the RP2040 request/brake timestamps, and prints
  the measured latency. The corrected five-run raised-wheel set measured
  204 us-10.690 ms with all runs stopping safely. The agreed maximum and
  loaded-car stopping-distance test remain pending.
- [x] Validate angular rate and the derived left/right wheel targets against
  supported limits.
- [x] Check and act on motor-driver return values.

### Timing and encoder plausibility

- [x] Use measured elapsed time for speed and PID calculations.
- [x] Run the Motion loop from an absolute schedule or an RTOS timed wait so
  work time does not accumulate as period drift.
- [x] Add a physically justified maximum encoder delta/speed check.
- [x] Define an invalid-transition rate/window that raises
  `MOTION_FAULT_ENCODER` instead of merely reporting a counter.

### Regression tests to add

- [x] A raised-wheel distance command after preceding velocity/recovery
  activity started from its own relative origin and reached 100 mm; loaded
  floor distance remains a later-stage calibration test.
- [x] Two raised-wheel distance commands in one boot both reached their
  requested relative distance; loaded floor distance remains a later-stage
  calibration test.
- [x] `-90`, `+90` and `180` degree turn geometry/completion paths pass host
  calculation tests; physical completion remains a raised-wheel/floor test.
- [x] Repeated same-direction velocity updates use the tested no-reset path.
- [x] Reversal resets the appropriate controller state and observes the
  direction-change brake interval. The automatic `v` forward-to-reverse
  capture showed the brake interval and balanced reverse speeds.
- [x] Stop followed immediately by a new command has deterministic behaviour;
  the automatic `p` capture accepted the replacement reverse command.
- [x] Mismatched signs, excessive angular rates and excessive derived wheel
  speeds are rejected.
- [x] Encoder wraparound and a delayed control period are handled by tested
  hardware-independent calculations.
- [x] An impossible encoder jump is rejected by the tested plausibility check;
  the Motion loop maps that rejection to `MOTION_FAULT_ENCODER`.
- [x] Run Motion and VC host tests, all Motion application builds, and the
  integrated `vc_car` build after the fixes.

### Exit check

- [ ] All known correctness defects above have a test that failed before the
  fix and passes after it.
- [x] Builds are warning-free under the project's enabled warning set.
- [x] Raised-wheel direction, velocity, repeated-distance and discrete-turn
  smoke tests pass before the car is placed on the floor. One non-repeatable
  slow right-wheel start remains recorded for loaded motor characterisation.

## Stage 3 - Loaded encoder and geometry calibration

Goal: obtain the geometry used for distance and turn calculations with the
complete vehicle configuration.

The added weight does not normally change electrical encoder counts per motor
revolution. It can change tyre compression, slip, rolling resistance, starting
torque and the effective turning radius. Therefore re-check CPR, but expect the
largest changes in effective millimetres/count, feed-forward and PID tuning.

### Fix the test configuration

- [x] Record the complete-car mass: approximately 665 g (0.665 kg).
- [ ] Install the powerbank in its normal physical position for the loaded
  rolling test; battery charge is not relevant while rolling manually.
- [x] Record approximate loaded axle height: 30 mm, implying an initial
  diameter cross-check of approximately 60 mm.

### Measure

- [x] Reuse the documented manual CPR calibration (2536 left, 2529 right);
  repeat it only if the wheel, encoder or gearing changes.
- [x] Calculate effective millimetres/count and loaded wheel diameter from a
  controlled 300 mm forward-and-return test. Powered distance belongs to Stage 6.
- [ ] Measure effective track width during Stage 7 turn calibration rather
  than blocking this stage on a rough physical measurement.
- [x] Verify forward/reverse signs again with the assembled car.
- [x] Update configuration constants only from saved measurements.
- [x] Add the date, vehicle configuration and method beside every calibrated
  constant; avoid anonymous magic numbers.

### Exit check

- [x] CPR and effective distance/count are repeatable and documented.
- [x] Left/right wheel-diameter constants match the final loaded car.
- [ ] Encoder invalid-transition behaviour remains acceptable with motors on;
  collect this together with the Stage 4 powered tests.

## Stage 4 - Loaded motor characterisation

Goal: replace unloaded and emergency `test_integrate` guesses with measured
loaded feed-forward and start-up values.

### Power and control arrangement

- The powerbank supplies electrical power to the car only.
- Do not connect computer USB power and the powerbank simultaneously on the
  current hardware arrangement.
- WiFi and MQTT are not required for the Stage 4 raw motor tests. Use a
  dedicated test application controlled by the Robo-Pico user buttons on
  GP20 and GP21 while the car is powered only by the powerbank.
- Do not use the Pico BOOTSEL button or reset control to command motion.
- The existing `motion_motor_test` application accepts USB-console commands,
  so it cannot perform this powerbank-only test. The dedicated
  `APP=motion_stage4_test` mode starts braked, runs only one bounded test per
  button press and always provides an immediate stop action.
- WiFi/MQTT testing is deferred until the later integrated-load checks. The
  integrated `vc_car` MQTT interface only exposes mission `start`/`stop`, not
  raw PWM sweep commands.

### Button-controlled test firmware

- [x] Add a dedicated `motion_stage4_test` build target for the Robo-Pico.
- [x] Keep WiFi disabled; the required build uses `WIFI=none`.
- [x] GP20 repeats the selected one-second raw-PWM test. After GP20 is
  accepted, the buzzer gives an immediate acknowledgement; after GP20 is
  released, the status LED flashes for a one-second hand-clear countdown
  before the motors start.
- [x] GP21 brakes an active test immediately. While idle, a short GP21 press
  gives an immediate acknowledgement and advances one case; holding GP21 for
  at least one second resets to forward 150 permille.
- [x] The sweep is 150, 175, 200, 225, 250, 275, 300, 325 and 350 permille,
  first forward and then reverse. The same case can be repeated with GP20.
- [x] Each run resets the encoders, drives both wheels for no more than one
  second, brakes, and checks wheel activity, encoder direction and invalid
  transitions. No console operation occurs during the powerbank test.
- [x] GP19 (if an external status LED is fitted) and the Robo-Pico buzzer on
  GP22 (driven with a 2 kHz PWM tone) report the selected case and result.
  Forward selection is 1-9
  short flashes. Reverse selection is one long flash followed by 1-9 short
  flashes. Result codes are: 1 pass, 2 stopped, 3 left low activity, 4 right
  low activity, 5 both low, 6 wrong encoder direction, 7 invalid encoder
  transition, and repeating 9-flash groups for an initialisation failure.
- [x] Add deterministic host tests for the case table and result priority.
- [x] All 61 current Buddy 2 host tests pass with `-Wall -Wextra -Werror`.
- [x] Build `APP=motion_stage4_test BOARD=robo_pico_robot SMP=1
  CONSOLE=usb_cdc WIFI=none` with no emitted warnings or errors. The generated
  ELF uses 94760 / 152 / 32316 text/data/bss bytes.
- [ ] Flash the generated Stage 4 UF2 and complete the physical worksheet at
  `evidence/motion/2026-10-28_STAGE4_LOADED_MOTOR_RESULTS.md`.

### Test matrix

- [ ] Use a clear, straight floor lane and a spotter/emergency power cut-off.
- [ ] Complete the initial sweep with the same powerbank arrangement used on
  the final car. Battery-range robustness can be repeated later; no charge
  percentage is required for this first pass.
- [ ] Sweep safe PWM values in small steps with both wheels driving. Record
  one-second travel, drift, whether each wheel starts, and the status/buzzer
  encoder result code.
- [x] The initial low-duty results are recorded in
  `evidence/motion/2026-10-28_STAGE4_LOADED_MOTOR_RESULTS.md`. For the short
  course, 275-350 permille is optional characterization rather than a required
  operating point.
- [ ] Repeat in forward and reverse; do not assume one reverse trim fits both
  directions or battery levels.
- [ ] Measure the lowest repeatable moving PWM and the lowest repeatable
  start-from-rest PWM for each wheel/direction.
- [ ] Determine the shortest reliable start-up boost and ensure it does not
  cause a dangerous launch or excessive current.
- [ ] Repeat each important point enough times to expose intermittent starts.

### Fit and update

- [ ] Plot or tabulate PWM against wheel speed for each wheel/direction.
- [ ] Decide whether the current piecewise affine model is adequate.
- [ ] Refit offsets, base slopes, high-speed slopes, knee and reverse boosts.
- [ ] Set `max_speed_mm_s` to a speed the loaded car can sustain at the agreed
  minimum battery voltage, not merely the fastest fresh-battery result.
- [ ] Revisit the 350-permille/400 ms boost and 1000 ms stall limit using the
  measurements; keep stall detection independent for each wheel.

### Exit check

- [ ] Loaded feed-forward predicts both wheel speeds without relying on a large
  PID correction.
- [ ] Both wheels start reliably across the agreed battery range.
- [ ] Selected limits are safe for the car, driver and test area.

### Stage 5 powerbank validation (prepared)

The raw-PWM floor sweep has established the loaded start threshold and the
left-faster/right-drifting bias. Precision straightness is now tested through
the encoder/PID controller, not by assigning equal raw PWM to both motors.

- [x] Add `APP=motion_closed_loop_button_test` for powerbank-only testing.
- [x] GP20 submits a 300 mm forward move at 100 mm/s using `motion_task`.
- [x] GP21 requests an immediate stop while the move is active.
- [x] Startup boost, per-wheel PID, normalized straight correction and encoder
  distance completion remain owned by the Motion task.
- [x] Replace the provisional 350 permille / 400 ms startup boost with the
  provisional value 200 permille / 250 ms (via 225). Changes did not resolve
  the fault; startup power has not been established as its cause.
- [x] Separate encoder physical-speed plausibility from the 300 mm/s command
  limit. The physical bound is provisionally 1000 mm/s with the existing
  transient margin. Actual launch speed was not measured: the raw test summed
  requested RTOS delays, so its travel cannot be treated as mm/s.
- [x] Buzzer feedback: one acknowledgement beep, then one completion beep;
  two means operator stop, three timeout, four stall, five encoder fault, six
  invalid command, seven hardware/configuration and eight command rejection.
- [x] Preserve the Vehicle Controller's single encoder-fault category while
  adding button-test diagnostic detail. After five encoder-fault beeps, a
  second group means: one count-delta plausibility, two speed calculation,
  three invalid quadrature transitions, or four unknown detail.
- [x] After the repeat identified count-delta plausibility, retain the fault
  wheel mask, sample interval and raw left/right delta in `motion_status_t`.
  The powerbank-only diagnostic emits these as three additional beep groups
  so the false delta can be corrected without disabling the safety check.
- [x] Restore immediate count-delta braking. The three-sample workaround did
  not resolve the fault and let suspect counts affect distance/steering.
  Reject the sample before any PID, distance or steering update.
- [x] Retain timer read-order and bounded restart-boost changes as provisional;
  neither has been shown to resolve the physical failure.
- [x] Add `g` to `motion_closed_loop_test` for an exact 300 mm / 100 mm/s USB
  diagnostic and print the retained encoder fault interval and left/right raw
  deltas. This replaces further threshold guessing from buzzer buckets.
- [x] USB serial-only diagnostic completed at encoder travel 301/300 mm with
  no fault and zero reported invalid transitions. This is not proof that the
  button firmware passes on USB or that the powerbank causes the fault.
- [x] Add a symmetric 25-permille-per-control-period drive-output soft-start.
  It did not resolve the fault. Latest runs gave `5,1,2,2,4` and `5,1,2,3,5`
  with about 130 mm travel and variable heading; underlying cause is unresolved.
- [x] Add `[button-diag-01]` reports to the button image, frozen first-fault
  counts/timestamps/IRQs/pre-brake PWM, and idle `p` report replay.
- [x] Add actual Motion-loop host regression checks with mocked hardware;
  fourteen checks plus the existing 65 helper checks pass.
- [x] Compare GP20 using the same button image on USB and powerbank. USB
  completed without a fault; powerbank turned slightly right and faulted with
  `5,1,2,3,4` (right count-delta check, 16-30 ms, maximum delta 501-1000).
  Electrical supply and USB-connected software timing remain possible causes.
- [x] Same button image over USB reached `state=COMPLETE`, `fault=0`, encoder
  travel 300/300 mm, counts 4214/4204 and invalid transitions 0/0. The reported
  160 mm floor movement was followed by about 145 mm on a repeat. The tester
  carried the computer alongside the vehicle; cable drag is not established.
  Physical distance accuracy remains unresolved despite encoder completion.
- [x] Repeat the same button image on powerbank with both drive wheels clear
  of the floor. Both attempts, including after reset, faulted with `5,1,1,2,3`
  (left count-delta check, 6-15 ms, maximum delta 251-500). The tester observed
  slight movement of possibly both wheels first, then only the left on repeat.
  The fault persists without floor load and is not confined to the right side.
- [x] Confirm actual power inputs: the original powerbank setup used VIN;
  the later two-port comparison explicitly used Pico micro-USB, as did the
  computer. The later comparison changes source and USB-host presence.
- [x] Compare both powerbank output sockets through the Pico micro-USB
  arrangement. The tester saw the right wheel with the left socket
  (`5,1,2,2,3`) and the left wheel with the right socket (`5,1,1,2,3`).
  These observations do not establish a causal port-to-wheel relationship;
  electrical and software timing causes remain unresolved.
- [x] Re-read the powerbank output label: both ports are nominally 5 V.
  This does not establish voltage or current delivery under motor load.
- [x] Reproduce a software timing defect: preemption between clock read and
  encoder acquisition can create a false count-delta fault. Add the timestamp
  to the sequence-validated snapshot and use it throughout Motion sampling.
  Use raw high/low/high timer reads to avoid the shared timer latch.
- [x] Verify button-diag-02: 65 helper, 19 Motion-loop and four encoder-reader
  host checks pass, including genuine fault braking; button UF2 builds without
  emitted warnings/errors. The physical cause is not yet established.
- [x] Button-diag-02 powerbank raised-wheel run on the tester's right port
  produced `5,1,1,2,4`; only the left wheel was visibly moving. The timing fix
  did not clear the powerbank fault. Repeated unchanged port tests are not
  useful; a hardware cause is not established by the beep code.
- [x] Flash button-diag-02 and run GP20 once on powerbank through micro-USB,
  VIN feed disconnected, both wheels raised, one fixed powerbank port. The
  right-port run produced `5,1,1,2,4`; only the left wheel visibly moved.
- [x] Wall adapter labelled 5 V / 1 A also gave `5,1,1,2,3` with only the
  left wheel visibly moving. Neither source adequacy nor a hardware defect
  is established; stop attributing the issue solely to the powerbank.
- [x] Prepare button-diag-03 `n` diagnostic: temporarily disconnect USB data
  through the USB owner task while retaining computer power, then restore
  USB and replay the exact saved report with `p`. Diagnostic hooks are absent
  from normal firmware. Both build profiles pass; 95 host checks pass.
- [ ] With button-diag-03 on computer USB and wheels raised, run GP20 baseline.
  If successful, send `n`, wait for COM to return, reopen Serial Monitor and
  send `p` without resetting. Compare both reports before further changes.
- [ ] Flash the button-controlled closed-loop UF2 and complete the physical
  straightness worksheet.
- [ ] If the path still curves right, use the logged/observed direction to
  tune the normalized correction and per-wheel loaded feed-forward; do not
  change Stage 3 wheel geometry for a steering bias.

## Stage 5 - Closed-loop wheel-speed tuning

Goal: achieve repeatable left/right wheel speed under the final load.

### Tuning order

- [ ] Start with the measured feed-forward from Stage 4.
- [ ] Tune proportional gain, then integral gain; add derivative only if the
  evidence shows it is necessary.
- [ ] Confirm anti-windup and output limits during start, saturation and stop.
- [ ] Tune slew rate separately from PID gains.
- [ ] Re-check start-up boost and stall thresholds after PID tuning.

### Required tests

- [ ] Test 100, 200 and 300 mm/s where each value is within the proven loaded
  range; also test the VC's 80, 100, 120 and 200 mm/s operating points.
- [ ] Test forward and reverse.
- [ ] Record rise time, overshoot, steady-state mean/error, left/right mismatch,
  PWM and battery voltage.
- [ ] Repeat at least ten times at important operating points.
- [ ] Repeat at high, nominal and minimum battery voltage.
- [ ] Verify velocity setpoint updates do not create a stop/start pulse or
  clear useful integral state.
- [ ] Verify WiFi/console activity does not corrupt speed estimates or create
  missed encoder transitions.

### Exit check

- [ ] Every supported operating point meets the Stage 0 limits.
- [ ] Gains and feed-forward constants are recorded with units and test data.
- [ ] No intermittent start or false stall remains.

## Stage 6 - Distance and straight-line control

Goal: make encoder distance commands and normal straight driving repeatable.

### Tasks

- [ ] Test forward and reverse distance commands from a fresh boot and after
  prior velocity/turn commands.
- [ ] Test 100 mm reverse because this is the Vehicle Controller's current
  obstacle rescan command.
- [ ] Test 500, 1000 and 1500 mm straight runs, at least ten times each.
- [ ] Record commanded distance, left/right wheel distance, measured floor
  distance, lateral drift, stopping overshoot and battery voltage.
- [ ] Make straight correction use normalised wheel distance/speed error.
- [ ] Add a bounded approach/deceleration profile if constant-speed threshold
  stopping cannot meet the agreed distance limit.
- [ ] Confirm completion is reported exactly once with the matching sequence
  ID, and a later command cannot be mistaken for the earlier completion.

### Exit check

- [ ] Mean, maximum and spread of distance error meet the agreed limits.
- [ ] Lateral drift meets the agreed limit on the defined test surface.
- [ ] Consecutive commands remain relative to their own encoder origins.

## Stage 7 - Encoder-based turn calibration

Goal: make navigation turns reliable for the barcode/controller path.

### Tasks

- [ ] Begin with the physical track width, then determine effective track width
  from measured pivot turns on the actual track surface.
- [ ] Run at least ten `-90` degree left turns.
- [ ] Run at least ten `+90` degree right turns.
- [ ] Run at least ten `180` degree U-turns.
- [ ] Test turns after straight driving, after another turn and after a stop;
  do not test only the first command after boot.
- [ ] Record angle error, completion time, overshoot, wheel counts, PWM,
  invalid transitions, battery voltage and surface.
- [ ] Add a bounded turn approach profile if needed for accuracy.
- [ ] Confirm both wheels are stopped before Motion publishes `COMPLETE`.
- [ ] Set the Vehicle Controller turn timeout to at least twice the measured
  worst normal turn time, while retaining a finite fault timeout.

### Exit check

- [ ] Left, right and U-turn mean/maximum errors meet the Stage 0 limits.
- [ ] Negative and positive turns both complete with the correct sequence ID.
- [ ] Barcode turn requests can use the public Motion API without a timing
  workaround.

## Stage 8 - Stop and fault verification

Goal: prove that failures stop safely and are reported to the controller.

### Required fault tests

- [ ] Operator/Vehicle Controller stop while driving forward, reverse and
  turning; measure command-to-brake latency and stopping distance.
- [ ] Stop immediately followed by a new command.
- [ ] Command timeout.
- [ ] Single left-wheel stall and single right-wheel stall.
- [ ] Disconnected left encoder and disconnected right encoder.
- [ ] Excessive/invalid encoder transitions or simulated impossible delta.
- [ ] Out-of-range linear speed, angular rate, distance and turn inputs.
- [ ] Queue saturation.
- [ ] Low battery/brownout behaviour within the safe test range.
- [ ] Confirm both motors brake for every fatal Motion fault.
- [ ] Confirm the Vehicle Controller maps stall, encoder and other Motion
  faults to the intended telemetry bits and enters `EMERGENCY_STOP`.
- [ ] Confirm the documented recovery/acknowledgement procedure; a stale fault
  must not silently disappear while the hardware problem remains.

### Exit check

- [ ] Every failure has a bounded response, motor-safe state and visible code.
- [ ] Stop latency/stopping distance meet the Stage 0 limit.
- [ ] Fault tests can be repeated without undefined queue or state behaviour.

## Stage 9 - Integration with the current team branch

Goal: prove Motion remains stable with the current Vehicle Controller and all
merged subsystem workloads.

### Current integration contract to preserve

- The Vehicle Controller runs every 20 ms and is the only high-level motion
  requester.
- Normal speed is currently 200 mm/s; hump speed 100 mm/s; line-recovery speed
  120 mm/s.
- Line steering currently maps to angular rate using
  `steer * 400 mdeg/s` (positive is right) and resends after an 8/127 steering change. These are
  provisional track values and may need coordinated VC/Motion tuning.
- Turns use `-90000`, `+90000` and `180000` mdeg at 100 mm/s.
- Obstacle reverse uses -100 mm at -80 mm/s.
- Buddy 3 line/barcode and Buddy 5 obstacle-ahead inputs are connected.
- Buddy 5 scan/bypass motion is not implemented yet; the present controller
  fails safe by stopping and timing out.
- Buddy 4 hump input and the mission finish source are not fully connected.

### Pairwise integration order

- [ ] Motion + encoders: repeat the Stage 5-8 regression.
- [ ] Motion + Buddy 3 line following: tune steering sign/gain and confirm no
  other module writes PWM.
- [ ] Motion + barcode/controller: LEFT, RIGHT, STRAIGHT and U-TURN, five to ten
  successful repetitions each.
- [ ] Motion + Buddy 4: verify vibration/motor noise does not corrupt IMU data
  and the agreed hump speed command remains stable.
- [ ] Motion + Buddy 5/controller: stop, reverse-rescan and the agreed bypass
  sequence. The Vehicle Controller must compose/approve the manoeuvre; Buddy 5
  only recommends it.
- [ ] Add WiFi/MQTT: repeat speed, turn, stop and timing tests while telemetry
  and reconnect activity are running.
- [ ] Verify Motion telemetry units, sequence IDs, state and fault values.
- [ ] Measure Motion loop jitter/overruns under the full integrated workload.
- [ ] Repeat five to ten times at every integration step and fix regressions
  before adding the next subsystem.

### Exit check

- [ ] Pairwise tests and the full `vc_car` build/run meet their agreed limits.
- [ ] No task other than Motion owns drive hardware.
- [ ] Network and sensor load do not cause false stalls, missed encoders or
  unsafe delayed stops.

## Stage 10 - Final regression and hand-off

Goal: close the Motion deliverables and prepare a reviewable merge.

### Final checks

- [ ] Run all Motion and Vehicle Controller host tests.
- [ ] Build all Motion applications and the team's exact final `vc_car`
  profile; record compiler warnings and binary size.
- [ ] Repeat the final-car speed, 500/1000/1500 mm distance, 90/180 degree turn
  and fault tests.
- [ ] Calculate mean, maximum error and standard deviation/spread.
- [ ] Update `docs/project/BUDDY2_MOTION.md` with final constants and procedures.
- [ ] Update the project gate/evidence documents without overwriting another
  buddy's results.
- [ ] Store raw CSV/logs under `evidence/motion/`; use the existing CSV header
  and add a setup/readme if extra columns are required.
- [ ] Remove stale comments that describe provisional `test_integrate` values
  as awaiting Buddy 2 confirmation.
- [ ] Review the diff against `main`, keep each commit focused, and obtain team
  review before merging.

### Definition of done

- [ ] The public API is defined, non-blocking and used only by the controller.
- [ ] Independent motor/encoder tests are repeatable.
- [ ] Loaded calibration and PID results are recorded.
- [ ] Straight distance and all turn directions meet agreed accuracy limits.
- [ ] Stop, timeout, stall, encoder and queue failures are deterministic.
- [ ] Integrated tests pass with line, barcode, obstacle, IMU and WiFi loads.
- [ ] Gate G4 evidence includes mean, maximum and spread, not only a successful
  demonstration video.

## Evidence record for every hardware run

Record at least:

- date/time, tester and firmware commit;
- application/build profile and board;
- exact vehicle mass/configuration and power-bank position;
- battery voltage/state, surface and available run length;
- calibration constants and PID gains used;
- command/target, expected result and observed result;
- repetitions, failures and fault codes;
- raw CSV/log/video path.

Use `evidence/motion/README.md` as the base CSV format. Never replace raw data
with only a screenshot or a statement that the car “worked”.

## Stage-clearance note template

Copy this block below the relevant stage when it is completed:

```text
Cleared on:
Tester:
Commit:
Vehicle configuration:
Tests/builds run:
Result summary:
Evidence paths:
Known limitations carried forward:
```
