# Buddy 2 encoder fault audit - 2026-10-28

## Latest resolution update

This audit's original hardware-signal conclusion is superseded by a later BSP
comparison. The teammate-provided `cpu_clock.c` explicitly initialized the
RP2040 watchdog `TICK` generator after configuring `CLK_REF`; the repository
version did not. That tick is also the hardware timer reference used by Motion
to judge encoder counts against elapsed time.

The missing initialization has been added to
`kernel/sysdepend/pico_rp2040/cpu_clock.c`. The post-fix computer-USB run
completed at `travel=301`, `target=300`, counts 4220/4233, speed 105/105 mm/s,
invalid transitions 0/0 and `fault=0`. The same rebuilt button image then ran
from powerbank through VIN without the previous encoder-fault beep groups.
This is strong evidence that the power-dependent plausibility fault was caused
by an inconsistent timer timebase, although repeat testing is still required.

The tester measured about 350 mm of physical travel. The safety fault appears
cleared, but distance calibration is still open because the encoder estimate
was 301 mm and the approximate physical result was about 17% longer.

## Original conclusion before the timer fix

The new report was a left count-plausibility fault, correctly stopped by
Motion. At this point in the investigation no reviewed code defect had been
shown to cause the burst, so the accepted encoder phase stream required
verification against actual wheel rotation.
Do not increase the fault threshold, change PID gains, or insert arbitrary
debouncing to make the fault disappear.

### Controlled power comparison update

The tester then repeated the same closed-loop button test with both wheels
raised and the powerbank connected. It produced the same encoder-fault beep
behaviour. The otherwise matching raised-wheel computer-USB run completed at
300 mm with no fault. This removes floor load as the differentiating condition
for these two runs and makes the external-power path the leading shared factor.
The exact raw diagnostic line from the powerbank repeat was not supplied, so
the wheel, interval and delta should not be inferred from the beep pattern.
This is strong comparative evidence, but it does not distinguish a power-rail
disturbance, motor-supply switching interference, ground/reference problem,
or another difference in the external connection.

The leading investigation is encoder signals, connectors, common ground and
motor-related interference, followed by verification of actual unloaded speed
and the provisional physical-speed bound. This is a diagnostic priority, not
a proven hardware diagnosis. A short fast rotation can look like slight motion;
observation alone cannot rule out genuine unloaded overspeed.

## User evidence and test conditions

Earlier computer-USB baseline: state 3, fault 0, encoder travel 300/300 mm,
counts 4224/4199, fault deltas 0/0. USB-data isolation also passed:
data_off_trial=1, counts 12729/12775, fault 0. These are user-reported results;
no new hardware test was performed during this audit. Successful-run zero fault
deltas are cleared diagnostic fields, not proof of zero encoder activity.

The user confirms the earlier successful computer-USB test was on the floor.
The current failing computer-USB test is on Robo-Pico with wheels raised:
left wheel moved slightly, not visibly rapidly; right wheel remained still.
Thus the latest comparison changes mechanical load as well as test date.
The previous floor runs are recorded in BUDDY2_MOTION_CATCHUP_PLAN.md, including
an unresolved difference between encoder completion and physical floor travel.

Latest supplied report:

```text
[button-diag-03] init_core=1 seq=1 state=5 fault=3 detail=1 wheel=1
[button-diag] usb_state_at_print=3 (1=no host, 2=mounted, 3=serial open)
[button-diag] data_off_trial=0 usb_irq_start=151 usb_irq_end=151
[button-diag] dt_ms=14 prev_us=422617 sample_us=436680 read_us=0
[button-diag] prev=175/2 count=774/2 delta=599/0
[button-diag] irq_prev=177 irq=776 invalid=0/0
[button-diag] last_speed=956/0 pre_fault_pwm=125/157 pwm=0/0 travel=6 target=300
```

| Evidence | Interpretation |
| --- | --- |
| 436680 - 422617 | 14063 us actual sample interval; controller uses 14 ms |
| 774 - 175 | 599 new left counts; right delta is zero |
| 776 - 177 | 599 ISR samples with owned pending edge bits |
| 599 counts / 2536 counts per wheel revolution | About 0.236 wheel revolutions in 14.063 ms, equivalent to 1008 RPM |
| Current left conversion = 4684 Q16 mm/count | About 42.81 mm in the interval, equivalent to 3044 mm/s |
| Current plausibility calculation | Allows up to 406 counts in 14 ms, including its 2x speed and 16-count margins |
| invalid=0/0 | No sampled two-bit illegal transitions; does not certify electrical signal quality |
| last_speed=956/0, travel=6 | Previous accepted speed/travel, deliberately retained when the new sample is rejected |
| pre_fault_pwm=125/157 | Commands were 12.5% / 15.7%; these are software values, not measured motor voltage/current |
| pwm=0/0 | Brake command recorded after fault |
| read_us=0 | Snapshot completed within microsecond timer resolution; not a zero sample interval |

Each ISR can add at most one left count. Assuming normal memory execution,
599 net positive counts in 599 counted ISR visits requires a positive left
quadrature transition on every visit. Repeated delivery of the same IRQ with
unchanged GPIO levels cannot do this. Simple chatter on only one phase normally
alternates the count direction. Both-phase interference, actual rotation, or
sampling/aliasing of more complex signals remain possible.

The beep code `5 1 1 2 4` means encoder fault / count delta / left wheel /
6-15 ms / maximum absolute delta 501-1000. `5 1 2 2 4` selects the right wheel.
The final bucket is the maximum of both wheel deltas; it is not above 1000.
An above-1000 delta has final beep group 5.

## Software review

Scope: motion configuration, types and APIs, quadrature and control math,
encoder ISR/snapshot/clock, motor PWM driver, RTOS task, simulation/motor/
encoder/closed-loop/button/Stage 4 test apps, host tests, board startup and
underlying GPIO/PWM/interrupt helpers. This is not a kernel-wide qualification.

- Motor GP8/9 and GP10/11 match the user wiring. Physical left phases are
  GP16/17, deliberately decoded in logical order GP17/16 for forward sign.
  Right phases are GP4/5. Changing left logical order only reverses sign.
- Encoder masks correctly cover groups 0 and 2. Input mode, pull-ups and
  Schmitt input are set. Pull-ups are explicitly provisional: the actual
  encoder model/output type and its valid 3.3 V operating range remain unknown.
- The ISR acknowledges owned edge bits and reads phase levels. Its edge-clear
  mechanism agrees with the [RP2040 GPIO documentation](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#hardware_gpio).
  This register review does not validate physical waveforms.
- The sequence-protected snapshot pairs counts, IRQ count and raw timer time.
  The prior pre-snapshot timestamp defect is already fixed. init_core=1 agrees
  with the motion task's processor assignment (T-Kernel processor 1 = core 0).
- Motor PWM slices are 4 and 5; GP22 buzzer uses slice 3. The button test does
  not start the other Buddy tasks. No pin or PWM-slice conflict was found in
  this profile. The robot board startup avoids I2C on GP8/9 and uses LED GP19.
- Count validation precedes PID/distance updates; the first fault freezes
  its evidence and commands both motors to brake. The new sample reproduces
  the expected fault in the actual Motion-loop host harness.
- USB printing in the button test happens after the motion result. The earlier
  data-off pass weakens a requirement-for-USB-host theory; it does not prove
  every electrical power arrangement is sound. Current USB IRQ counters do
  not change during the failed run.

Separate robustness concerns found, without evidence linking them to this burst:

- Motor init/apply ignore several BSP return values. Valid present pins avoid
  the BSP's invalid-pin errors, but hardware error propagation is incomplete.
- Encoder init enables the IRQ before setting encoder_ready. An edge in that
  small window can invoke a handler that returns without acknowledging it.
  This is an initialization robustness issue, not an explanation demonstrated
  for a run already producing valid snapshots.
- Signed encoder accumulation can overflow after long operation. Counts below
  1000 in the new report cannot trigger that boundary.
- The seqlock reader has no retry bound under continuous ISR traffic; the
  current read_us=0 does not show that condition. Shared GPIO IRQ ownership
  must also be revisited before adding another handler on the same core.
- Motion uses integer milliseconds and caps elapsed time at 1000 ms. Flooring
  14.063 ms to 14 ms changes the estimate by under 0.5%, insufficient to explain
  this fault. Long scheduling gaps need separate treatment before integration.
- The raw motor-test watchdog restarts its timer on any input key. Use single
  isolated key presses in the procedure below; do not stream commands.

## Next bench procedure

Use one fixed computer USB source/cable first. Keep the wheels raised. Record
which exact UF2 is flashed and change only one connection or condition at a
time. Power off before changing wiring. Keep the plausibility stop enabled.

1. Disconnect only the two-wire drive-motor outputs, preserving encoder 3V3,
   ground and both phase pairs. Flash `APP=motion_encoder_test` for the robot
   board. Hold both wheels still for 10 seconds. Counts should stay fixed.
   Gently move each connector/harness without turning a wheel. Count changes
   indicate an input/harness/ground problem even without motor switching.
2. Mark each wheel. Slowly rotate the left wheel forward exactly one revolution,
   then the right. Record before/after L/R counts and invalid transitions.
   Expect approximately +2536 on left only, then +2529 on right only. Repeat
   backward for negative counts. A turning right wheel with no right counts
   identifies a right encoder path problem independently of PID. Turn slowly:
   this diagnostic prints live and is not a high-rate accuracy test.
3. If these pass, power off and reconnect drive outputs. Use
   `APP=motion_motor_test`, default 100 permille. Send `1` once (left forward),
   wait for its automatic stop/report; then `3` once (right forward). Note
   which physical wheel turns and which encoder count changes. `X` brakes.
   This raw-PWM bench app has a three-second limit, not closed-loop encoder
   protection. Do not raise duty just to force a result. A moving motor must
   correspond to its own count channel; a stationary wheel should not collect
   substantial net counts from the other motor running.
4. If a fault remains, capture A/B at the Pico pins and encoder 3V3 relative
   to encoder ground during motor startup with a scope/logic analyser. The
   reported sample corresponds to about 42.6k decoded transitions/s. Compare
   A/B ordering and frequency with visible/measured shaft rotation; correlate
   disturbances with motor PWM. A multimeter cannot establish absence of
   brief supply dips or fast interference. Obtain the encoder model before
   choosing external pull-ups, RC filtering or changing encoder voltage.
5. If needed, with power off, exchange only the complete A/B signal pairs
   between the two encoder GPIO pairs, leaving power/ground and motor wiring
   unchanged. Use the encoder-only manual test to see whether the defect
   follows the encoder/harness or stays with a Pico input pair. Do not run
   closed-loop motion with crossed feedback; restore and verify wiring first.

If waveforms and measured unloaded RPM confirm genuine speed above the current
bound, characterize that speed and startup control before changing the bound.
If physical motion is slow while A/B cycles rapidly, fix the electrical/encoder
path first. Floor-loaded PID tuning resumes only after repeatable encoder tests.

## Validation and changes at the time of the original audit

- Added seven actual-ISR register-mock checks and one exact fault replay in
  the Motion-task harness. All 103 Motion checks pass with
  `-std=c11 -Wall -Wextra -Werror -O2`.
- Forced rebuild of `APP=motion_closed_loop_button_test BOARD=robo_pico_robot
  SMP=1 CONSOLE=usb_cdc` succeeds, producing ELF/UF2. Arm GCC 15.2.Rel1,
  SDK 2.3.1; explicit `-Wall -Wextra` produces 73 warnings in existing
  non-Motion sources, no warnings attributed to motion_*.c, and no errors.
  This is not a warning-free whole-project build.
- Encoder-only and raw motor-test USB/SMP robot profiles also rebuild
  successfully, with no Motion-source warnings or errors. Their current UF2s
  are available under `build_make/`; neither was flashed.
- Local logs: `build_make/motion_audit_host.log` and
  `build_make/motion_audit_firmware.log`.
- No application or kernel behavior had changed during the original audit;
  source changes were regression tests, stubs/build integration, ignore entries
  and documentation. This is superseded by the later watchdog-tick kernel fix.
- No board was flashed or hardware fix verified during the original audit.
  Subsequent post-fix USB and powerbank-to-VIN runs passed as recorded above.
