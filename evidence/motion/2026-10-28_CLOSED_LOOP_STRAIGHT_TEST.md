# Closed-loop straightness test - 2026-10-28

## Latest update: external-power fault cleared after timer-tick initialization

The RP2040 BSP did not explicitly configure the watchdog `TICK` generator that
supplies the hardware timer reference. A teammate-provided `cpu_clock.c` added
`ENABLE` plus `XOSC_MHz` cycles after `CLK_REF` was configured. The repository
now contains the same change in
`kernel/sysdepend/pico_rp2040/cpu_clock.c`, so every boot establishes a 1 us
timer tick instead of relying on state established elsewhere.

The post-fix computer-USB button run completed successfully:

```text
[button-diag-03] init_core=1 seq=1 state=3 fault=0 detail=0 wheel=0
[button-diag] usb_state_at_print=3 (1=no host, 2=mounted, 3=serial open)
[button-diag] data_off_trial=0 usb_irq_start=131 usb_irq_end=131
[button-diag] dt_ms=0 prev_us=0 sample_us=17145319 read_us=2
[button-diag] prev=0/0 count=4220/4233 delta=0/0
[button-diag] irq_prev=0 irq=8438 invalid=0/0
[button-diag] last_speed=105/105 pre_fault_pwm=0/0 pwm=0/0 travel=301 target=300
```

The same rebuilt image then ran from powerbank through Robo-Pico VIN without
the former multi-group encoder-fault beeps. The tester measured approximately
350 mm of physical travel for the 300 mm command. This is a first successful
external-power run and strong evidence that the missing timer initialization
caused the power-dependent false plausibility fault. It is not yet a final
repeatability result. The encoder estimate was 301 mm while the approximate
physical measurement was 350 mm, an overshoot of about 50 mm or 17%; distance
calibration therefore remains open and must not be changed from one approximate
measurement.

The successful counts are consistent with the earlier passing USB runs:
4220/4233 now versus 4224/4199, 4272/4148 and 4247/4186 previously. Total
counts differ by only about 0.2-0.4%. Keep the encoder safety check enabled.

The sections below preserve the earlier investigation in chronological order.

Related historical audit: [encoder fault audit](2026-10-28_ENCODER_FAULT_AUDIT.md).
Before the watchdog-tick fix, computer USB also produced one raised-wheel fault
(599 left counts in 14063 us). Earlier USB baseline and USB-data-isolation
passes were reported by the tester. Preserve these entries as chronological
evidence; the post-fix USB and VIN results above are the latest evidence.

The tester subsequently repeated the raised-wheel run with the powerbank and
reported the same beep failure. The matching raised-wheel computer-USB run
completed successfully. No raw powerbank diagnostic line was provided with
that repeat, so only the comparative result is recorded here.

## Earlier review: cause was unresolved before the watchdog-tick fix

### Latest result and next diagnostic: button-diag-03

The tester reports that a wall adapter labelled 5 V / 1 A also produces
`5,1,1,2,3` with only the left wheel visibly moving. The code identifies a
left count-delta failure in a 6-15 ms interval; the largest absolute wheel
delta is 251-500. This repeats the failure on a second external supply.
The 1 A label does not establish adequate motor-start current, so supply
effects are not ruled out, but the original powerbank is not established as
the cause. The previous timing fix did not resolve physical behaviour.
Neither the beep buckets nor a lack of invalid quadrature transitions alone
can establish whether the counts, interval, physical speed bound or electrical
signals are correct. Do not declare the hardware defective from these tests.

Button-diag-03 adds a controlled software USB-data detach test (`n`): retain
computer power and the same cable/board socket, request soft detachment from
the USB service task, wait for acknowledgement, settle for one second, and
run the same bounded 300 mm command with wheels raised. GP21 still stops.
The USB service owns TinyUSB calls and handles requests through its existing
SMP lock; no kernel code changes. This interface is compiled only for the
button diagnostic (`TM_USB_HOST_DIAGNOSTIC=1`), not normal vehicle firmware.

All exits restore USB, including command rejection, fault and operator stop.
The result stays in RAM for `p` after Serial Monitor reconnects; do not reset
or remove power before retrieving it. Reports include USB interrupt counters
at start/end to check whether USB activity actually ceased during the trial.
The service task continues sleeping/polling; this tests soft detach, not every
electrical difference between a charger and a computer. A failing detached
run provides exact counts/timestamps on unchanged computer power. A passing
run would not prove a particular supply or cable defective.

Verification: 95 host checks pass (65 control, 19 Motion loop, four encoder
snapshot, seven isolation orchestration). The seven new tests cover refusing
to move without a host or detach acknowledgement, preserving the fault latch,
and restoring USB after completion/fault/rejection/operator stop. Both normal
USB firmware (without diagnostic hooks) and button firmware build without
emitted warnings/errors. Enumeration and physical operation remain untested.

Next steps: flash button-diag-03, keep computer USB connected and VIN feed
disconnected, both wheels raised. Run GP20 once as a baseline. If that faults,
send its report first. Otherwise send `n` in Serial Monitor; COM disappearing
is expected. Wait for motors and beeps to finish (allow about 20 seconds),
reopen the serial port, send `p`, and provide all diagnostic lines. Do not
reset or disconnect power. No more power-source swaps or PID retuning yet.

### Latest code correction: button-diag-02

Code review found that `motion_loop()` read its timestamp before acquiring
the encoder snapshot. A preemption between those operations paired newer
counts with an older time, potentially creating a false count-delta fault.
A regression injecting 100 ms of delay and 800 counts after an initial 20 ms
interval failed with the old code. The same test passes with the correction:
the actual sample interval is 120 ms (about 477 mm/s), not 20 ms.

`motion_encoder_get_snapshot()` now includes a 64-bit acquisition timestamp
inside its existing sequence check. An encoder ISR during acquisition forces
both counts and time to be retried. Motion uses this timestamp for speed,
plausibility and control elapsed time. A shared raw high/low/high timer reader
follows the local Pico SDK `time_us_64()` algorithm, avoiding the shared
TIMELR/TIMEHR latch. Snapshot acquisition duration remains diagnostic evidence.
PID tuning, wheel geometry and count-fault limits have not changed in this fix.

Verification: 65 control helper checks, 19 Motion-loop checks and four actual
encoder-reader checks pass with host warnings treated as errors. The button
firmware builds without emitted warnings/errors as `[button-diag-02]`.
Tests cover delayed acquisition, coherent retry during an ISR update, timer
rollover, and immediate braking for a real count spike after the correction.
This proves the software defect and its regression fix, not that it caused
the observed powerbank failures. Physical validation remains open.

The first raised-wheel powerbank run on button-diag-02, using the tester's
right powerbank port through Pico micro-USB, still produced `5,1,1,2,4` and
only the left wheel was visibly moving. This is a left count-delta fault, a
6-15 ms sample interval and a maximum delta in the 501-1000 bucket. The timing
fix therefore did not clear the powerbank failure. The check stops the motors,
but its activation does not establish a hardware cause; software and electrical
possibilities remain open. Repeated unchanged port tests are not useful.

The wall-adapter test above supersedes the request to try another power source.
Proceed with the button-diag-03 USB-data comparison, preserving fault limits.

This review supersedes the causal conclusions in the chronological notes
below. The latest soft-start runs travelled about 130 mm with changing heading
and codes `5,1,2,2,4`, then `5,1,2,3,5` twice. These identify a right-wheel
count-delta fault, a 6-15 ms or 16-30 ms sample, and a largest absolute delta
of 501-1000 or above 1000 counts. The buckets do not identify the underlying
electrical or software cause, nor prove that both wheels exceed 1000 counts.

The successful USB run used `motion_closed_loop_test`; the failing powerbank
runs used `motion_closed_loop_button_test`. Both supply AND application changed.
This is not a controlled power-supply comparison. Powerbank noise, a timer
defect, a one-sample discontinuity and insufficient restart torque were earlier
hypotheses, not confirmed diagnoses. A single successful encoder-distance run
also does not establish physical distance accuracy or straightness.

The same `motion_closed_loop_button_test` image was then run over computer USB.
It completed with `state=3`, `fault=0`, `count=4214/4204`, encoder travel
`300/300 mm`, zero invalid transitions and no fault detail. The tester reports
about 160 mm of chassis travel, followed by about 145 mm on a repeat. The
tester carried the computer alongside the car, so cable drag is not established
as the cause. Retain these measurements as an unresolved physical-distance
discrepancy; do not recalibrate from the completion flag alone. Zero invalid
transitions does not independently validate counts or chassis travel.

The same image on powerbank then turned slightly right on the spot and faulted
with `5,1,2,3,4`: encoder count-delta check, right wheel, 16-30 ms interval,
and largest absolute wheel delta 501-1000 counts. The magnitude bucket is the
maximum across both wheels, not a separate exact right-wheel measurement.
This establishes different observed behaviour under USB and powerbank
operation, but does not separate electrical supply effects from USB connection
or software timing effects. It does not prove a defective encoder.

The requested wheels-off-floor powerbank test then produced `5,1,1,2,3`
twice, including after reset: left-wheel count-delta check, 6-15 ms interval,
and maximum absolute wheel delta 251-500 counts. The tester thought both wheels
moved slightly on the first attempt; only the left visibly moved on the second.
The fault therefore persists without floor load and has affected either side.
Floor resistance and a right-wheel-only defect are insufficient explanations.
Early fault braking can also truncate visible wheel startup; this observation
alone does not establish a failed right motor. Supply, signal integrity and
software count/timestamp handling remain unresolved.

The tester originally described powerbank operation through the Robo-Pico VIN
terminal, then explicitly confirmed that the later two-port comparison below
was already through the Pico micro-USB socket. Do not relabel all earlier runs
as micro-USB tests. For the later comparison, computer and powerbank use the
same board input, while source and USB-host presence differ. Cytron documents
both inputs as supported; the later failures cannot be attributed solely to
the VIN connection.
Reference: https://www.cytron.io/p-robo-pico-simplifying-robotics-with-raspberry-pi-pico

The raw-PWM duration loop sums requested `tk_dly_tsk(5)` delays. RTOS delays
and overhead can make actual wall time longer. Therefore the earlier conversion
of 91 cm into 910 mm/s is unverified and cannot justify a physical-speed bound.
The 1000 mm/s configured bound remains provisional, not measured calibration.

Current changes:

- Restore immediate braking for an implausible sample. The failed three-strike
  workaround allowed untrusted cumulative counts into distance and steering,
  even when speed feedback held its last accepted value.
- Freeze the first fault's previous/current counts, timestamps, IRQ totals,
  invalid transitions and pre-brake PWM until explicit stop acknowledgement.
- Add `[button-diag-01]` serial reports to the SAME GP20 test image, emitted
  only after braking. Idle `p` replays the saved report; `h` shows help/report.
- Keep current motor tuning for the comparison. No new torque adjustment.
- Add a host harness executing the actual `motion_loop()` with mocked hardware
  and RTOS calls. Fourteen checks pass for immediate braking, rejection before
  PID/distance updates, fault evidence preservation, acknowledgement/retry and
  invalid-transition braking. Existing 65 helper tests also pass. These do not
  emulate real IRQ timing or certify physical operation.

The requested powerbank-through-micro-USB comparison has already been done.
Do not repeat it with the old image as a new diagnostic step. The next test is
the corrected button-diag-02 firmware described above.

Powerbank port comparison: with the same Pico micro-USB arrangement, using the
port the tester calls the left socket made the right wheel move slightly and produced
`5,1,2,2,3` (right count-delta check, 6-15 ms interval, maximum delta 251-500).
Using the right socket made the left wheel move slightly and produced
`5,1,1,2,3` (left count-delta check, same interval and magnitude bucket).
Both motor channels share the board's motor supply; there is no firmware
mapping from powerbank port to wheel. One attempt per port does not establish
a repeatable causal port-to-wheel relationship. Electrical transients and
software timing remain hypotheses; neither encoder is proved defective.

The tester initially misread the label, then confirmed both powerbank outputs
are `5 V` (the current rating remains recorded as 2.1 A per port). This is a
compatible nominal supply rating, not a measurement under motor load.
Port-dependent wheel startup remains unexplained. Cytron specifies VIN
3.6-6 V and motor/servo voltage equal to the selected input:
https://www.cytron.io/p-robo-pico-simplifying-robotics-with-raspberry-pi-pico

## Firmware and setup

- Firmware: `motion_closed_loop_button_test`
- Command: encoder/PID-controlled forward move, 300 mm target at 100 mm/s
- Board: Robo-Pico with complete 0.665 kg car
- Power: powerbank only; USB disconnected during movement
- Surface/lane:
- Tester: Suen Hong

GP20 starts one 300 mm closed-loop move. The controller applies startup boost,
per-wheel PID and encoder-normalized straight correction. GP21 stops the active
move. A one-beep completion means the encoder distance target completed. The
diagnostic codes are: two operator stop, three timeout, four stall, five
encoder fault, six invalid command, seven hardware/configuration and eight
command rejection.

The first provisional run stopped at approximately 130 mm; its following
command produced four beeps under the older firmware, meaning the earlier run
had left a latched Motion fault. Based on the raw loaded sweep, startup boost
was reduced from 350 permille / 400 ms to 225 permille / 250 ms before the
repeat.

Repeated runs with the reduced boost stopped at approximately 95 mm and gave
five beeps, identifying an encoder fault. The encoder delta plausibility check
was incorrectly based on the 300 mm/s maximum *commanded* speed. The raw loaded
sweep measured approximately 910 mm/s at 350 permille, and even 225 permille
covered 530-560 mm in one second. A launch transient can therefore legitimately
exceed the check's old effective 600 mm/s limit. The check now uses a separate
1000 mm/s physical-speed bound with its existing two-times transient margin;
invalid quadrature-transition monitoring remains unchanged.

The repeat still produced the five-beep encoder code. The powerbank-only test
therefore now emits a second beep group after an encoder fault: one means count
delta plausibility, two speed calculation, three invalid quadrature transitions
and four unknown encoder detail. The public Motion fault remains
`MOTION_FAULT_ENCODER`, so Vehicle Controller handling is unchanged.

The diagnostic repeat produced five beeps followed by one: count-delta
plausibility. The next diagnostic build retains the exact fault context in
`motion_status_t` and emits three further groups for this subtype:

- wheel: one left, two right, three both, four unknown;
- sample interval: one 1-5 ms, two 6-15 ms, three 16-30 ms, four above 30 ms;
- largest absolute delta: one <=100, two <=250, three <=500, four <=1000,
  five above 1000 counts.

This is diagnostic data only. Do not raise or disable the plausibility check
again until the offending wheel, interval and delta bucket are known.

The detailed result was `5, 1, 3, 3, 5`: both wheels, a 16-30 ms sample and
more than 1000 counts. Both calibrated wheels cannot physically travel at the
implied multi-metre-per-second speed, and both failing together rules against
an isolated encoder channel. The controller now reads the RP2040 latched timer
in its documented low-then-high order and requires three consecutive
implausible samples before latching a count-delta fault. Each non-terminal
sample advances the count/time baseline, so one scheduling discontinuity
cannot make every later delta accumulate from a stale sample. A sustained
impossible signal still faults within approximately 60-90 ms; invalid
quadrature transitions retain their independent immediate check.

With that change the vehicle no longer immediately reported the previous
fault, but it travelled about 100 mm, curved roughly 75 degrees, then wiggled
forward with the right wheel stationary. Inspection found two coupled recovery
defects. A non-terminal implausible delta was still converted into a very high
PID speed, causing motor output to be cut, and the startup boost was marked
permanently exhausted as soon as a wheel first moved. The controller now holds
the last accepted per-wheel speed for a rejected single sample and advances
the sample baseline. It also re-arms the bounded 225-permille boost after
genuine wheel movement so either wheel can overcome static friction again if
it subsequently stops. The one-second independent-wheel stall fault remains.

The powerbank repeat nevertheless reproduced `5,1,3,3,5` after approximately
110 mm. Because three consecutive samples now report simultaneous deltas above
1000 counts, beep buckets are no longer sufficient for safe diagnosis. The
USB `motion_closed_loop_test` now provides command `g` for the same 300 mm at
100 mm/s move and prints exact encoder fault detail, wheel mask, elapsed
milliseconds, left/right deltas, cumulative counts, speed, PWM, invalid
transitions and IRQ count after braking. The next run is computer USB power
only; do not connect the powerbank simultaneously.

The USB diagnostic completed successfully: final state `COMPLETE`, `fault=0`,
encoder travel 301 mm against the 300 mm target, and zero invalid transitions.
Across the readable samples both wheels settled near 85-107 mm/s; steady PWM
was approximately 190-202 left and 184-193 right. The left/right cumulative
counts remained close. Several later lines were mangled while the complete
33-line capture was dumped through USB CDC, so the diagnostic now paces each
printed sample by 10 ms.

Because the same controller completes on USB power but reports simultaneous
count spikes only on the powerbank, the remaining fault is power-condition
specific. The loaded raw sweep proved 200 permille starts both wheels, and the
successful closed-loop run settled near 190-200 permille. Startup boost is
therefore reduced from 225 to 200 permille to reduce the powerbank current/noise
transient. If the exact fault persists at 200, investigate power integrity,
grounding and encoder wiring rather than relaxing encoder safety limits.

The 200-permille powerbank repeat produced `5,1,3,2,4`: both wheels, a
6-15 ms interval and a 501-1000-count delta. It travelled about 70 mm and
initially turned slightly right. Reducing the boost therefore reduced both the
fault magnitude and travel compared with the 225-permille result, supporting a
shared startup power transient. Motion now applies a symmetric drive-output
slew limit of 25 PWM permille per nominal 20 ms cycle. This ramps both motors
to the required loaded torque without changing immediate brake behaviour.
If simultaneous encoder spikes persist after this soft-start, stop software
tuning and inspect power/ground integrity and encoder-wire routing.

## Runs

| Trial | Distance target | Completed beep | Travel endpoint | Drift (car's side) | Left/right start | Notes |
| ---: | ---: | ---: | ---: | --- | --- | --- |
| 1 | 300 mm | No fault groups | ~350 mm | Not recorded | Both drove | Post-watchdog-tick fix; powerbank to VIN |
| 2 | 300 mm | | | | | |
| 3 | 300 mm | | | | | |
| 4 | 300 mm | | | | | |
| 5 | 300 mm | | | | | |

## Review

- Straight on how many trials: not recorded for the first post-fix run
- Right drift on how many trials:
- Left drift on how many trials:
- Any fault beep: none reported on the first post-fix VIN run
- Controller change required: timer fix retained; distance calibration pending
