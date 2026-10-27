# Buddy 2 raised-wheel manoeuvre defect - 2026-10-27

Branch: `buddy2-motion-catchup`  
Firmware: `motion_closed_loop_test`, Robo-Pico, SMP, USB CDC  
Setup: vehicle held with the wheels clear of the surface; supply voltage not
measured. Source data was the tester's pasted serial log.

## Observed failure

- The first `l` command completed at `travel=116`, `target=113`, but the wheel
  speeds were strongly asymmetric. The intended targets were -60/+60 mm/s;
  the captured behaviour approached approximately -56/+165 mm/s.
- The `u` command ran much too quickly. It approached approximately
  +288/-169 mm/s instead of the intended +60/-60 mm/s.
- `t`, both `m` commands, and repeated `l`/`u` commands printed `complete`
  with zero captured samples and appeared not to run.
- No encoder invalid transitions or Motion fault codes were reported.

These results are a failed manoeuvre test and do not validate turn or distance
accuracy.

## Root cause and correction

`start_command()` stored the active command type for `SET_VELOCITY`, but did
not store it for `MOVE_DISTANCE` or `TURN_ANGLE`. The control loop therefore
continued using the previous/default velocity mode:

- `l` mixed a nominal linear speed of 60 mm/s with -90000 mdeg/s, producing
  approximately -53/+173 mm/s wheel targets;
- `u` mixed 60 mm/s with +180000 mdeg/s, producing approximately
  +288/-168 mm/s wheel targets;
- `m` used lifetime encoder travel, which already exceeded 100 mm, and thus
  completed immediately.

The Motion task now assigns `active_type = command->type` whenever a new
distance or turn manoeuvre starts.

The diagnostic application also treated the preceding command's stale
`COMPLETE` state as completion of the newly submitted command. Captures now
match `active_sequence_id` to the submitted command and ignore stale terminal
states. Explicit stops still wait for `STOPPED`.

The Motion status updater also overwrote a completed command's final relative
travel with lifetime encoder travel on the following idle cycle. Completed
travel is now preserved until the next command starts.

## Software verification after correction

- Buddy 2 Motion host suite: 47 checks passed.
- Vehicle Controller host suite: 150 checks passed, 0 failed.
- `motion_closed_loop_test` firmware with the automatic reversal and
  stop-priority diagnostics: built without warnings;
  text/data/bss = 97784/156/35556 bytes.
- Exact WiFi/MQTT `vc_car` firmware: built without warnings;
  text/data/bss = 366369/2708/69588 bytes.

## Hardware retest after correction

The corrected firmware passed the first staged retest using `l` (`-90` degree
left turn):

- left encoder/speed remained negative and right remained positive;
- settled samples 5-13 averaged approximately -76.4/+73.6 mm/s;
- the two wheel-speed magnitudes differed by approximately 3.8 percent;
- completion occurred at `travel=114`, `target=113`;
- the final status was `state=3`, `fault=0`, `pwm=0/0`;
- invalid encoder transitions remained `0/0`.

This confirms the command-mode and repeated-status corrections for the first
left-turn run. The roughly 74-76 mm/s unloaded speed is above the 60 mm/s
setpoint; loaded PID/feed-forward tuning belongs to later stages and must not
be derived from this raised-wheel run. Right-turn, U-turn and repeated distance
tests were run next and are recorded below.

## Additional repeated manoeuvre results

The tester subsequently ran three `t` commands, two `u` commands and two
consecutive `m` commands in the same boot.

| Command | Sequence | Result |
| --- | ---: | --- |
| `t`, +90 degrees | 3 | Completed at 113/113 mm, final PWM 0/0; wheel travel and speed were closely balanced |
| `u`, 180 degrees | 4 | Header reported complete with 32 samples; supplied sample text was truncated near the end, but the visible run was balanced and fault-free |
| `m`, 100 mm | 5 | Completed at 100/100 mm, final state 3, PWM 0/0 |
| `m`, 100 mm | 6 | Completed again at 100/100 mm without reboot, final state 3, PWM 0/0 |
| `t`, +90 degrees | 7 | Completed at 114/113 mm, final PWM 0/0; wheel travel and speed were closely balanced |
| `u`, 180 degrees | 8 | Header reported complete with 32 samples; supplied sample text was truncated near the end, but the visible run was balanced and fault-free |
| `t`, +90 degrees | 9 | Completed at 113/113 mm and PWM 0/0, but the left/right count changes differed by roughly 30 percent during the run |

All supplied samples reported `fault=0` and invalid encoder transitions of
`0/0`. The two consecutive distance commands confirm that command-relative
origins work across repeated manoeuvres.

Sequence 9 is not repeatable enough to accept as a cleared turn result. The
right reverse wheel started slowly while its PWM rose to approximately 220,
whereas the left wheel travelled farther. This can be caused by temporary
physical drag/contact or by low-speed unloaded motor variability. It also
shows that average wheel travel can reach the angular target despite an
off-centre pivot. Repeat once with both wheels visibly clear before deciding
whether turn synchronisation or fault criteria need a software change.

## Controlled right-turn repeat

The tester confirmed that neither wheel was obstructed and repeated `t` as
sequence 10:

- it completed at `travel=113`, `target=113`;
- the final status was `state=3`, `fault=0`, `pwm=0/0`;
- invalid transitions remained `0/0`;
- from the first captured sample to completion, the left/right encoder changes
  were 1400/1408 counts, a magnitude difference of approximately 0.6 percent;
- after start-up, the measured wheel speeds were closely matched and had the
  correct +left/-right signs.

This clean repeat means the sequence-9 imbalance is not a systematic command
mapping or encoder-sign defect. Because no physical obstruction was present,
retain it as evidence of intermittent low-speed unloaded starting behaviour.
Reassess it under the final power-bank load during motor characterisation and
PID/feed-forward tuning; do not tune the final car from raised-wheel USB tests.

## Timeout safety check

The tester ran `o` (a 500 ms velocity command) as sequence 12:

- the capture ended with `motion fault` after five samples;
- final status was `state=5` (`MOTION_STATE_FAULT`), `fault=1`
  (`MOTION_FAULT_TIMEOUT`), and `pwm=0/0`;
- encoder invalid-transition counts remained `0/0`.

This confirms the command timeout brakes both motors and latches the timeout
fault. Stop acknowledgement and post-fault recovery still need to be checked.

The tester then sent `x`, which returned `stop result=0`, and submitted `1`
again as sequence 13. The recovery command ran with closely matched visible
speeds (approximately 100/100 mm/s), `fault=0`, and invalid transitions `0/0`
through the supplied samples. The capture header reports that the subsequent
three-second safety stop completed with the motors braked. The pasted text
omits the final sample line, so its exact terminal state is not duplicated
here; the sequence acceptance and normal post-fault operation are confirmed.

The tester then ran `m` as sequence 15 in the same boot. It began at relative
`travel=5` despite the lifetime encoder totals being much larger, reached
`travel=100` against `target=100`, and ended with `state=3`, `fault=0`, and
`pwm=0/0`. The visible wheel speeds remained closely matched and invalid
transitions stayed at `0/0`. This confirms that a distance command starts from
its own encoder origin after preceding velocity/recovery activity.

## Automatic reversal check

The diagnostic `v` test drove forward as sequence 1, submitted reverse as
sequence 2 after 500 ms, and then applied the normal three-second safety stop.
The capture showed:

- forward speeds positive before the sequence change;
- a direction-change brake interval at the first reverse sample (`seq=2`,
  `pwm=0/0`);
- reverse speeds settling near -100/-100 mm/s with closely matched wheel
  behaviour;
- no invalid encoder transitions or Motion fault during the reversal;
- a completed safety stop with the motors braked, as reported by the capture
  header.

This confirms the reversal reset and motor direction-change brake path on the
raised-wheel diagnostic setup.

## Automatic stop-priority check

The diagnostic `p` test drove forward as sequence 1, called `motion_stop()`,
and immediately submitted reverse as sequence 2. The log reported
`stop requested result=0; replacement submitted seq=2`; sequence 2 then ran at
closely matched negative speeds near -100/-100 mm/s with `fault=0` and invalid
transitions `0/0`. The capture ended with its intended three-second safety
stop and motors braked.

This confirms that a command submitted after a stop request is not flushed by
that earlier stop. Because both commands are velocity commands, their
`target=0` is expected; they finish through the watchdog stop rather than a
normal distance/turn completion state.

## Timestamped emergency-stop check

The tester ran the new `e` diagnostic with the driven wheels raised and the
USB-CDC firmware. The test drove at 100 mm/s, requested `motion_stop()` after
500 ms, and compared the request timestamp with the Motion task's timestamp
when it published `STOPPED`:

| Measurement | Result |
| --- | ---: |
| Stop request timestamp | 18,279,082 us |
| Brake/`STOPPED` timestamp | 18,279,370 us |
| Request-to-brake latency | 288 us (reported as 1 ms rounded up) |
| Final state/fault | `state=4` / `fault=0` |
| Final PWM | `0/0` |
| Invalid encoder transitions | `0/0` |

The four captured samples show both wheels moving before the stop, with the
final sample at `t_us=18,279,370`, speed `0/0`, and both PWMs zero. This is a
clean single-run emergency-stop result. Repeat measurements and the team's
agreed maximum latency limit are still required before clearing the Stage 2
stop-latency gate.

### Diagnostic timestamp correction

Four subsequent runs produced `90097`, `90101`, `246`, and `90098` us. The
three approximately 90 ms values were not accepted as physical stop latency:
the original diagnostic used the continuously refreshed `timestamp_us` field,
so the 100 ms console-test polling period could report a later stopped sample.
The Motion status now includes a latched `state_timestamp_us`, set only when
the state changes, and the `e` diagnostic uses that field for `brake_t_us`.
The updated firmware must be flashed before collecting the repeatability set.

## Corrected five-run emergency-stop repeatability set

The tester flashed the corrected firmware and repeated `e` five times with the
wheels raised:

| Run | Sequence | Latency |
| ---: | ---: | ---: |
| 1 | 1 | 10,687 us |
| 2 | 2 | 204 us |
| 3 | 3 | 239 us |
| 4 | 4 | 231 us |
| 5 | 5 | 10,690 us |

All five runs reported `result=0`, ended at `state=4` (`STOPPED`),
`fault=0`, `pwm=0/0`, and invalid encoder transitions `0/0`. The observed
range was 204 us to 10,690 us (mean 4,410 us). The two approximately 10.7 ms
samples are consistent with the request arriving at a later point in the
Motion task's 20 ms schedule; the maximum remains below one control period.
This is raised-wheel USB evidence only. The team's maximum accepted stop
latency and loaded-car stopping distance still need agreement and testing.

