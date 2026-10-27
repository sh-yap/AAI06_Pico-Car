# Vehicle Controller

Owner: Buddy 1 (Toh Wen Qian).  Branch: `vehicle-controller`.

The Vehicle Controller (VC) decides what the car does next.  It reads every
subsystem's latest report each 20 ms, picks one state, and tells Motion
(Buddy 2), Obstacle (Buddy 5) and comms (Buddy 1) what to do.  No other
subsystem drives the motors or changes the mission state.

## Status (26 Oct 2026)

| Part | Status |
| --- | --- |
| State machine (`vc.c`) | Done. 150 host-test checks, 0 failed |
| RTOS task (`vc_task.c`) + `APP=vc_car` | Done. Builds (SMP=0 with WiFi, SMP=1). **Not yet tested on the car** |
| Motion (Buddy 2) | Connected: drive, stop, status.  Turns and reverse wait on Motion fixes (see Known issues) |
| Comms (Buddy 1) | Connected: laptop `start`/`stop`, `robot/state`, fault word, run summary, motion telemetry |
| Line + barcode (Buddy 3) | Code ready behind `VC_HAVE_BUDDY3`; waits on Buddy 3 merging into `main`.  Until then: placeholder line "centred" |
| Obstacle (Buddy 5) | Placeholder (no obstacle).  "Obstacle ahead" task is on branch `buddy-5-obstacle-task`; scan/bypass not written yet |
| Hump (Buddy 4) | Placeholder (never on a hump).  Needs a task on the car |

## Files

| File | Purpose |
| --- | --- |
| `app_program/vc.h` | States, input/output structs, tunable values (all marked TBC) |
| `app_program/vc.c` | The state machine: pure logic, no RTOS, no hardware |
| `app_program/vc_task.c/.h` | The 20 ms task: gathers inputs, calls `vc_step()`, acts on outputs |
| `app_program/vc_car_main.c` | `APP=vc_car` entry: starts comms, Motion and the VC task |
| `tests/vc/` | Host unit tests for `vc.c` (no board needed) |

## States

| State | What the car does | Leaves when |
| --- | --- | --- |
| INITIALISE | Nothing | Init OK → READY; init failed or 10 s → EMERGENCY_STOP |
| READY | Stopped | Laptop `start` → FOLLOW_LINE |
| FOLLOW_LINE | Drives, steering from the line error; half speed on a hump; remembers barcodes | Finish → FINISHED; obstacle → OBSTACLE_SCAN; junction → HANDLE_JUNCTION; line lost → REACQUIRE_LINE |
| HANDLE_JUNCTION | Passes through in the same step: uses the stored barcode | LEFT/RIGHT/U-TURN → TURNING; else → FOLLOW_LINE |
| TURNING | Encoder turn by Motion | Done → FOLLOW_LINE; failed or 3 s → EMERGENCY_STOP |
| OBSTACLE_SCAN | Stopped; asks Buddy 5 to scan | CONTINUE → FOLLOW_LINE; TURN_LEFT/RIGHT → BYPASS; REVERSE_RETRY → reverse, then rescan (max 2); STOP, unknown or 3 s → EMERGENCY_STOP |
| BYPASS | Goes around the obstacle | Obstacle passed → REACQUIRE_LINE; failed or 6 s → EMERGENCY_STOP |
| REACQUIRE_LINE | Creeps forward, straight | Line found → FOLLOW_LINE (on a junction: handled at once); obstacle → OBSTACLE_SCAN; 4 s → EMERGENCY_STOP |
| EMERGENCY_STOP | Brakes; latched | Faults clear **and** wheels still **and** laptop `stop` (acknowledgement) → READY |
| FINISHED | Stopped | Stays (restart the board) |

**Order of rules every step:**
1. A serious fault in any state → EMERGENCY_STOP.
2. Laptop `stop` in a running state → READY.
3. The state's own rules (in FOLLOW_LINE: finish, then obstacle, then
   junction, then line lost).
4. The state's timeout, only if step 3 changed nothing, so success at the
   deadline wins.

**Safety details:**
- A junction is handled once, however long the sensors stay over it.
- Bad input values fail safe: an unknown line code counts as LOST, unknown
  commands and barcodes are ignored, an unknown recommendation or a
  corrupted state leads to EMERGENCY_STOP.
- Buddy 5 gets ABORT only if it was actually asked for something.
- After REVERSE_RETRY, no new scan is requested until the reverse is done;
  scan results that arrive while reversing are ignored.

## How the task works

Every 20 ms (priority 5, just below Motion's 4; processor 2 on SMP builds):

1. **Inputs:** one laptop command; Motion status (done / failed /
   stationary, fault); line and barcode (Buddy 3, or the placeholder).
2. **Decide:** `vc_step()`.
3. **Act:** Motion commands; comms state, fault word, run begin/end; a
   console line `[vc] FROM -> TO` on every change.

The period is kept from absolute time, so work time does not drift it.

**Mapping to Motion:**

| VC output | Motion call |
| --- | --- |
| DRIVE (speed, steer) | `SET_VELOCITY`, angular rate = −steer × 400 mdeg/s.  Re-sent only when speed changes or steering moves by 8/127 or more |
| TURN LEFT / RIGHT / U-TURN | `motion_stop()`, then 40 ms later `TURN_ANGLE` −90° / +90° / 180° at 100 mm/s |
| REVERSE | `motion_stop()`, then 40 ms later `MOVE_DISTANCE` −100 mm at −80 mm/s |
| STOP | `motion_stop()` |

**Fault bits** (published in `robot/state` `fault`; proposed until the team
agrees the fault mask):

| Bit | Meaning |
| --- | --- |
| 0 | Motion reports FAULT, or its status can't be read |
| 1 | Motion rejected a drive command |

**VC state → `robot/state` name** (until `comms.h` uses the VC states):
INITIALISE → INIT, FOLLOW_LINE → LINE_FOLLOW, HANDLE_JUNCTION and
TURNING → EXECUTE_NAV, REACQUIRE_LINE → LINE_RECOVERY,
EMERGENCY_STOP → SAFE_STOP; the others keep their names.

## Build, test, flash

Host tests (no board needed), from `tests/vc`:

```sh
make test
```

Car firmware, from `build_make` in PowerShell (adapt the paths):

```powershell
$env:PATH = 'C:/Users/<you>/.pico-sdk/toolchain/15_2_Rel1/bin;C:/msys64/ucrt64/bin;C:/msys64/usr/bin;' + $env:PATH
make APP=vc_car BOARD=robo_pico_robot CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 PICO_SDK_PATH=C:/Users/<you>/.pico-sdk/sdk/2.3.1 SHELL=C:/msys64/usr/bin/sh.exe -j8
```

Flash `mtk3pico_smp0_usb_cdc_wifi_mqtt_vc_car_robo_pico_robot.uf2`.
WiFi needs `config/wifi_credentials.h` and `config/mqtt_config.h` (never
commit them).

**Car test 1 (placeholder line):**
1. Console shows `[init] ... -> OK`, `[vc] INITIALISE -> READY`,
   `[mqtt] CONNECTED`.
2. Car on the floor with about 1.5 m clear, or wheels lifted.
3. Publish `start` to `robot/command`: the car drives straight and stops by
   itself after 5 s (`[vc] test run time up: auto STOP`).
4. `start`, then `stop` within 5 s: it brakes at once.
5. `robot/state` shows READY → LINE_FOLLOW → READY.

Save logs in `evidence/buddy1/`.

## Known issues

**In Motion (Buddy 2), blocking turns and reverse:**
- `motion_encoder_reset()` is never called, so turns and moves measure
  from power-on and finish at once after any driving.
- A left turn (negative angle) never completes: its target is negative but
  the travelled distance is always positive, so it times out and faults.
- Every `SET_VELOCITY` resets the speed controller.  The VC works around
  this by re-sending only on real change; steering updates ideally
  shouldn't reset it.
- `motion_stop()` discards commands queued before Motion's next loop.  The
  VC waits 40 ms after a stop before sending the next command.

**In the VC:**
- Tunables are first guesses (TBC): speeds, timeouts, steering gain.  Tune
  on the track.
- The finish condition (`b_finish`) has no source yet.
- BYPASS has no Motion routine yet: the VC stops and its bypass timeout
  ends in EMERGENCY_STOP (fail safe).
- An obstacle seen during a turn is ignored (the car turns on the spot).

## Next steps

1. Car test 1 (above).
2. Buddy 3 merged → set `VC_HAVE_BUDDY3` to 1, start Buddy 3's task and
   message buffer in `vc_car_main.c`, tune the steering on the track.
3. Buddy 2 turn fixes → test junction turns and reverse.
4. Buddy 5 merged → connect "obstacle ahead" (`OBSTACLE_EVT_AHEAD` +
   `obstacle_task_ack_ahead()`) and the sensor fault; later scan and bypass.
5. Buddy 4 task → hump flag.
6. Merge `main` into `vehicle-controller`, then a pull request into `main`.

## BARR-C:2018 notes

The VC code follows the team's BARR-C rules: braces on their own lines,
fixed-width types, `p_`/`b_` prefixes, constants on the left of `==`, one
exit per function, `else` closing every `if`/`else if` chain, `default` in
every switch, no `#include` in headers, `/*** end of file ***/` marker.

Deviations:
1. `usermain()` returns `INT` (signature fixed by micro T-Kernel), and
   `vc_car_main.c` has no header (it exports only `usermain()`).
2. `vc.c` includes `<stddef.h>` for `NULL`; it includes no kernel headers,
   so the kernel's `size_t` clash does not apply.
3. The task loop is `for (;;)`, as every RTOS task body is.
