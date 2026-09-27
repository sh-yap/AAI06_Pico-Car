# Buddy 5 — ultrasonic and servo

Buddy 5 owns the HC-SR04 ultrasonic sensor and the SG90 scan servo. It
reports distances and, later, obstacle profiles and bypass recommendations to
the Vehicle Controller. It never drives the motors; Buddy 2 does.

Current stage: **Stage 3 / 7A–7B bring-up**. Drivers and two bench test
programs build; they have not yet been run on hardware.

## Files

| File | Purpose |
| --- | --- |
| `app_program/obstacle_config.h` | All pins and calibration constants (edit here) |
| `app_program/obstacle_types.h` | Shared types: range reading, servo calibration |
| `app_program/obstacle_calc.c/.h` | Pure maths, unit-tested on the PC |
| `app_program/obstacle_ultrasonic.c/.h` | HC-SR04 driver |
| `app_program/obstacle_servo.c/.h` | SG90 driver |
| `app_program/obstacle_console.c/.h` | Console helpers for the test programs |
| `app_program/ultrasonic_test_main.c` | Bench test, `APP=ultrasonic_test` |
| `app_program/servo_test_main.c` | Bench test, `APP=servo_test` |
| `tests/obstacle/` | Host unit tests for `obstacle_calc.c` |

## Wiring (Robo-Pico)

| Signal | Connection |
| --- | --- |
| Servo | Servo port **S1 = GP12** (brown GND, red V+, orange signal) |
| HC-SR04 Trig | **GP16** |
| HC-SR04 Echo | **GP17 through a divider**: Echo → 1 kΩ → GP17, GP17 → 2 kΩ → GND |
| HC-SR04 VCC / GND | V+ and GND of a spare servo port (about 4.6 V on USB power) |

- Do not use Grove 1 (GP0/GP1): startup code gives those pins to the UART
  console in every build.
- Grove ports only supply 3.3 V; the HC-SR04 needs 5 V. Powered at 5 V its
  Echo output is 5 V, so the divider is required before GP17.
- Use the Grove port wired to GP16/GP17 for the two signal wires, or jumper
  them. Check the board labels, not the wire colours.
- To move a pin, change it in `obstacle_config.h` only. The build rejects
  invalid choices: Echo must be an odd GPIO, and neither pin may share a PWM
  slice with the other or with the drive motors (GP8–GP11).

## How it works

- **Echo timing, no interrupt:** GP17 is PWM slice 0 channel B. The slice
  runs in level-gated mode at 1 µs per count, so it counts only while Echo is
  high. After each ping the counter holds the echo width in µs. This avoids
  Buddy 2's GPIO interrupt, and task switches or USB printing can't skew it.
- **Measurement:** wait for 60 ms since the last ping, send a 12 µs trigger,
  sleep 40 ms, read the counter. Readings are classified `OK`, `NO_ECHO`,
  `TOO_CLOSE` (<20 mm), `TOO_FAR` (>4 m), `TIMEOUT` (nothing reflected) or
  `NOT_READY`. A non-`OK` reading is never reported as a distance.
- **Servo:** 50 Hz PWM on slice 6, 1 µs resolution. Angles map linearly
  through the calibrated 0/90/180° pulse widths and are clamped to the safe
  range; raw pulse commands are clamped to 500–2500 µs.

## Build, test, flash

Host unit tests (no board needed):

```sh
make -C tests/obstacle test
```

Firmware, from `build_make` in PowerShell (adapt the paths):

```powershell
$env:PATH = 'C:/Users/<you>/.pico-sdk/toolchain/15_2_Rel1/bin;C:/msys64/ucrt64/bin;C:/msys64/usr/bin;' + $env:PATH
make APP=ultrasonic_test BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=C:/Users/<you>/.pico-sdk/sdk/2.3.1 SHELL=C:/msys64/usr/bin/sh.exe -j8
make APP=servo_test      BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=C:/Users/<you>/.pico-sdk/sdk/2.3.1 SHELL=C:/msys64/usr/bin/sh.exe -j8
```

`BOARD=robo_pico_robot` is required: it releases the PWM block from reset.
With `BOARD=pico` both drivers report `init FAILED`.

Flash: hold BOOTSEL, plug in USB, copy
`mtk3pico_smp1_usb_cdc_<app>_robo_pico_robot.uf2` to `RPI-RP2`, then open the
board's COM port in a serial monitor.

## Test procedure and evidence

Save results in `evidence/ultrasonic/`. Record the tester, date, commit,
supply (USB or battery) and setup with each file.

### 1. `ultrasonic_test` (Stage 7A)

Keys: `b` batch of 10, `p` pause or resume, `h` help. Output is CSV:
`seq,time_ms,echo_us,distance_mm,status`.

1. **Smoke test:** move your hand in front; the distance should follow it.
2. **Fault handling:** point it at open space (expect `TIMEOUT`/`TOO_FAR`),
   hold something within 2 cm (`TOO_CLOSE`), unplug Echo (`NO_ECHO`).
3. **Calibration:** use a flat board square to the sensor at 100, 200, 300,
   500, 800, 1000 and 1500 mm, measured from the front of the transducers.
   Press `b` at each distance. Record in `ultrasonic_calibration.csv`:
   `true_mm,median_mm,min_mm,max_mm,valid,error_mm,error_pct`.
4. If the error grows with distance, the speed of sound is off. Set
   `OBSTACLE_SOUND_SPEED_M_S = 343 * true_mm / median_mm` (use a long
   distance), rebuild, and repeat step 3.

### 2. `servo_test` (Stage 7B)

Keys: `a`/`d` ±5°, `A`/`D` ±1°, `c` centre, `1`–`5` = 30/60/90/120/150°,
`[`/`]` ±10 µs raw, `{`/`}` ±50 µs raw, `s` coarse scan, `t` settle test.

1. **Centre:** at start-up the servo goes to 90°. If the sensor isn't facing
   straight ahead, refit the horn one spline tooth at a time, then fine-tune
   with `[`/`]` until it's exactly forward. That pulse is `PULSE_90_US`.
2. **Range:** with a protractor, step the raw pulse to find 0° and 180°.
   **Stop as soon as the servo buzzes or strains:** that's the end stop.
   Record `PULSE_0_US` and `PULSE_180_US`. If 0° or 180° can't be reached
   safely, set `ANGLE_MIN_DEG`/`ANGLE_MAX_DEG` to the angles you did reach.
3. **Settling:** point the sensor so 150° faces a wall and press `t`. The
   `settle,t_ms,...` rows change while the horn moves and wobbles. The time
   when they stop changing is the real settle time; compare it with the
   printed prediction and tune `MS_PER_DEG` / `SETTLE_MARGIN_MS`.
4. **Scan preview:** put a box in front, press `s`, and check which angles
   see it.
5. Put the calibrated values into `obstacle_config.h`, rebuild, re-run
   steps 1–4, and save the logs as `servo_calibration.md`.

## BARR-C:2018 notes

The Buddy 5 code follows BARR-C:2018:

- braces on their own lines, even for single statements; 4-space indents,
  no tabs; lines of at most 80 characters;
- fixed-width `stdint.h` types, `bool` for booleans;
- `static` for everything private; public names prefixed with the module
  name (`obstacle_servo_...`);
- `p_` prefix for pointers, `b_` for booleans;
- constants on the left of `==`; one exit per function; every
  `if`/`else if` chain ends in `else`; every `switch` has a `default`;
- a comment on every cast; file and function header comments; the
  `/*** end of file ***/` marker.

Deliberate deviations, each commented in the code:

1. `usermain()` returns `INT`: the signature is fixed by micro T-Kernel.
2. `obstacle_console.c` declares `tm_usb_console_getchar()` itself: the
   lecturer's USB console module has no header for it.
3. The two `*_test_main.c` files have no header: they export nothing but
   `usermain()`, which the kernel declares.
4. Headers contain no `#include`, so each `.c` file includes
   `<stdint.h>`, `<stdbool.h>` and `obstacle_types.h` first (see each
   header's note). Files that include kernel headers must not include
   `<stddef.h>`: the kernel's `syslib.h` redefines `size_t`.

Buddy 2's motion files were written before the team adopted BARR-C and don't
follow it yet.

## Next steps

1. Run the two tests above on hardware and commit the calibration values.
2. Stage 7C–7E: coarse and fine scan, obstacle profile (position, width,
   closest point, left/right clearance).
3. Stage 7F–7G: bypass recommendation and line-recovery plan, handed to the
   Vehicle Controller (not to Buddy 2 directly).
