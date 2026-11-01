# Testing the `test_integrate` branch

How to build, flash and test the integrated car on your own laptop, what the
current version does, and what changed in each buddy's code.

Last updated: 28 Oct 2026 (Buddy 1 / VC). Questions: Wen Qian.

---

## 1. What the car does now

`APP=vc_car` runs everything together on the Pico W (dual core):

| Part | Owner | Status |
|---|---|---|
| Vehicle Controller (VC): state machine, safe stops | Buddy 1 | Working. Follows the line, searches for a lost line, turns at junctions from barcodes, stops safely on any fault. |
| WiFi + MQTT telemetry and commands (binary format) | Buddy 1 | Working. Heartbeat every 1 s, line/motion telemetry, events, `start`/`stop` from the laptop. |
| Motion: wheel speed, turns, encoders | Buddy 2 | Merged (Suen's 28 Oct version). See known issues: encoder noise. |
| Line following + barcodes | Buddy 3 | Working with 2 line sensors (centre, right). Barcodes often read `INVALID`. |
| Hump / IMU | Buddy 4 | Not connected yet (VC assumes "no hump"). |
| Obstacle ahead (ultrasonic) | Buddy 5 | Connected. False alarms seen on the track (`dist_mm: 0`). Bypass not written yet: the car stops safely instead. |

**Run sequence.** Laptop sends `start` → `LINE_FOLLOW` → at a junction,
turn if a barcode was read before it (A = left, F = right, Z = U-turn,
**placeholder meanings** until the lecturer gives the real ones) → `stop`
from the laptop, or `SAFE_STOP` on any fault, obstacle or a line lost for
more than 4 s.

---

## 2. What changed, per buddy

These changes are on `test_integrate` only. Owners, please review the
changes in your files (each is commented "Buddy N to review").

### Buddy 2 (Motion), Suen Hong
- Your 27 and 28 Oct work is merged **as-is**.
- `motion_task.c`, `motion_submit()`: waits up to **30 ms** for
  `command_mtx` (was `TMO_POL`). The Motion loop holds the lock for its
  whole body and both tasks run every 20 ms, so the VC got `BUSY` every
  step: the car sat still with no fault.
- `motion_encoder.c`, encoder ISR: reads both pairs twice, **3 µs
  apart**, and ignores a level that changes in between (noise filter).
  It helps a little, but count-jump faults still happen. See section 6.
- `motion_config.c`: the start-up kick is your value (200). On the floor
  it sometimes does not start the wheels, so please re-check it with
  proper power.

### Buddy 3 (Line + barcode)
- `barcode.c/.h`: added **F** and **Z**. Barcode codes now match the MQTT
  barcode field (`NONE, A, B, C, D, F, Z, INVALID` = 0..7). Testing
  mapping: **A = LEFT, F = RIGHT, Z = U-TURN** (B/C/D kept for bench
  tests). Change the A/F/Z lines in the `COURSE MAPPING` switch once the
  lecturer gives the meanings.
- Your task is now **pinned to processor 1** (in `vc_car_main.c`): the ADC
  interrupt belongs to processor 1. Please pin it in your own `app_main.c`
  too.

### Buddy 5 (Obstacle)
- No changes to your code.
- New build switch **`VC_OBSTACLE=0`**: builds the car without your
  obstacle input, for line-following tests while the false alarms are
  being fixed.

### Buddy 4 (IMU)
- No changes. Not connected yet.

### Buddy 1 (WiFi + VC)
- **VC tuning:** 160 mm/s on the line, slowing to 90 mm/s on bends;
  stronger steering (max ~89°/s); lost-line search turns near the spot.
- **VC safety:** if Motion refuses a drive command for 0.2 s, the car stops
  with fault 4 instead of waiting silently.
- **WiFi:** chip power saving off (faster and steadier on a phone hotspot).
- **Crash fix (27 Oct):** a diagnostic print overflowed the start-up task's
  stack ~5 s after boot and crashed the car.

### Shared build system
- `build_make/pico_rp2040.mk`: the "discard old objects when the build
  type changes" step **never ran on Windows** (it called Windows'
  `FIND.EXE`). Every switch between builds silently reused old objects.
  Fixed. If you built before this fix, delete `build_make/.build_profile`
  once.

---

## 3. One-time setup on your laptop

1. **Pico SDK 2.3.1** and the ARM toolchain (VS Code Pico extension), plus
   **MSYS2** (`C:\msys64`) for `make` and `sh`.
2. **Get the branch:**
   ```
   git fetch origin
   git checkout test_integrate
   git pull
   ```
3. **Two config files that are not in git.** Copy the templates and fill
   them in. Get the hotspot name/password and the broker IP from Wen Qian
   **privately**, and never commit these files.
   ```
   copy config\wifi_credentials.example.h config\wifi_credentials.h
   copy config\mqtt_config.example.h      config\mqtt_config.h
   ```
   - `wifi_credentials.h`: SSID, password, and set `UTK_WIFI_CREDENTIALS_SET` to `1`.
   - `mqtt_config.h`: `UTK_MQTT_BROKER_ADDRESS` = the IPv4 of the laptop
     running Mosquitto on the hotspot (`ipconfig`, Wi-Fi adapter), and set
     `UTK_MQTT_CONFIG_SET` to `1`.
4. **Only on the laptop that runs the broker:** install **Mosquitto**, and in
   `C:\Program Files\mosquitto\mosquitto.conf` add:
   ```
   listener 1883 0.0.0.0
   allow_anonymous true
   ```
   Restart the service (`net stop mosquitto`, then `net start mosquitto`,
   as admin) and allow **TCP 1883** in Windows Firewall.
5. **Phone hotspot:** 2.4 GHz ("Maximise Compatibility" on iPhone),
   **phone charged, Low Power Mode off**, screen on, near the track. A low
   phone battery hides the hotspot and drops the car.

---

## 4. Build

From `build_make\` in PowerShell (replace `<you>`):

```
$sdk = "PICO_SDK_PATH=C:/Users/<you>/.pico-sdk/sdk/2.3.1"
$sh  = "SHELL=C:/msys64/usr/bin/sh.exe"
$wifi = "WIFI=cyw43","WIFI_JOIN=1","WIFI_NETIF=1","WIFI_DHCP=1","WIFI_DNS=1","WIFI_MQTT=1"

# A. Full car (all buddies)
make APP=vc_car BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc @wifi $sdk $sh -j8
#  -> mtk3pico_smp1_usb_cdc_wifi_mqtt_vc_car_robo_pico_robot.uf2

# B. Line following only (obstacle input off; the car will NOT stop for obstacles)
make APP=vc_car BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc @wifi VC_OBSTACLE=0 $sdk $sh -j8
#  -> mtk3pico_smp1_usb_cdc_wifi_mqtt_noobst_vc_car_robo_pico_robot.uf2

# C. Motion only (Buddy 2's button test; no WiFi needed)
make APP=motion_closed_loop_button_test BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc $sdk $sh -j8
#  -> mtk3pico_smp1_usb_cdc_motion_closed_loop_button_test_robo_pico_robot.uf2
```

A successful build ends with `elf2uf2 ...` and no `error`.

**Flash:** hold **BOOTSEL**, plug in USB, and copy the `.uf2` onto the
`RPI-RP2` drive. The board restarts by itself.

---

## 5. Test

Test in this order: first that the wheels work, then the line, then everything.

### Test C: Motion only (no laptop needed)
1. Flash image **C**. Power the car the way it will race (see section 6).
2. Put it on the floor with about 50 cm clear in front.
3. Press **GP20** (go: 300 mm forward at 100 mm/s); **GP21** stops it.
4. Count the beep groups:

| 1st group | Meaning |
|---|---|
| 1 | Success: reached 300 mm |
| 2 | You pressed stop |
| 3 | Timeout |
| 4 | Stall (a wheel didn't turn: power or mechanical) |
| 5 | Encoder fault: more groups follow ↓ |

For an encoder fault, the next groups are: **check** (1 = count jump,
2 = speed calculation, 3 = invalid transitions) · **wheel** (1 = left,
2 = right, 3 = both) · **time** (1 ≤ 5 ms, 2 ≤ 15, 3 ≤ 30, 4 more) ·
**size** (1 ≤ 100 counts, 2 ≤ 250, 3 ≤ 500, 4 ≤ 1000, 5 more). The real
maximum is about 60 counts per 15 ms, so a big jump means electrical noise.

### Test B and A: on the track with the laptop
1. Flash image **B** (line only) or **A** (full car).
2. Window 1, the logger (the broker laptop records into `evidence\buddy1\`):
   ```
   cd <repo>
   powershell -ExecutionPolicy Bypass -File tools\mqtt_log.ps1 -Name mytest1
   ```
   The first 4 lines are old retained messages. Wait for **new** lines:
   `robot/status online` and `robot/info <build date/time>` (check that
   it is your build).
3. Put the **centre line sensor on the black tape**. The logger must show
   `"pos":"CENTRE"`, not `LOST`, before you start.
4. Window 2:
   ```
   & "C:\Program Files\mosquitto\mosquitto_pub.exe" -h localhost -t robot/command -m start
   & "C:\Program Files\mosquitto\mosquitto_pub.exe" -h localhost -t robot/command -m stop
   ```
   (Use the broker laptop's IP instead of `localhost` from another laptop.)
   After a `SAFE_STOP`, send `stop` and then `start` again.
5. After the run, **don't power off**. Wait about 30 s so the car can send its
   final state, then press Ctrl+C in the logger and share the `.log` file.

**Serial console** (VS Code Serial Monitor, COM port, 115200) shows
`[init] ... -> OK`, `[vc] STATE -> STATE`, `[diag] ...` every 5 s,
`[mqtt] CONNECTED` and any `Hard fault`.

### Reading the log

| You see | Meaning |
|---|---|
| `state: LINE_FOLLOW` | Following the line |
| `state: LINE_RECOVERY` | Line lost; searching (up to 4 s) |
| `state: EXECUTE_NAV` | Turning at a junction |
| `state: OBSTACLE_SCAN` then `SAFE_STOP` | Obstacle ahead (bypass not written yet) |
| `state: SAFE_STOP, fault: 0` | Line lost for more than 4 s, or a scan timeout |
| `fault: 1` | Motion: stall |
| `fault: 2` | Motion: encoder fault |
| `fault: 4` | Motion: refused or busy command |
| `fault: 32` | Ultrasonic fault |
| `robot/status offline` | WiFi/MQTT lost (the car keeps driving on its own) |

---

## 6. Known issues (28 Oct)

1. **Encoder noise (Buddy 2 hardware/power).** The Motion-only test gives
   count-jump faults (for example `5 1 3 2 4`) on **both** laptop USB and
   power bank, with both wheels together. That is motor electrical noise
   on the encoder wires.
   - Fit a **0.1 µF ceramic capacitor ("104") across each motor's terminals**.
   - Add **4.7–10 kΩ pull-ups** on GP4, GP5, GP16, GP17.
   - Keep the encoder wires away from the motor wires.
2. **Power.** The slide says **VIN, 3.6–6 V, 1 A per motor**: use **VIN
   from a 2 A+ supply**, not the Pico's micro-USB. A weak supply stalls
   the motors and drops WiFi when they start.
3. **False obstacles (Buddy 5)** at `dist_mm: 0` / 1.26 m. Use image B
   meanwhile.
4. **Barcodes read `INVALID` (Buddy 3).** Only 2 line sensors, so left
   bends are guessed.
5. **Not done yet:** obstacle bypass (Buddy 5 + VC), hump input (Buddy 4),
   finish detection, real A/F/Z meanings.
6. **Windows Smart App Control** may block the host unit-test `.exe` files
   in `tests/`. That affects tests only, not the firmware.
7. `docs/COMMS_GUIDE.md` still says the liveness LED is GP16. It is now
   **GP19** (`sysdef.h`), so there is no conflict with the left encoder.

**Reporting a problem:** say which image (A/B/C) and its `robot/info`
build time, the power source (VIN / USB / power bank), and attach the
`.log` (or the beep groups for test C).
