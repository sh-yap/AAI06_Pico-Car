# Communication Subsystem Guide (Buddy 1)

What the WiFi / MQTT / telemetry code is, where it lives, how to read it,
and how the other subsystems use it.

- **Part A** - for Buddy 1: every file, what it does, what to read first.
- **Part B** - for the whole team: how to call the API from your task.
- **Part C** - build, flash and test on the car.

---

## Big picture in one paragraph

Other subsystems never touch WiFi.  They call small functions in
`comms.h` (for example `comms_report_motion(&m)`), which copy the data and
return immediately.  One task - the **Radio Service Task** - owns the WiFi
chip and the network stack.  Every 2 ms it keeps WiFi connected, keeps the
MQTT session to the broker alive, takes whatever the other tasks left in
`comms`, turns it into JSON and publishes it.  Commands from the laptop
(`start`, `stop`, `status`) come back the same way and wait in a queue
until the vehicle controller picks them up.

Diagrams: `docs/img/comms_interaction.png` (who talks to whom) and
`docs/img/comms_statechart.png` (connect / reconnect behaviour).

---

## Part A - Where the Buddy 1 code is

### Files that are new (written for this project)

| File | Lines | What it does | Read? |
|---|---|---|---|
| `app_program/comms.h` | 176 | **The public API.**  The enums (states, nav commands, line position...), the telemetry structs with units, and every function teammates call. | **1st** |
| `app_program/comms.c` | 969 | Implements the API.  Top half (to line ~400): the *caller side* - copies data into snapshot slots or the event queue and returns.  Bottom half: the *radio side* - `comms_link_next()` decides what to publish next and builds the JSON. | **2nd** |
| `app_program/comms_link.h` | 42 | The narrow boundary between `comms.c` and the MQTT session: 3 functions, plain C types only. | 3rd |
| `lib/libnet/lwip/lwip_utk_mqtt.c` | 410 | **The MQTT session state machine** (IDLE → CONNECTING → CONNECTED → BACKOFF).  Connects to the broker, sets the last-will, subscribes to `robot/command`, publishes what `comms_link_next()` gives it, retries with back-off. | **4th** |
| `lib/libnet/lwip/include/lwip_utk_mqtt.h` | 53 | Its status struct (state, counters, last command). | with 4th |
| `app_program/comms_demo.c/.h` | 170 | **Test only** (`COMMS_DEMO=1`): fake motion/line/terrain data and events, driven by `start`/`stop`.  Shows exactly how teammates will call the API. | useful example |
| `app_program/timing_probe.c/.h` | 141 | **Test only**: measures how late a 10 ms task starts while WiFi is busy (`[probe]` lines). | optional |
| `app_program/build_info.c/.h` | 15 | Build date/time, printed at boot and published on `robot/info`, so you always know which firmware is running. | skim |
| `config/mqtt_config.example.h` | 30 | Template for the broker address. The real `config/mqtt_config.h` is **not** in git. | skim |
| `tools/mqtt_log.ps1` | 51 | Records all MQTT traffic to `evidence/buddy1/<time>_<name>.log`. | how to use |
| `tools/mqtt_check.ps1` | 154 | Analyses a recording: rates, heartbeat gaps, dropped events, command times. | how to use |
| `tools/soak.ps1` | 34 | Long unattended test: record + start + wait + stop + analyse. | how to use |
| `docs/TEST_PLAN_COMMS.md` | - | Tests T1-T8, results, and defects D1-D4 found and fixed. | for the report |
| `docs/DESIGN_REVIEW_BUDDY1.md` | - | Buddy 1 content for the Week 6 design review. | for the review |

### Files that existed and were changed

| File | What was changed | Where to look |
|---|---|---|
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | The existing radio task.  Added: automatic **WiFi rejoin** with growing waits (`service_rejoin()`), calling the MQTT session every poll, copying MQTT status out, reading RSSI every 5 s, DHCP state for diagnostics. | `service_rejoin()` line ~504; MQTT call line ~702; constants lines 51-59 |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h` | Added fields to the status struct: rejoin counters, MQTT state and last command, current IP, DHCP state. | search `mqtt_`, `rejoin_` |
| `lib/libnet/lwip/include/lwipopts.h` | lwIP settings: TCP on for MQTT, bigger MQTT output buffer, **DHCP conflict check off** (bug D1), faster give-up on an unreachable broker. | lines 25, 38, 57-89 |
| `build_make/pico_rp2040.mk` | New build switches `WIFI_MQTT` and `COMMS_DEMO`; include path for SDK 2.3.1 (`pico_util`); build stamp always rebuilt. | lines 148, 163, 278, 454, 462 |
| `app_program/app_main.c` | Calls `comms_init()`; starts demo + probe in demo builds; prints `[wifi]`, `[net]`, `[mqtt]` status lines; prints the build stamp. | lines 206, 284-400, 622-625 |
| `config/config.h` | Allowed message buffers raised 4 → 8 (comms uses 2). | `CNF_MAX_MBFID` |
| `.gitignore` | `config/mqtt_config.h` kept out of git. | - |

### Suggested reading order (about 2 hours)

1. **`comms.h`** top to bottom.  Understand the four kinds of call:
   `comms_report_*` (latest value), `comms_event_*` (queued, never lost
   silently), `comms_set_state/fault` (controller), `comms_get_command`.
2. **`comms.c` caller side** (lines ~200-400).  Key idea: nothing waits.
   - `comms_report_motion()` (line ~311): the *seqlock* - bump a counter
     to odd, copy, bump to even.  The reader copies and checks the counter
     did not change; if it did, it simply tries again next time.
   - `queue_event()` (line ~177): `tk_snd_mbf(..., TMO_POL)` - "send if
     there is room, otherwise give up immediately and count it".
3. **`comms.c` radio side**: `comms_link_next()` (line ~813).  It is a
   priority list - command ack, firmware info, state change, queued event,
   heartbeat, then telemetry.  `next_snapshot()` (line ~730) shows the
   5 Hz rate limit and the "skip if unchanged" hash.
4. **`lwip_utk_mqtt.c`**: `lwip_utk_mqtt_poll()` (line ~321) is the state
   machine - compare it with the statechart diagram.  Then
   `service_connected()` (line ~249) and `publish_pending()` (line ~221):
   if lwIP is full, the message is kept and retried, not lost.
5. **`cyw43_utk.c`**: only `service_rejoin()` (line ~504) and the MQTT
   lines in the main loop - the rest is the original radio port.

Questions worth being able to answer at Tribunal 1:

- *Why can't another task just call `mqtt_publish()` directly?*  lwIP is
  configured with one execution context (`NO_SYS=1`); only the radio task
  may call it.  That is why `comms` exists.
- *What happens to a barcode event if WiFi is down?*  It waits in the
  16-slot queue and is sent after reconnecting, with its original time
  `t`.  If the queue is full it is dropped and counted in `ev_drop`.
- *Why can't the laptop drive the car?*  The brief requires autonomy;
  commands only start/stop the mission.
- *How do you know the network doesn't disturb control?*  Test T7:
  max 2.0 ms delay to a 10 ms task during a busy run.

---

## Part B - How to use it from your subsystem (all buddies)

```c
#include "comms.h"
```

Rules:

1. `comms_init()` is already called once in `usermain()` - don't call it again.
2. Every call returns immediately.  It is safe in a control loop.
3. Each `comms_report_*()` must be called from **one task only** (its owner).
4. Units: **mm, mm/s, degrees, ms, integers only** (the RP2040 has no FPU).
5. Report as often as you like - the link sends at most 5 per second per
   topic and skips values that have not changed.

### Buddy 2 - Motion (Suen Hong)

```c
comms_motion_t m = {0};

m.speed_left_mm_s  = left_speed;
m.speed_right_mm_s = right_speed;
m.encoder_left     = left_count;     /* raw counts since boot */
m.encoder_right    = right_count;
m.distance_mm      = distance_travelled;
comms_report_motion(&m);             /* e.g. every PID cycle */
```

### Buddy 3 - Line and barcode (Wynn)

```c
comms_line_t l = {0};

l.position = COMMS_LINE_CENTRE;      /* LOST/LEFT/CENTRE/RIGHT/JUNCTION */
l.error    = line_error;             /* your own scale, signed */
l.sensors  = raw_bits;               /* bit0 left, bit1 centre, bit2 right; 1 = black */
comms_report_line(&l);

(void) comms_event_barcode('A', COMMS_NAV_LEFT);   /* once per barcode */
```

### Buddy 4 - IMU and terrain (Zhi Hong)

```c
comms_terrain_t t = {0};

t.motion         = COMMS_MOTION_CLIMBING_HUMP;
t.pitch_deg      = pitch;
t.roll_deg       = roll;
t.hump_height_mm = current_height;   /* 0 on flat ground */
comms_report_terrain(&t);

(void) comms_event_hump(peak_mm);    /* once per hump, at its peak */
(void) comms_event_motion(COMMS_MOTION_IMPACT);
```

The run summary's `max_hump_mm` is worked out from the hump events.

### Buddy 5 - Obstacle (Nicholas)

```c
comms_obstacle_t o = {0};

o.distance_mm    = closest_mm;
o.angle_deg      = closest_angle;    /* 90 = straight ahead */
o.width_mm       = width;
o.clear_left_mm  = left_gap;
o.clear_right_mm = right_gap;
o.action         = COMMS_AVOID_TURN_LEFT;
(void) comms_event_obstacle(&o);     /* once per completed profile */
```

### Vehicle controller (owner to be decided)

```c
comms_cmd_t cmd;

if (comms_get_command(&cmd))          /* poll in the controller loop */
{
    if (COMMS_CMD_START == cmd) { (void) comms_begin_run(); /* ...start mission */ }
    if (COMMS_CMD_STOP  == cmd) { /* ...stop safely */ (void) comms_end_run(); }
}
comms_set_state(COMMS_STATE_LINE_FOLLOW);   /* on every state change */
comms_set_fault(0U);                        /* bit flags, 0 = no fault */
```

### MQTT topics (what appears on the laptop)

| Topic | Content | When |
|---|---|---|
| `robot/status` | `online` / `offline` | connect / lost (retained) |
| `robot/info` | firmware build time | each connect (retained) |
| `robot/state` | state + fault | on change (retained) |
| `robot/heartbeat` | seq, state, fault, ev_drop | every 1 s |
| `robot/telemetry/motion` | speeds, encoders, distance | ≤ 5 Hz, on change |
| `robot/telemetry/line` | position, error, sensor bits | ≤ 5 Hz, on change |
| `robot/telemetry/terrain` | motion class, pitch, roll, hump, max hump | ≤ 5 Hz, on change |
| `robot/event/barcode` · `hump` · `obstacle` · `motion` · `run` | one message per event | as it happens (QoS 1) |
| `robot/event/command` | acknowledgement `{"cmd":"start","ok":1}` | per command |
| `robot/summary` | run time, distance, max hump, counts | end of run (retained) |
| `robot/command` | **send** `start` / `stop` / `status` here | - |

Every payload starts with `"v":1` (format version) and `"t"` (robot
uptime in ms).  The full reference will be in `docs/MQTT.md`.

---

## Part C - Build, flash and test on the car

### Each laptop that builds WiFi firmware needs

- Pico SDK 2.3.1 (path passed as `PICO_SDK_PATH`).
- `config/wifi_credentials.h` - copy from `.example.h`, fill in, set the flag to 1.
- `config/mqtt_config.h` - copy from `.example.h`, set the broker laptop's IP, set the flag to 1.
- These two files are **not in git**: share them privately, never commit them.

### Build commands (from `build_make/`)

```sh
# Real firmware (teammates' code + comms, no fake data)
make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 PICO_SDK_PATH=C:/Users/<you>/.pico-sdk/sdk/2.3.1 -j8
#  -> mtk3pico_smp0_usb_cdc_wifi_mqtt.uf2

# Same plus fake data and timing probe (testing / demo of telemetry only)
make ... WIFI_MQTT=1 COMMS_DEMO=1 ...
#  -> mtk3pico_smp0_usb_cdc_wifi_mqtt_demo.uf2

# No WiFi at all (for a buddy testing only their own hardware)
make CONSOLE=usb_cdc PICO_SDK_PATH=... -j8
#  comms calls still compile and work; nothing is sent.
```

### Checklist for testing on the car

1. **Is the car's board a Pico W?**  Without the W there is no WiFi.
2. **Broker laptop on the hotspot** - run `ipconfig`; if its IPv4 is not
   `172.20.10.13`, edit `config/mqtt_config.h` and rebuild.
3. **Mosquitto running** on that laptop (Windows service; firewall rule
   for port 1883 added - see earlier setup).
4. **Flash, then check the build stamp** on the serial console or
   `robot/info` - confirms the right firmware.
5. **Watch everything:** `tools\mqtt_log.ps1 -Name car_test1` (records
   into `evidence\buddy1\`), or MQTT Explorer for a live view.
6. **Pin check against the Robo Pico:** GP23, GP24, GP25, GP29 are the
   WiFi chip's; GP16 is the liveness LED.  Any teammate wiring must avoid
   them.  WiFi also takes one PIO state machine and two DMA channels.
7. **If it does not connect**, read the `[net] waiting:` line on the
   serial console: `link=0` → WiFi problem; `ip=0.0.0.0` → no address;
   `mqtt_state=3` with `err` → broker unreachable (IP / firewall).
