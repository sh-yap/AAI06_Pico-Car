# Communication Subsystem Test Plan (Buddy 1)

Covers the WiFi / MQTT / telemetry subsystem: connection, recovery,
commands, telemetry content, resource use and interference with control
timing.  Tests run on the real Pico W with the fake-data build, so they do
not depend on the other subsystems.

## Setup

| Item | Value |
|---|---|
| Firmware | `build_make/mtk3pico_smp0_usb_cdc_wifi_mqtt_demo.uf2` |
| Build command | `make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 COMMS_DEMO=1 PICO_SDK_PATH=... -j8` |
| Network | iPhone hotspot, 2.4 GHz ("Maximise Compatibility" on) |
| Broker | Mosquitto 2.1.2 on the laptop (Windows service), port 1883 |
| Serial console | USB CDC, VS Code Serial Monitor |
| MQTT recorder | `tools\mqtt_log.ps1 -Name <test id>` → `evidence\buddy1\<time>_<test id>.log` |
| Log analyser | `tools\mqtt_check.ps1 <log>` → `<log>.report.txt` |

Before every test, confirm the firmware: the retained `robot/info` message
and the `build ...` line on the serial console must show the same build
time as the `.uf2` that was flashed.

Serial-console evidence (lines starting `[wifi]`, `[mqtt]`, `[probe]`,
`[monitor]`) is copied into `evidence\buddy1\<test id>_serial.txt`.

## Tests

### T1 - Boot and connect
1. Hotspot on, laptop on the hotspot, recorder running.
2. Power-cycle the Pico.

**Pass:** `robot/info` shows the expected build; `robot/status online`
and `robot/state READY` are published; `[mqtt] CONNECTED` within 30 s of
power-up.

### T2 - Full telemetry run
1. Recorder running.  Send `start`; let it run 60 s; send `stop`.

**Pass:** command acknowledged (`ok:1`) for both; state goes
`READY → LINE_FOLLOW → (EXECUTE_NAV ↔ LINE_FOLLOW) → FINISHED`; every topic
type appears (motion, line, terrain, barcode, hump, obstacle, run, summary);
summary counts match the number of events in the log and `max_hump_mm`
equals the largest `peak_mm`; heartbeat has no gaps; `ev_drop` stays 0.

### T3 - Command handling
Send, in order: `hello`, `status`, `start`, `start`, `stop`, `stop`.

**Pass:** `hello` → `{"cmd":"unknown","ok":0}`; `status` → acknowledged
and the current state / telemetry are sent again at once; the second
`start` and second `stop` are acknowledged but change nothing (one run, one
summary).  No command ever moves the car: commands only start or stop the
autonomous mission.

### T4 - Broker outage during a run
1. `start`, wait 10 s.
2. Administrator PowerShell: `Stop-Service mosquitto`, wait 15 s,
   `Start-Service mosquitto`.  (The recorder must be restarted after the
   broker comes back.)
3. Wait 20 s, `stop`.

**Pass:** serial shows `[mqtt] not connected ...` then `CONNECTED` within
15 s of the broker restarting; `[monitor] ... dropped=0` throughout; events
raised during the outage (barcode / hump / obstacle) arrive after the
reconnect with their original timestamps `t`, or are counted in `ev_drop`.

### T5 - WiFi outage during a run
1. `start`, wait 10 s.
2. Turn the hotspot off for 30 s, then on (keep the hotspot screen open).
3. Wait until reconnected + 20 s, `stop`.

**Pass:** `[wifi] link DOWN`, then `link UP` and `[mqtt] CONNECTED` within
30 s of the hotspot returning; `robot/status offline` is published by the
broker (last will) during the outage and `online` after; the fake run keeps
going throughout (`dist_mm` continues to rise); `[monitor] dropped=0`.

### T6 - WiFi unavailable at boot
1. Hotspot off.  Power-cycle the Pico.  Wait 60 s.
2. Hotspot on.

**Pass:** `[wifi] not associated ...` and `rejoin attempt` lines with
growing waits (5 s → 10 s → 15 s cap); after the hotspot returns:
`link UP`, IP address, `[mqtt] CONNECTED`, all within 30 s.

### T7 - Control-timing interference
The `[probe]` line (every 10 s) reports how late a 10 ms control-style task
starts (priority 5, below the radio task).

1. Hotspot off (radio idle), record 3 probe lines.
2. Hotspot on, `start` (MQTT busy), record 3 probe lines.

**Pass:** in both conditions `>5ms=0`; record `avg` and `max` for the
report.  The difference between (1) and (2) is the cost of the
communication subsystem to control timing.

### T8 - Soak
1. `start` and leave running for 30 minutes (recorder running), then `stop`.

**Pass:** no heartbeat gaps or reboots in the report; `ev_drop` 0;
`[mqtt] ... errors=0` on the serial console; `>5ms=0` on every probe line.
The report's per-topic rates are the figures used for the resource section
of the documentation.

## Results

| Test | Date | Build | Result | Evidence | Notes |
|---|---|---|---|---|---|
| T1 | 2026-09-27 | 00:45:24 | Pass | serial log (D1 retest) | 4/4 boots incl. 3 quick replugs: `[mqtt] CONNECTED` ~6-7 s after power-up |
| T2 | 2026-09-27 | 01:06:54 | Pass | `20260927_010815_T2_run.log` + `.report.txt` | 63.7 s run; summary barcodes 11 / humps 8 / obstacles 5 = events in log, `max_hump_mm` 48; heartbeat 1.00/s, 0 missing, sent-interval max 1.02 s; `ev_drop` 0; total 4.1 msg/s, 328 B/s |
| T3 | 2026-09-27 | 01:06:54 | Pass | same log | `hello` → unknown/ok 0; `status` → ack + info resent; 2nd `start` / 2nd `stop` acknowledged, one run and one summary only; ack round trip avg 72 ms, max 107 ms |
| T4 | 2026-09-27 | 01:06:54 | Pass | `20260927_011514_T4_broker.log` | (a) Idle restart at 01:19:06: `offline` → `online` 3 s after broker returned; 2 QoS 0 heartbeats lost as designed. (b) During a run, broker down ~38 s (01:20:35-01:21:13): 15 events raised in the outage (Pico `t` 761310-797610) all delivered within 0.4 s of reconnect, in order, with original timestamps; `ev_drop` 0; run continued (`run_ms` 129030). Note: used 15 of 16 event-queue slots. |
| T5 | 2026-09-27 | 00:59:35 | Pass | serial log (D4 retest) | Hotspot off ~60 s: rejoin fallback after 20 s, attempts with growing waits; hotspot on → `link UP` → `[mqtt] CONNECTED` ~2 s; radio `polls` rising throughout; `dropped=0` |
| T6 | 2026-09-27 | 23:57 (26th) | Pass | serial log | Hotspot off at boot: waits 10 → 15 → 15 s; `link UP` → `CONNECTED` ~12 s |
| T7 | 2026-09-27 | 01:06:54 | Partial pass | serial log | 10 ms task (priority 5) wake-up latency per 1000 samples. MQTT down: avg 1-7 us, max 0.23 ms. Connected, idle: avg ~120 us, max 1.8 ms. Busy fake run: avg ~185 us, max 2.0 ms, ~20/1000 over 1 ms, 0 over 5 ms. **Rare peaks over 5 ms:** 6.4 ms while joining (D1 retest, 12/1000 in the first window after connect) and 8.4 ms once in 1000 samples while connected and idle (retest later on 2026-09-27). Cause: the radio driver busy-waits inside the priority-4 radio task. Mitigation: time-critical loops (motor PID) run above priority 4 - to be confirmed with Buddy 2's real loop at integration. |
| T8 | 2026-09-27 | 01:06:54 | Pass (with observation) | `20260927_013319_T8_soak.log` + `.report.txt` | 30 min run (1802 s), 14 553 messages, avg 8.0 msg/s, 694 B/s. Heartbeat 1810 received, 0 missing, 0 reboots; `ev_drop` 0; summary counts = events in log (barcodes 327, humps 234, obstacles 148). Observation: 20 delivery stalls of 2.5-4.7 s (phone hotspot); during them publishing backed up, so some heartbeats left the Pico up to 2.7 s late (max 3.74 s interval by Pico clock) - nothing was lost. |

### Earlier results (development builds, 2026-09-26/27)

| Test | Result | Observed |
|---|---|---|
| Broker restart | Pass | Reconnected automatically after `Start-Service mosquitto` |
| Hotspot off 34 s | Pass | `link DOWN` → `link UP` → `CONNECTED`; `[monitor] dropped=0` |
| Hotspot off at boot | Pass | Rejoin waits 10 → 15 → 15 s; `link UP` → `CONNECTED` in ~12 s |
| Fake telemetry run | Pass | All topics, summary `max_hump_mm` correct, `ev_drop` 0 |

### Defects found by testing

| # | Found in | Symptom | Cause | Fix | Verified |
|---|---|---|---|---|---|
| D1 | T1 (replug) | After a quick power-cycle the Pico joined WiFi (`link=1`) but never got an IP address, so MQTT never connected. Did not happen after flashing. | lwIP's DHCP address-conflict probe (ACD): the phone hotspot still answered ARP for the Pico's previous association, so every offered lease was declined and DHCP backed off 10 s, forever. | `LWIP_DHCP_DOES_ACD_CHECK 0` in `lib/libnet/lwip/include/lwipopts.h` (as in Raspberry Pi's Pico W examples). Added a `[net] waiting:` diagnostic line every 10 s while not connected. | Build `Sep 27 2026 00:45:24`: 4/4 boots, including 3 quick replugs, reached `[mqtt] CONNECTED` about 6-7 s after power-up. |
| D4 | T5 (after replug) | After a replug, heartbeats slowed from 1.0 s to 1.7 s; then, after a hotspot off/on, the Pico never rejoined. | The signal-strength (RSSI) read failed after the replug and was retried on every 2 ms radio poll. Each attempt is a blocking request to the radio chip, which starved the radio loop (so the rejoin logic never ran). | RSSI read at most once every 5 s. Added `[net]   radio: polls=...` diagnostic showing the radio loop is alive. | Build `Sep 27 2026 00:59:35`: replug, hotspot off ~60 s, on → rejoin attempts with growing waits, `link UP` → `[mqtt] CONNECTED` in ~2 s; `polls` rose throughout. |
| D2 | Step 5 | Rejoin back-off never grew (always 2 s). | Driver reports "joined" as soon as a join *starts*. | Success now requires the netif link to be up. | Hotspot-off-at-boot test: waits 10 → 15 → 15 s. |
| D3 | Step 5 | MQTT could stay off for good if the first DHCP lease took over 30 s. | MQTT start was gated on a one-shot DHCP flag. | MQTT checks link + address itself on every poll. | Hotspot-off-at-boot test connected. |
