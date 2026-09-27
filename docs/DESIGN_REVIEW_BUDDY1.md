# Week 6 Design Review - Buddy 1 input (Communications)

Buddy 1's content for each section of the team Design Review template.
Copy the rows into the team document and keep names consistent with the
other buddies' sections.  Every number marked *measured* comes from the
test logs in `evidence/buddy1/` and `docs/TEST_PLAN_COMMS.md`.

Diagrams (PNG for Word, SVG for editing): `docs/img/comms_statechart.png`,
`docs/img/comms_interaction.png`.

---

## Team Role and Subsystem Ownership

| Buddy | Student ID | Primary subsystem / responsibility |
|---|---|---|
| Buddy 1 | *(your ID)* | WiFi, MQTT communication, telemetry and commands: WiFi connection and recovery, MQTT client, topic and message design, telemetry publishing, command reception, heartbeat/status.  Owns the CYW43439 radio (through the Radio Service Task) and the `comms` API used by all other subsystems. |

---

## 1. Requirements

### 1.1 Functional requirements (Buddy 1)

| ID | Requirement |
|---|---|
| FR-B1-01 | The robot shall join the configured 2.4 GHz WPA2 network and connect to the MQTT broker automatically after power-up, with no user action. |
| FR-B1-02 | The robot shall publish telemetry over MQTT: robot state, motor speeds, encoder counts, distance travelled, line sensor status, and terrain / hump data. |
| FR-B1-03 | The robot shall publish an event for each barcode decoded (code and navigation command), each hump (peak height), each obstacle profile (distance, bearing, width, left/right clearance, recommended action) and each motion event (e.g. impact). |
| FR-B1-04 | At the end of a run the robot shall publish a run summary including the highest hump peak, distance travelled, run time and event counts; the summary shall remain available on the broker (retained). |
| FR-B1-05 | The robot shall accept the commands `start`, `stop` and `status` on `robot/command` and acknowledge every command.  Commands shall never drive the motors directly; they only start or stop the autonomous mission. |
| FR-B1-06 | The robot shall publish a heartbeat every second (state, fault flags, dropped-event count) and an `online` / `offline` status; `offline` shall be published by the broker (MQTT last will) if the robot disappears. |
| FR-B1-07 | The robot shall detect loss of the WiFi network or the broker and reconnect automatically, without restarting. |

### 1.2 Non-functional requirements (Buddy 1)

| ID | Requirement | Measured (evidence) |
|---|---|---|
| NFR-B1-01 | No `comms` API call shall block the calling task; if a queue is full the item is dropped and counted, never waited for. | By design (all kernel calls use `TMO_POL`); `ev_drop` 0 in all tests. |
| NFR-B1-02 | WiFi/MQTT activity shall not delay a 10 ms control-style task (priority 5) by more than 5 ms. | **Not always met.** Busy run: avg 0.19 ms, max 2.0 ms (T7).  Rare peaks above 5 ms: 6.4 ms while joining, and 8.4 ms once in 1000 samples while idle (2026-09-27 retest).  Cause: the radio driver busy-waits inside the priority-4 radio task.  Mitigation: time-critical loops run above priority 4 (section 3, A-C3). |
| NFR-B1-03 | A command shall be acknowledged within 200 ms. | avg 72 ms, max 107 ms (T3); one 371 ms outlier while the phone hotspot was congested (T4 log). |
| NFR-B1-04 | Each telemetry topic shall be published at most 5 times per second, and unchanged values at most every 2 s. | Whole link during a run: 4.1 msg/s, 328 B/s (T2). |
| NFR-B1-05 | After the network or broker returns, the MQTT session shall be restored within 30 s; after power-up, within 30 s. | 2 s after WiFi link up (T5); 3 s after broker restart (T4); 6-7 s after power-up (T1). |
| NFR-B1-06 | Events raised while disconnected shall be delivered after reconnection in their original order with their original timestamps, or counted as dropped. | 38 s broker outage: 15 events queued and all delivered in order within 0.4 s of reconnect (T4). |
| NFR-B1-07 | The communication code shall use no dynamic memory allocation and fit in < 16 KB flash and < 4 KB RAM (excluding the lwIP stack). | ~12.3 KB flash, ~2.7 KB RAM (comms + MQTT session + lwIP MQTT client). |

---

## 2. Hardware Integration Pin Layout (Buddy 1 rows)

| Device / Module | Type | Signal Name | Sensor label | Pico GPIO Pin | Interface | Notes / RTOS Task Owner |
|---|---|---|---|---|---|---|
| CYW43439 WiFi (on Pico W) | Comms | WL_ON | WL_REG_ON | GP23 | GPIO out | Radio power enable. Radio Service Task only. Internal to Pico W, not on the header. |
| CYW43439 WiFi | Comms | WL_DATA | WL_D | GP24 | PIO SPI (bidirectional) | Radio Service Task only.  The chip's host-wake interrupt is not used: the radio task polls every 2 ms, leaving the GPIO interrupt free for Buddy 2's encoders. |
| CYW43439 WiFi | Comms | WL_CS | WL_CS | GP25 | PIO SPI | Radio Service Task only.  (On a plain Pico GP25 drives the on-board LED; on the Pico W that LED is wired to the WiFi chip instead.) |
| CYW43439 WiFi | Comms | WL_CLK | WL_CLK | GP29 | PIO SPI | Radio Service Task only.  GP29 cannot be used as ADC3 while WiFi is on. |

Shared-resource notes for the other buddies:

- The radio claims **one PIO state machine and two DMA channels** at boot
  (whichever are free first).  Any subsystem using PIO or DMA (e.g. the
  ultrasonic or encoder capture) must use different ones.
- GP23, GP24, GP25 and GP29 are wired inside the Pico W and are not on
  the header, so no car wiring uses them; they are listed so the pin map
  is complete.  **Buddy 1 uses no header pins.**
- The kernel's liveness LED is on GP16 (`BOARD_LED_PIN` in
  `include/sys/sysdepend/pico_rp2040/sysdef.h`); it can be moved or
  dropped if another subsystem needs GP16.

---

## 3. RTOS Task Table (Buddy 1 rows)

In micro T-Kernel a **lower number is a higher priority**.

| Task | Purpose | Inputs | Outputs | Trigger | Priority |
|---|---|---|---|---|---|
| Radio Service (`cyw43_utk_task`) | Owns the CYW43439 and lwIP: WiFi join and rejoin, DHCP, MQTT session, publishing, command reception | Radio events; `comms` snapshots, event queue, state/fault words | MQTT messages; command queue; radio status snapshot | Periodic poll, 2 ms (10 ms kernel tick), pinned to processor 1 | 4 |
| WiFi Status (diagnostics) | Prints link, MQTT and rejoin state to the console | Radio status snapshot | Console lines | Periodic, 200 ms | 8 |

The `comms` API (`app_program/comms.h`) is **not a task**: other tasks call
it and it returns immediately.

**Priority and ownership justification (Buddy 1):**
The Radio Service Task is the only code that touches the CYW43439 and lwIP
(lwIP runs with `NO_SYS=1`, i.e. one execution context), so no locks are
needed around the network stack.  It runs at priority 4: below the USB
console service (3) so the console cannot overflow, and above the
subsystem tasks so that the 2 ms poll, lwIP timers and MQTT keep-alive
stay on time.  Because it sits above the subsystem tasks, its CPU use
delays them: measured at most 2.0 ms in a busy run (NFR-B1-02), but the radio
driver's busy-waits reached 6.4 ms once while joining.  **Recommendation to
the team:** control loops that cannot tolerate ~2-6 ms of jitter (motor
speed PID) should run at a higher priority than 4 (e.g. 2-3) and keep each
cycle short; all other subsystem tasks can run below 4.

---

## 4. Task Interaction Diagram (Buddy 1 part)

Insert `docs/img/comms_interaction.png`.

**Brief explanation of the main communication paths:**
Subsystem tasks never touch the radio.  Periodic data (motion, line,
terrain) is written into three *snapshot slots* protected by a sequence
lock: the writer never waits and the latest value wins, which suits
telemetry.  Discrete events (barcode, hump, obstacle, motion, run
begin/end) go into a 16-entry kernel message buffer so none is lost or
reordered; if it is full the event is dropped and counted (`ev_drop` in the
heartbeat).  The vehicle controller writes the robot state and fault flags
as single 32-bit words and reads `start`/`stop` commands from a 4-entry
command message buffer.  The Radio Service Task drains all of these, turns
them into JSON MQTT messages (rate-limited, unchanged values skipped) and
publishes them; received commands flow back the other way.

---

## 6. Subsystem Statechart (Buddy 1)

| Field | Student Entry |
|---|---|
| Subsystem name | Communication subsystem: WiFi link + MQTT session |
| Related RTOS task(s) | Radio Service Task (`cyw43_utk_task`); `comms` API used by all subsystem tasks |
| Purpose | Keep the robot connected to the WiFi network and MQTT broker, recover automatically from outages, and publish telemetry/events and receive commands without ever blocking the control tasks. |
| Reason a statechart is needed | Connection handling is event-driven with timeouts and retries (join failures, link loss, broker refusal, back-off timers).  Two interacting machines, WiFi link and MQTT session, must be explicit so every failure has a defined recovery path. |

Insert `docs/img/comms_statechart.png`.

**Brief explanation of states, events and fault handling:**
*WiFi link:* after boot the radio scans and joins (JOINING).  On link up
it is CONNECTED and DHCP supplies an address.  If the access point
disappears (LINK_LOST) the radio firmware first retries on its own; if the
link is still down after 20 s, or the join fails outright (e.g. hotspot off
at boot), REJOINING calls the join again with growing waits (2, 5, 10,
15 s cap) until the link returns.  *MQTT session:* IDLE until the link is up
with a valid IP, then CONNECTING (TCP + MQTT CONNECT with a last-will
message).  In CONNECTED it announces `online`, subscribes to
`robot/command`, and every 20 ms publishes up to 4 queued messages.  Any
refusal, timeout (~9 s), link loss or broker loss leads to BACKOFF, which
waits 1, 2, 4 ... 30 s before retrying, and retries immediately when the
network comes back.  During any outage the car keeps running; telemetry
is simply not sent (latest values are resent on reconnect) and events
wait in the queue.

---

## 7. Assumptions (Buddy 1)

| ID | Assumption | Risk if wrong | Validation method | Owner | Status |
|---|---|---|---|---|---|
| A-C1 | A 2.4 GHz WPA2 network and an MQTT broker on the same network are available at the demonstration (phone hotspot + laptop running Mosquitto). | No telemetry during the demo, although the car still runs autonomously. | Ask the lecturer whether an official broker or network is provided; rehearse the demo with the team's own hotspot and laptop broker; broker address is one line in `config/mqtt_config.h`. | Buddy 1 | Partially validated (works with own hotspot; venue unconfirmed) |
| A-C2 | Network outages during a run are short (under ~40 s at the test event rate). | Events raised during a long outage overflow the 16-slot queue and are dropped (visible as `ev_drop`, not silent). | T4: 38 s broker outage, 15 of 16 slots used, all delivered. Real course events are far less frequent than the test generator's. | Buddy 1 | Validated (limit known) |
| A-C3 | Radio activity delays other subsystem tasks by no more than ~2 ms in normal operation. | A motor PID loop below the radio's priority could see timing jitter and less stable speed control. | T7 probe: max 2.0 ms during a busy run (6.4 ms once during join). Re-measure with Buddy 2's real PID loop during integration. | Buddy 1 (with Buddy 2) | Needs validation at integration |
| A-C4 | The telemetry fields and units in `comms.h` match what each subsystem actually produces (mm, mm/s, degrees, ms, integers). | API changes late in the project; mismatched units in the dashboard. | Review `comms.h` with each buddy before integration; fake-data build (`COMMS_DEMO=1`) lets each buddy see their topic. | Buddy 1 | Open |

---

## 8. AI-Supported Design Evidence (Buddy 1)

### 8.1 Pre-submission AI ideation

| Buddy | AI prompt or discussion used | Useful AI ideas | Weak or unrealistic AI ideas | Team decision |
|---|---|---|---|---|
| Buddy 1 | Asked the AI assistant (Claude) to study the provided micro T-Kernel repository and plan the WiFi / MQTT / telemetry subsystem step by step, then to implement and debug it on the Pico W. | Reuse the repository's existing CYW43 + lwIP radio service instead of writing a new stack; use lwIP's built-in MQTT client; keep all network work in the one radio task; give other tasks a non-blocking API (snapshots + event queue) so the network can never stall the car. | Several AI-written details were wrong and only found by hardware testing: rejoin logic trusted the driver's "joined" flag (back-off never grew, D2); MQTT start depended on a one-shot DHCP flag (D3); an RSSI retry on every 2 ms poll starved the radio loop (1.7 s heartbeats, no rejoin, D4); first telemetry version resent unchanged values.  The AI's first explanation for the "no IP after replug" failure (laptop address changed) was disproved with `ipconfig`. | Accepted the architecture; every AI change was accepted only after a hardware test passed.  Added diagnostics (`[net] waiting`, build stamp on boot and on `robot/info`) so each claim could be checked.  All four defects are recorded with cause, fix and retest in `docs/TEST_PLAN_COMMS.md`. |

### 8.2 Team-reviewed architecture freeze (Buddy 1 rows)

| AI or initial idea | Team decision | Engineering reason | Affected artefact |
|---|---|---|---|
| Run MQTT inside the existing radio task (lwIP `NO_SYS`), rather than a separate MQTT task. | Accepted | lwIP must run in one context; a second task would need locks around the whole stack and could deadlock with the radio driver. | Task table, interaction diagram |
| Let the laptop drive the robot remotely over MQTT. | Modified | The brief requires autonomous operation with no remote control; commands are limited to `start` / `stop` / `status`, and the vehicle controller decides how to act on them. | FR-B1-05, statechart, API |
| Send each report as it happens, at whatever rate the subsystem produces it. | Modified | Resource efficiency is assessed: telemetry is rate-limited to 5 Hz per topic and unchanged values are skipped (2 s refresh); measured ~60 % fewer terrain messages. | NFR-B1-04, interaction diagram |
| JSON payloads vs a compact binary format. | Accepted (JSON) | Readable in any MQTT tool for debugging and the demo; measured cost only 328 B/s. Integers only (no FPU on RP2040). | Message format, NFR-B1-04 |
| Keep lwIP's DHCP address-conflict check (ARP probe). | Rejected with reason | After a quick power-cycle the phone hotspot answered the probe for the old lease, so DHCP declined every address forever (D1).  Disabled, as Raspberry Pi's own Pico W examples do. | Assumption A-C1, test plan |

### 8.3 Final gap check (Buddy 1 part)

Reviewer prompt used: *"Act as a strict embedded systems reviewer.  Check
the communication subsystem's requirements, pin/resource usage, task
priority, interaction design, statechart, assumptions and test evidence.
List only gaps the team must review and justify."*

| AI-identified gap | Team response | Action taken | Evidence or reason |
|---|---|---|---|
| MQTT is unauthenticated and unencrypted (anonymous, plain TCP). | Accepted as risk | None for the prototype. | Private hotspot, lab-only data, no remote control of motion (FR-B1-05); TLS would cost RAM/CPU the project does not need. |
| The radio task outranks subsystem tasks, so its busy-waits can delay a motor control loop. | Needs validation | Measured (T7) and recommended that the PID loop run above priority 4. | A-C3; re-measure with Buddy 2's PID at integration. |
| The event queue can overflow in a long outage. | Accepted as risk | Drops are counted in `ev_drop`, never silent; size (16) can be raised cheaply (~40 B per slot). | T4: 38 s outage used 15 of 16 slots with test-generator rates. |
| Broker address is fixed at build time. | Needs validation | Address kept in `config/mqtt_config.h` (one line). | A-C1; confirm venue / broker with the lecturer. |
| The vehicle controller that consumes commands and sets the robot state has no owner. | Needs validation | Raised with the team; `comms_get_command()` and `comms_set_state()` are ready. | Interaction diagram marks it "owner to be assigned". |
| Longest continuous test is short. | Fixed | 30-minute soak (T8) run with the fake-data build: 14 553 messages, 0 heartbeats missing, 0 reboots, 0 events dropped. | `evidence/buddy1/20260927_013319_T8_soak.report.txt`.  Observed 2.5-4.7 s delivery stalls from the phone hotspot, with no data loss; a dedicated router may be steadier for the demo. |

---

## 9. Design Consistency (Buddy 1 input)

| Check | Buddy 1 response |
|---|---|
| Which task owns each hardware device? | The Radio Service Task alone owns the CYW43439 (GP23/24/25/29, one PIO state machine, two DMA channels).  No other task calls the radio or lwIP. |
| Which task has the highest priority, and why? | (Team answer.)  Buddy 1 input: the radio task is priority 4 - below the USB console, above subsystem tasks - and time-critical control loops should be placed above it (see section 3). |
| Which subsystem statechart corresponds to which task? | The communication statechart (WiFi link + MQTT session) corresponds to the Radio Service Task. |
| How are functional requirements reflected? | FR-B1-01, FR-B1-07: WiFi link and MQTT session statechart.  FR-B1-02 to FR-B1-04: `comms` snapshots and event queue, published by the Radio Service Task.  FR-B1-05: command queue read by the vehicle controller.  FR-B1-06: heartbeat and last will in the MQTT session. |
| What changes are still expected after Week 6? | Telemetry fields/units may change after review with each buddy (A-C4); the PID priority recommendation must be confirmed at integration (A-C3); broker details depend on the venue (A-C1). |
