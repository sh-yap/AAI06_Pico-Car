# Assignment starting point

Start with **Stage 0: requirements and team organisation**. Finish the shared
platform checks in Stage 1, agree interfaces in Stage 2, and only then begin
hardware-driver implementation in Stage 3.

The existing [roadmap](../autonomous_robot_project_roadmap_latest/README.md)
remains the authoritative stage sequence. Its stage files and checklists have
not been moved or duplicated. This directory adds assignment-specific planning
and evidence to that roadmap.

## Read in this order

1. [Requirements and brief references](REQUIREMENTS.md).
2. [Starting checklist, team table, and hardware audit](STARTING_CHECKLIST.md).
3. [Current gate status](PROJECT_GATES.md) and the
   [authoritative gate checklist](../autonomous_robot_project_roadmap_latest/PROJECT_GATES.md).
4. The relevant stage under `docs/autonomous_robot_project_roadmap_latest/`.
5. [Platform setup](../GETTING_STARTED.md) and
   [RP2040 ownership and safety rules](../PORT_RP2040.md).

## What the repository already supplies

| Area | Existing foundation | Assignment work still needed |
|---|---|---|
| RTOS | micro T-Kernel tasks, synchronization, timing, and SMP | Robot tasks, contracts, scheduling, and application fault handling |
| Application | `app_program/app_main.c` producer/consumer demo and liveness task | Motion, line/barcode, terrain, obstacle, telemetry, and controller modules |
| GPIO/PWM | RP2040 BSP helpers | Board-specific motor/servo drivers and calibration |
| I2C/ADC/UART | Inherited sample device drivers | Sensor drivers, resource agreement, and independent board tests |
| Networking | CYW43 service, lwIP raw API, DHCP/DNS/UDP/TCP development profiles | MQTT session, telemetry schema, heartbeat, and recovery |
| Build | GNU Make, ELF/UF2 packaging, selectable profiles | Reproducible setup and build/flash evidence from every member |

The supplied kernel's published qualification results are background evidence.
They do not establish that our vehicle or our boards pass the assignment gates.

## Working rules

- One final robot controller; the five Maker Pi Pico boards are bench platforms.
- Buddy 2 owns drive-motor access. Sensors report data or recommendations;
  the Vehicle Controller arbitrates and requests movement.
- Keep kernel internals intact while building the assignment application.
- Adding a `.c` file directly under `app_program/` is picked up by the existing
  build. Nested module directories require explicit build integration.
- Learn task creation, bounded message passing, event flags, and fixed pools
  from the demo. New robot shared state must use synchronization; do not copy
  the demo's diagnostic counters as a general concurrency contract.
- Each checked item must have an identifiable document, measurement, log,
  or test result. Reading a requirement does not prove its implementation.
- Collect engineering evidence from the first bench test onward.

## Review scope

The starting assessment uses `AGENTS.md`, both local assignment PDFs (all 21
pages of extracted text), all 14 roadmap stages, gates and open questions,
the root README and platform guides, and the RP2040 build/application/driver
and networking integration paths. It is a project assessment rather than a
line-by-line audit of every kernel implementation or unrelated board port.

The original briefs remain in ignored `docs/private/`. These planning documents
summarize requirements; they do not reproduce the briefs.
