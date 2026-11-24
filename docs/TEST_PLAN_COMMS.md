# Communications verification reference

Current implementation is described in [COMMS_GUIDE.md](COMMS_GUIDE.md).
Host verification uses `make -C tests/comms test` and
`powershell -File tools/test_robot_decode.ps1`. The codec tests cover
golden payload bytes and packed-record round trips; decoder tests use the
corresponding wire format. These are software checks, not radio measurements.

## Retained recording scope

The existing files in `evidence/buddy1/` include development recordings and
analysis from September 2026. They used a fake-data communications application,
an iPhone hotspot and a laptop Mosquitto broker. They do not measure the
submitted integrated car, current binary payload sizes or final motor timing.

The recordings cover startup, command handling, broker restart, hotspot loss,
boot without WiFi and a 30-minute telemetry soak. Interpret each raw file with
its recorded build stamp and setup. The associated older probe task was a
10 ms control-style task, not Buddy 2's production control loop.

## Available verification tools

| Tool or code | Purpose |
| --- | --- |
| `comms_demo.c` with `COMMS_DEMO=1` | Synthetic snapshots/events independent of physical sensors |
| `timing_probe.c` | Task wake-up latency diagnostics |
| `tools/mqtt_log.ps1` | Capture raw MQTT traffic |
| `tools/mqtt_check.ps1` | Analyse rates, heartbeat gaps, events and command timing |
| `tools/soak.ps1` | Capture a bounded communications run |
| `tests/comms/` | Codec, packed-record and API regression checks |

Snapshots represent the latest sample, whereas queued events retain their
original timestamps. Event storage is finite: overflow is visible through
`ev_drop`. QoS 0 samples and heartbeat can be lost. Reconnection and a nominal
heartbeat interval do not guarantee uninterrupted delivery.
