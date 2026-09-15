# Stage 8 — Buddy 1: WiFi, MQTT and Telemetry

## Goal
Provide reliable communications without disturbing real-time control.

## 8A — WiFi
- [ ] Connect to WiFi
- [ ] Detect connection loss
- [ ] Reconnect automatically
- [ ] Reconnect does not block control tasks

## 8B — MQTT
Example topic structure:
```text
robot/status
robot/telemetry/motion
robot/telemetry/line
robot/telemetry/terrain
robot/telemetry/obstacle
robot/event/barcode
robot/command
```

- [ ] Connect to broker
- [ ] Publish test message
- [ ] Subscribe to test topic
- [ ] Recover after broker/network interruption

## 8C — Telemetry data
Include:
- [ ] current robot state
- [ ] motor speed
- [ ] encoder counts
- [ ] line sensor status
- [ ] barcode detected
- [ ] hump measurements
- [ ] obstacle scan results
- [ ] distance travelled

Define:
- [ ] field names
- [ ] units
- [ ] message version
- [ ] update rate
- [ ] missing/invalid-data handling

## 8D — Heartbeat
- [ ] Periodic heartbeat
- [ ] Robot state
- [ ] Uptime
- [ ] Fault state

## 8E — Failure behaviour
- [ ] Turn AP off during operation
- [ ] Robot remains safe
- [ ] Comms task reconnects later

## Deliverables
- [ ] Communication API
- [ ] Telemetry framework
- [ ] MQTT documentation
- [ ] Telemetry demonstration
