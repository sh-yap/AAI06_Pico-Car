# Stage 12 — Robustness and Resource Optimisation

## Variation/fault testing
- [ ] Bright lighting
- [ ] Dim lighting
- [ ] Different car speeds
- [ ] Slight track imperfections
- [ ] Low battery
- [ ] Wheel slip
- [ ] WiFi loss
- [ ] MQTT broker loss
- [ ] Ultrasonic timeout
- [ ] Invalid sonar reading
- [ ] Temporary line loss
- [ ] IMU communication failure
- [ ] Barcode at multiple speeds
- [ ] Obstacle left/centre/right
- [ ] Multiple humps

## Embedded quality checks
- [ ] No uncontrolled blocking loops
- [ ] Sensor calls have timeouts
- [ ] ISR work is minimal
- [ ] Queues are bounded
- [ ] Buffers are bounded
- [ ] Dynamic allocation is minimal
- [ ] Stack sizes measured
- [ ] Task timing measured
- [ ] Shared state protected
- [ ] Stale sensor data detected
- [ ] Compiler warnings fixed
- [ ] Peer review performed
- [ ] Static analysis used where possible
