# Stage 2 — System Architecture and Subsystem Interfaces

## Goal
Define how the five buddies fit together before implementing algorithms.

## Hardware ownership

| Hardware | Owner |
|---|---|
| Motor driver | Motion subsystem |
| Wheel encoders | Motion subsystem |
| 3 IR sensors | Line/barcode subsystem |
| LSM303DLHC | Terrain subsystem |
| HC-SR04 | Obstacle subsystem |
| Servo | Obstacle subsystem |
| WiFi / MQTT | Communications subsystem |

- [ ] Every device has exactly one owner
- [ ] No other subsystem manipulates that hardware directly

## GPIO/resource map
Create one shared resource table containing:
- [ ] GPIO number
- [ ] PWM slice/channel
- [ ] ADC channel
- [ ] I2C bus
- [ ] UART
- [ ] PIO use
- [ ] interrupt use
- [ ] timer/alarm use

## Public APIs
Examples:
```c
motion_move_forward(...)
motion_move_backward(...)
motion_turn_left(...)
motion_turn_right(...)
motion_stop(...)

line_get_status(...)
barcode_get_command(...)

terrain_get_status(...)

obstacle_get_profile(...)

telemetry_publish(...)
```

## Shared units
Agree on:
- [ ] distance: mm
- [ ] speed: mm/s
- [ ] angles: degrees
- [ ] time: ms or us
- [ ] ultrasonic distance: mm
- [ ] acceleration unit
- [ ] encoder count semantics

## Suggested RTOS tasks
```text
vehicle_controller_task
motion_task
line_task
terrain_task
obstacle_task
communications_task
```

For each task define:
- [ ] priority
- [ ] nominal update rate
- [ ] queue/event inputs
- [ ] outputs
- [ ] timeout policy
- [ ] hardware ownership
- [ ] ISR interaction

## Exit condition
- [ ] System block diagram
- [ ] GPIO/resource map
- [ ] Task diagram
- [ ] Subsystem APIs
- [ ] Units/data contract
- [ ] Error/status types
- [ ] Hardware ownership table
