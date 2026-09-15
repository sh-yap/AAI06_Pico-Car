# Stage 3 — Hardware Driver Bring-Up

## Goal
Prove every physical device works before building assignment algorithms.

## Buddy 1
- [ ] WiFi hardware detected
- [ ] Connects to network

## Buddy 2
- [ ] Motors can be driven
- [ ] Direction control works
- [ ] Encoders can be counted

## Buddy 3
- [ ] All IR sensors can distinguish black and white

## Buddy 4
- [ ] LSM303 data can be read reliably

## Buddy 5
- [ ] Servo moves to commanded angles
- [ ] Ultrasonic measurement works
- [ ] Ultrasonic timeout is implemented

## Required test applications
```text
motor_test
encoder_test
ir_test
imu_test
ultrasonic_test
servo_test
wifi_test
```

## Exit condition
- [ ] Every device can be independently read/controlled
- [ ] Results are repeatable
- [ ] No assignment algorithm is required yet
