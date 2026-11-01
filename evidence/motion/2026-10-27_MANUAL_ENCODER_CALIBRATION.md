# Manual encoder calibration — 2026-10-27

Tester: Suen Hong  
Branch: `buddy2-motion-catchup`  
Application: `motion_encoder_test`  
Board profile: `BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc`  
Electrical power: computer USB only; powerbank not connected electrically  
Method: wheels rotated manually; tape marks returned each wheel to the same
reference position after ten forward wheel revolutions.

## Raw observations

First run:

```text
[motion-encoder] L=0 R=0 invalid=0/0 total=0 raw=2/0 irq=0
[motion-encoder] L=25358 R=-1 invalid=0/0 total=0 raw=1/1 irq=25407
[motion-encoder] L=25376 R=25278 invalid=0/1 total=1 raw=2/0 irq=50741
[motion-encoder] L=20392 R=20285 invalid=0/1 total=1 raw=2/1 irq=60730
```

The second line followed ten forward left-wheel revolutions. The third line
followed a further ten forward right-wheel revolutions. The last line followed
two reverse revolutions of each wheel. The small count change on the stationary
side is consistent with the user handling the vehicle and is not treated as
encoder cross-coupling.

Second marked ten-revolution run (the start line was not included in the pasted
excerpt; the test was reported as starting from reset/zero):

```text
[motion-encoder] L=25356 R=25293 invalid=0/1 total=1 raw=2/0 irq=50737
```

## Calculation

| Wheel | Trial 1 total | Trial 1 CPR | Trial 2 total | Trial 2 CPR | Mean CPR | Rounded configuration |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Left | 25358 | 2535.8 | 25356 | 2535.6 | 2535.7 | 2536 |
| Right | 25279 | 2527.9 | 25293 | 2529.3 | 2528.6 | 2529 |

Trial 1 right uses `25278 - (-1) = 25279` counts. Repeatability difference was
2 counts over ten revolutions on the left (0.008%) and 14 counts over ten
revolutions on the right (0.055%). Left/right mean CPR differs by about 0.28%;
the Motion configuration retains separate values.

One invalid transition in approximately 50,700 encoder interrupts is about
0.002%. This is a clean bench result, but the final acceptable invalid-rate
limit remains a Stage 0 team decision and powered motor-noise testing remains
required.

## Result and limitations

- PASS: left/right channel mapping.
- PASS: forward-positive and reverse-negative sign handling.
- PASS: repeatable manual counts per wheel revolution.
- PASS: no observed IRQ storm; invalid-transition count was negligible in this
  manual test.
- Updated provisional configuration: left 2536 CPR, right 2529 CPR.
- Not established by this test: encoder electrical voltage compatibility,
  loaded distance per count, motor-powered noise, wheel diameter, straight-line
  accuracy, or final-car motion accuracy.
