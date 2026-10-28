# Stage 3 loaded geometry calibration — 2026-10-27

Tester: Suen Hong  
Branch: `buddy2-motion-catchup`  
Vehicle mass: approximately 665 g (0.665 kg)  
Current provisional CPR: left 2536, right 2529 counts/revolution  
Current provisional wheel diameter: 65.0 mm per side  
Measured loaded axle height: approximately 30 mm  
Derived approximate loaded diameter: 60 mm

The axle-height measurement is approximate, so it is a cross-check rather
than sufficient evidence to change the configured diameter by itself.

## Required configuration

| Item | Recorded value |
| --- | --- |
| Complete-car mass | Approximately 665 g |
| Powerbank physically installed | Confirm before rolling test |
| Loaded axle height | Approximately 30 mm |

Battery charge, tyre condition and track width are not required for this short
rolling calibration. Battery data belongs to powered testing, and effective
track width belongs to turn calibration.

## Manual loaded rolling-distance method

1. Fit the complete 665 g vehicle configuration with the powerbank in its
   normal position.
2. Flash `motion_encoder_test` and connect the computer by USB only. Keep the
   powerbank electrically disconnected but physically installed.
3. Mark a straight 300 mm lane on the floor and use the axle
   centre as the vehicle reference point.
4. Record the starting encoder line, push the chassis slowly and straight for
   exactly 300 mm without lifting or deliberately turning either wheel, then
   record the ending encoder line.
5. Repeat three forward trials. Subtract each starting count from its ending
   count; do not assume a fresh boot starts at exactly zero.
6. Perform one reverse trial to reconfirm negative encoder signs.

Keep the USB cable loose so it does not pull the vehicle. This establishes
loaded rolling geometry; powered slip and commanded-distance accuracy are
tested later.

## Forward 300 mm results

| Trial | Start L | End L | Delta L | Start R | End R | Delta R | Invalid L/R |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | | | | | | | |
| 2 | | | | | | | |
| 3 | | | | | | | |

The tester supplied an aggregate result after three consecutive 300 mm
forward movements (900 mm total):

| Distance | Start L/R | End L/R | Delta L/R | Invalid L/R |
| ---: | --- | --- | --- | --- |
| 900 mm forward | 0 / 0 | 12350 / 11967 | 12350 / 11967 | 0 / 0 |

Calculated from that aggregate:

| Wheel | Effective mm/count | Effective diameter |
| --- | ---: | ---: |
| Left | 0.072874 | 58.83 mm |
| Right | 0.075207 | 60.54 mm |

The mean effective diameter is approximately 59.68 mm, consistent with the
rough 30 mm loaded axle-height measurement. The left/right difference is
about 2.9 percent, so a controlled return-to-start repeat is required before
changing configuration constants.

For each wheel and trial:

```text
effective_mm_per_count = 300 mm / absolute_encoder_delta
effective_diameter_mm = effective_mm_per_count * counts_per_revolution / pi
```

## Reverse sign check

| Distance | Start L/R | End L/R | Delta L/R | Result |
| ---: | --- | --- | --- | --- |
| 300 mm reverse | 12350 / 11967 | 6817 / 7354 | -5533 / -4613 | Sign passed; geometry rejected |

The reverse signs are correct and invalid transitions remained `0/0`, but the
left/right reverse deltas differ by approximately 20 percent. They also do not
agree with the forward calibration. This result is not used to update wheel
diameter; repeat by moving 300 mm forward and then returning along the same
marked line to the original axle position.

## Controlled forward-and-return attempt

After a reset, the tester rolled 300 mm forward and then returned the vehicle
to the starting area:

| Position | Left | Right | Invalid L/R |
| --- | ---: | ---: | --- |
| After 300 mm forward | 4354 | 4302 | 0 / 0 |
| After returning | -742 | -153 | 0 / 0 |

The forward-only values would imply 55.62 mm left and 56.14 mm right effective
diameters, which do not agree closely enough with the earlier 900 mm aggregate.
The final offsets correspond to approximately 51 mm of extra left-wheel
rotation and 11 mm of extra right-wheel rotation. This shows the manually
returned path did not make both wheels retrace the same distance; it does not
indicate invalid encoder transitions.

The short encoder-roll method is therefore not used for final calibration.
The replacement method is a loaded rolling-circumference measurement: with
the electronics unpowered, mark one tyre at the floor contact point, roll the
complete vehicle for exactly three revolutions of that wheel, and measure the
travel distance. Repeat using the other wheel as the revolution reference.
This removes the USB-cable limit and does not require the vehicle to return to
an encoder zero.

## Controlled forward-and-return repeat

The tester repeated the 300 mm forward-and-return movement while ensuring
that both wheels rolled during the reverse path:

| Position | Left | Right | Invalid L/R |
| --- | ---: | ---: | --- |
| After 300 mm forward | 4212 | 4178 | 0 / 0 |
| After returning to start | 34 | -30 | 0 / 0 |

The reverse deltas were -4178 left and -4208 right. Final count offsets were
0.81 percent of the forward count on the left and 0.72 percent on the right.
This is sufficiently close to zero for the manual endpoint method.

| Wheel | Forward diameter | Reverse diameter | Bidirectional mean | Configured value |
| --- | ---: | ---: | ---: | ---: |
| Left | 57.495 mm | 57.963 mm | 57.729 mm | 57.7 mm |
| Right | 57.803 mm | 57.391 mm | 57.597 mm | 57.6 mm |

The loaded configuration is therefore updated to 57,700 um left and 57,600 um
right. These are effective rolling diameters for the approximately 665 g car,
not unloaded tyre dimensions. Powered distance accuracy remains a Stage 6
test and may justify a later refinement for slip.

## Results

The controlled bidirectional repeat supports the configured loaded diameters
of 57.7 mm left and 57.6 mm right. The earlier path-sensitive readings remain
documented but are not used in the final calculation.
