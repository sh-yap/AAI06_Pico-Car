# Buddy 2 Stage 4 software audit - 2026-10-28

Branch: `buddy2-motion-catchup`
Starting commit: `6e6a1d5`
Hardware testing performed in this audit: none

## Decision

Catch-up Stage 4 is not cleared. Existing physical results establish a useful
loaded PWM trend, but do not yet prove five-repeat forward and reverse starts.
The roadmap's Motion Stage 4 / Gate G4 is also open because repeatable speed,
distance, straightness and turn accuracy have not been measured against agreed
limits.

The remaining catch-up Stage 4 hardware run is limited to five forward and
five reverse trials at 200 permille, using 225 permille for a direction only
if 200 fails to start both wheels. The worksheet is in
`2026-10-28_STAGE4_LOADED_MOTOR_RESULTS.md`.

## Software findings corrected

- Motor PWM setup and compare writes were previously ignored by the driver.
  They now return `MOTION_RESULT_HARDWARE`, make a best-effort brake write and
  do not publish the requested drive state as successfully applied.
- Motion now converts a failed stop/completion/fault brake write into a latched
  `MOTION_FAULT_HARDWARE_CONFIG` requiring reinitialisation.
- Encoder GPIO setup errors are now returned. The ISR is installed and its
  state marked ready before encoder GPIO edge delivery is enabled.
- The Stage 4 GP20 image now uses the RP2040 microsecond clock for its
  one-second deadline instead of assuming requested RTOS delays equal elapsed
  wall time. It prints elapsed time, encoder counts and result fields only
  after braking when a console is present.
- The unsupported statement that the old approximate 350-permille travel
  proved 910 mm/s was removed. The 1000 mm/s encoder plausibility bound remains
  provisional until timed wheel data exist.

## Verification

Host environment: MSYS2 GCC, `-std=c11 -Wall -Wextra -Werror -O2`.

| Check | Result |
| --- | --- |
| Buddy 2 host suite | 110 checks passed after low-speed startup coverage |
| Vehicle Controller host suite | 150 checks passed, 0 failed |
| `motion_stage4_test`, SMP USB, no WiFi | Build passed; 0 emitted warnings/errors; text/data/bss 96088/152/32396 |
| `motion_stage5_test`, SMP USB, no WiFi | Build passed; 0 emitted warnings/errors; text/data/bss 96924/156/32392 |
| `motion_closed_loop_button_test`, SMP USB, no WiFi | Build passed; 0 emitted warnings/errors; text/data/bss 97552/164/32540 |
| `vc_car`, SMP USB, no WiFi | Build passed; 0 emitted warnings/errors; text/data/bss 95116/152/36488 |

Host mocks verify deterministic code paths and error propagation. They do not
prove motor voltage/current, encoder electrical levels, physical braking,
loaded starts, distance or steering.

## Deferred tests

No 275-350 permille repeats, battery-range matrix, WiFi/MQTT loading, long
distance runs or turns are required to answer the current Stage 4 question.
Those belong to later closed-loop, distance/turn, integration and robustness
stages.
