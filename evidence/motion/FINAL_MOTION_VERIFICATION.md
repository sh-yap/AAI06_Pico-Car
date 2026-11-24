# Recorded Motion verification

This consolidates completed observations from the repository's 27-28 October
2026 records. These are historical results, not new tests of the submitted
revision. The current configuration is in
[BUDDY2_MOTION.md](../../docs/project/BUDDY2_MOTION.md).

## Calibration carried into the source

| Test | Recorded observation | Source configuration |
| --- | --- | --- |
| Two marked ten-revolution manual trials | Left mean 2535.7 counts/rev; right 2528.6 | 2536 / 2529 CPR |
| Loaded 300 mm forward/return rolling check, approximately 665 g vehicle | Effective diameters 57.729 / 57.597 mm | 57700 / 57600 um |
| Powered distance scale | 500 mm encoder target gave 570 mm floor travel | Q10 scale 1167 |
| Pivot calibration | 90-degree command at 145 mm track gave about 110 degrees | Effective track 119000 um |

The calibration observations also appear in `motion_config.c` comments.

## Historical physical observations

| Check | Recorded result and scope |
| --- | --- |
| Raised-wheel steering/sign | Forward/reverse signs and positive-right angular convention confirmed |
| Low-speed control, 50 mm/s | Encoder-command averages about 48.1 forward and 50.15 reverse; smooth and fault-free |
| Low-speed control, 80 mm/s | 560 mm in 6.9 s forward and 6.7 s reverse, approximately 81.2 / 83.6 mm/s |
| Corrected reverse distance | 100 mm target, about 100 mm floor travel in 2.00 s |
| Longer distance | 1500 mm target gave about 1510 mm; lateral drift varied between runs |
| Pivot after track correction | Left/right approximately 95 degrees for 90-degree commands; U-turn reached opposite heading with slight endpoint slant |
| Command timeout | Both motors braked and timeout fault remained latched |
| GP21 during U-turn | Stopped after approximately half a turn, two beeps, no self-restart |

The older low-speed runs used PID Kp 0.40, Ki 1.20, Kd 0. The current source
uses Kp 0.60 and revised feed-forward; those earlier observations must not be
presented as a measurement of today's control response.

## Recorded stop timing

Five raised-wheel USB tests used the corrected state-transition timestamp:

| Run | Request-to-state timestamp interval |
| --- | ---: |
| 1 | 10687 us |
| 2 | 204 us |
| 3 | 239 us |
| 4 | 231 us |
| 5 | 10690 us |

All recorded STOPPED, fault zero, PWM zero and invalid transitions zero.
These are software brake/state timestamps, not measured wheel stopping times
or loaded stopping distances. The observed range does not prove a worst-case
deadline under every RTOS or network load.

## Software verification scope

`tests/motion` exercises control calculations, the real Motion loop with
mocks, packed records, encoder snapshots/IRQ decoding, USB-isolation behaviour
and motor errors. `tests/vc` covers mission transitions and fault policy.
The source rejects impossible encoder samples, brakes on timeout/stall,
latches faults and surfaces PWM/brake errors.

Mocks do not measure motor electrical behaviour, sensor voltage compatibility,
physical braking or tyre slip. The former combined-route form contained no
physical result, so no route-success claim is carried into this report.
