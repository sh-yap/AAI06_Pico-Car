# Buddy 2 combined route test - 2026-10-29

## Firmware and controls

- Application: `motion_route_test`
- Board: `robo_pico_robot`, SMP enabled, USB CDC, WiFi/MQTT disabled
- Straight speed: 50 mm/s
- Turn wheel speed: 80 mm/s
- GP20: start the entire route
- GP21: immediate stop at any point
- Build: passed with no emitted warnings or errors; ELF text/data/bss
  `96400/156/32400` bytes
- UF2: `build_make/mtk3pico_smp1_usb_cdc_motion_route_test_robo_pico_robot.uf2`

One ready beep is emitted after boot. Pressing GP20 gives a short button
acknowledgement, followed by a one-second safety delay. The route is:

1. Forward 500 mm.
2. Turn left 90 degrees.
3. Forward 500 mm.
4. Turn right 90 degrees.
5. Forward 1000 mm.
6. Turn right 180 degrees.
7. Forward 1500 mm.

One final result beep means all seven steps completed. Two beeps mean GP21
stopped the route. Codes 3-7 are Motion fault codes, 8 is command rejection,
and repeating 9 indicates initialisation failure. GP20 may start the route
again after a successful completion.

## Physical result

| Item | Observation |
| --- | --- |
| All seven steps completed | Pending |
| Final result beep | Pending |
| GP21 stopped immediately when tested | Pending |
| Unexpected pause or restart | Pending |
| Approximate final position/heading | Pending |

This is a command-transition and practical route check. It does not replace
the later Motion + Buddy 3 line/barcode integration or final statistical gate
evidence.
