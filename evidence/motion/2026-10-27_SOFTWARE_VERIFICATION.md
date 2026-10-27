# Buddy 2 software verification — 2026-10-27

Branch: `buddy2-motion-catchup`  
Toolchain: Arm GNU Toolchain 15.2.Rel1, Pico SDK 2.3.1, GNU Make 4.4.1  
Common firmware options: `SMP=1 CONSOLE=usb_cdc`

This verification followed the CPR update, positive-right Motion/VC sign
normalisation, command validation tests, Motion shared-state serialization,
fault latching and encoder snapshot memory barriers.

## Host regression

| Suite | Result |
| --- | --- |
| `make -C tests/motion test` | 47 checks passed; `-Wall -Wextra -Werror` |
| `make -C tests/vc test` | 150 checks passed, 0 failed |

The Motion checks include independent command origins, encoder wraparound,
measured-period speed calculation, `-90/+90/180` turn geometry, positive-right
velocity/discrete-turn targets, velocity-update reset decisions, command sign
and range validation, impossible encoder-delta rejection, and the measured CPR
configuration.

## Firmware builds

| Profile | Result | ELF text/data/bss bytes |
| --- | --- | ---: |
| `motion_sim_test`, `BOARD=pico` | Passed, no emitted warning/error | 92784 / 152 / 32168 |
| `motion_motor_test`, `BOARD=robo_pico_robot` | Passed, no emitted warning/error | 95612 / 152 / 32312 |
| `motion_encoder_test`, `BOARD=robo_pico_robot` | Passed, no emitted warning/error | 93528 / 152 / 32312 |
| `motion_closed_loop_test`, `BOARD=robo_pico_robot` | Passed, no emitted warning/error | 97784 / 156 / 35556 |
| `vc_car`, no WiFi | Passed, no emitted warning/error | 93828 / 152 / 36408 |
| Exact team `vc_car`, WiFi/DHCP/DNS/binary MQTT | Passed, no emitted warning/error | 366369 / 2708 / 69588 |

Exact integrated profile options:

```text
APP=vc_car BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc
WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
```

The local ignored WiFi and MQTT headers were used. No credential contents are
stored in this evidence file.

## Limitations

Compilation and deterministic host tests do not verify loaded wheel-speed
control, stop latency, motor current, encoder electrical levels, distance
accuracy or turns on the floor. Forward/reverse and physical steering signs
were subsequently checked with raised wheels; see
`2026-10-27_RAISED_WHEEL_CLOSED_LOOP.md`. The remaining checks stay explicit
in the catch-up plan.
