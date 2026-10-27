# CR1 baseline — JSON telemetry (before binary)

**Recorded:** 26 Oct 2026 by Buddy 1 · **Code:** `buddy1-wifi` at `b6f53e2`
(same comms code as `main`; the uncommitted fast-reconnect edits change only
two timing constants, not sizes).

This is the "before" measurement for CR1.  The "after" (binary) numbers are
to be taken with the same build command and the same logs/method.

## 1. Code and RAM size (target build)

Build (from `build_make`):

```
make APP=demo CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 COMMS_DEMO=1 PICO_SDK_PATH=C:/Users/twq61/.pico-sdk/sdk/2.3.1 -j8
```

`arm-none-eabi-size` (bytes; text = flash code, data + bss = RAM):

| Object | text | data | bss |
|---|---:|---:|---:|
| Whole image `mtk3pico_smp0_usb_cdc_wifi_mqtt_demo_demo_pico.elf` | 344 429 | 2 808 | 51 220 |
| `comms.o` (message building, queues) | 5 276 | 8 | 938 |
| `lwip_utk_mqtt.o` (MQTT session) | 1 239 | 0 | 1 787 |

Largest comms symbols (from `arm-none-eabi-nm --size-sort -S`):

| Symbol | Kind | Bytes |
|---|---|---:|
| `comms_link_next` (all message building) | code | 2 064 |
| `add_int` (JSON number writer) | code | 390 |
| `add_text` (JSON string writer) | code | 264 |
| `msg_begin` (JSON header) | code | 104 |
| `g_pending` (one outgoing message: topic + 256 B payload) | RAM | 260 |
| `g_event_buf` (16-event queue) | RAM | 704 |

## 2. Message sizes (JSON payload bytes, excluding topic)

From saved MQTT logs, line endings removed.  Topic length is sent with every
message as well.

| Topic | Topic B | Payload min | max | avg | Messages |
|---|---:|---:|---:|---:|---:|
| robot/telemetry/motion | 22 | 87 | 100 | 99.9 | 8 819 |
| robot/telemetry/terrain | 23 | 97 | 99 | 99.0 | 909 |
| robot/telemetry/line | 20 | 53 | 54 | 54.0 | 1 646 |
| robot/heartbeat | 15 | 66 | 74 | 74.0 | 1 810 |
| robot/state | 11 | 42 | 51 | 51.0 | 651 |
| robot/event/barcode | 19 | 46 | 51 | 48.5 | 327 |
| robot/event/hump | 16 | 43 | 44 | 44.0 | 234 |
| robot/event/obstacle | 20 | 116 | 117 | 117.0 | 148 |
| robot/event/command | 19 | 38 | 40 | 39.5 | 2 |
| robot/event/run | 15 | 32 | 33 | 33.0 | 1 |
| robot/summary | 13 | 102 | 113 | 110.0 | 2 |
| robot/info | 10 | 47 | 49 | 49.0 | 1 |
| robot/status | 12 | 6 | 6 | 6.0 | 1 |

min/max cover both logs; avg and counts are from the 30-minute soak.

Example (motion, 93 B):
`{"v":1,"t":207050,"spd_l_mm_s":201,"spd_r_mm_s":199,"enc_l":40,"enc_r":40,"dist_mm":20}`

## 3. Data rate

T8 soak, 30.5 min (`20260927_013319_T8_soak.log`, 14 553 messages):

| | Bytes | Per second |
|---|---:|---:|
| Payload only | 1 270 503 | 694 B/s |
| Payload + topic | 1 565 703 | 856 B/s |

(MQTT fixed header and TCP/IP overhead are not included.)

## Sources

- `20260927_152653_merge_test2.log` (1-minute demo run, all topics)
- `20260927_013319_T8_soak.log` (30-minute soak)
- Build above, measured 26 Oct 2026.
