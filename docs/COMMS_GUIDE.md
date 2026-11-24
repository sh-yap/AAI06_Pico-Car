# Communications (Buddy 1)

The radio service is the sole owner of CYW43, lwIP and the MQTT session.
Application tasks call `comms.h` to publish snapshots or queue events.
`comms_link.h` is the boundary used by the network service.

| Source | Responsibility |
| --- | --- |
| `app_program/buddy1_vehicle_comms/comms.c/.h` | Snapshots, events, command/acknowledgement queues and scheduling |
| `app_program/buddy1_vehicle_comms/comms_codec.c/.h` | Binary/JSON encoding and packed records |
| `app_program/buddy3_line_barcode/nav_map.c/.h` | Barcode-to-navigation mapping |
| `lib/libnet/lwip/lwip_utk_mqtt.c` | MQTT connection, subscribe, publication and retry |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | Radio polling and WiFi rejoin |
| `tools/robot_decode.ps1` | Laptop binary decoder |
| `tools/mqtt_log.ps1` / `mqtt_check.ps1` | Capture and analysis |

## Data and delivery

Each snapshot has one writer and sequence-checked publication. Events use a
16-slot queue; commands and acknowledgements each use four slots. Queue
operations use polling. Event overflow increments the saturating `ev_drop`
counter instead of silently claiming successful delivery.

Motion and line telemetry have a 100 ms interval during a run; the general
telemetry interval is 200 ms. Unchanged values refresh after 2000 ms.
Heartbeat is scheduled every 1000 ms. These are scheduling intervals;
network backpressure and higher-priority publications can delay delivery.

Publication priority is command acknowledgement, build info, navigation map,
changed state/fault, queued event, debug text, heartbeat, then snapshots.
Telemetry and heartbeat use QoS 0. Events use QoS 1; state and summary are
retained. `robot/status` uses online/offline and the MQTT last will.
A fresh session resends current snapshots.

The default format is binary version 2. `COMMS_FORMAT=json` selects JSON.
[Binary format](BINARY_FORMAT.md) documents field order and widths.
The fixed link buffers allow 32 bytes for a topic and 224 bytes for a payload.

## Commands on robot/command

| Command | Payload |
| --- | --- |
| Start autonomous mission | `start` or one byte 0x01 |
| Stop / acknowledge | `stop` or one byte 0x02 |
| Refresh published state | `status` or one byte 0x03 |
| Set barcode meanings | `map A=LEFT F=RIGHT Z=FORWARD X=UTURN` (example mapping) |

MAP accepts exactly four distinct Code 39 characters and one of each movement.
Invalid mappings preserve the active mapping. No character is mapped at boot.
The mapping lives in RAM; publishing the command retained restores it on
reconnection. The command payload limit is 64 bytes.
`robot/nav_map` reports the active mapping or `none`.
Commands start/stop or configure the autonomous mission; they do not provide
a remote wheel-control interface.

## Operation and checks

Use the [build/operation guide](TEST_INTEGRATE.md). Configure local
`wifi_credentials.h` and `mqtt_config.h` from the example headers.
The robot profile uses GP19 for the external status LED; GP16/GP17 are
left encoder inputs.

Run `make -C tests/comms test` and
`powershell -File tools/test_robot_decode.ps1` for codec and decoder checks.
[Communication test evidence](TEST_PLAN_COMMS.md) describes the scope of the
retained older network recordings; those recordings are not proof of final
integrated motion timing.
