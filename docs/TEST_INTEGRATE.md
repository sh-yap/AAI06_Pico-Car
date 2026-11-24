# Integrated car build and operation

The complete application is `APP=vc_car BOARD=robo_pico_robot SMP=1`.
`vc_car_main.c` starts communications, Motion, line/barcode, obstacle and
terrain tasks. The kernel starts the radio service when enabled.

## Build and flash

Install the Arm GNU toolchain, GNU Make, a host C/C++ compiler and Pico SDK.
[Platform setup](GETTING_STARTED.md) describes the supplied build environment.
Create `config/wifi_credentials.h` and `config/mqtt_config.h` from the example
headers, enter local connection settings and set their configuration flags.
These local files are ignored by Git.

From `build_make`, using PowerShell with the toolchain and Make on PATH:

~~~powershell
$robotSdk = "PICO_SDK_PATH=C:/Users/<you>/.pico-sdk/sdk/2.3.1"
$robotShell = "SHELL=C:/msys64/usr/bin/sh.exe"
$robotWifi = "WIFI=cyw43","WIFI_JOIN=1","WIFI_NETIF=1","WIFI_DHCP=1","WIFI_DNS=1","WIFI_MQTT=1"
make APP=vc_car BOARD=robo_pico_robot SMP=1 CONSOLE=usb_cdc @robotWifi $robotSdk $robotShell -j8
~~~

Copy `mtk3pico_smp1_usb_cdc_wifi_mqtt_vc_car_robo_pico_robot.uf2` to the
RPI-RP2 drive while connecting the board with BOOTSEL held. The board restarts.
The USB console prints the build stamp, subsystem initialization results,
VC transitions and MQTT link diagnostics.

The configuration requires USB CDC: GP0/GP1 belong to the ultrasonic sensor.
[Motion](project/BUDDY2_MOTION.md), [terrain](project/BUDDY4_TERRAIN.md) and
[obstacle](project/BUDDY5_OBSTACLE.md) document the actual pin assignments.
An external status LED, if fitted, uses GP19.

## Autonomous run

1. Connect the laptop broker and Pico W to the configured network. The broker
   address must match `mqtt_config.h`.
2. Observe `robot/info`, `robot/status`, `robot/state` and heartbeat.
3. Send the course mapping on `robot/command`, for example
   `map A=LEFT F=RIGHT Z=FORWARD X=UTURN`. Use the assigned course characters;
   these example letters are not fixed in firmware.
4. Put the line sensors over the start line with clearance for the calibration
   pivot, then send `start`. VC performs calibration before line following.
5. Send `stop` to abort. A serious fault enters SAFE_STOP; acknowledgement
   requires STOP, cleared serious faults and stationary wheels.

An absent mapping still permits barcode reporting, but produces no mapped
navigation turns. A failed calibration remains in READY with its fault bits.
Network reconnection is handled by communications independently of navigation.
Current mission decisions and configured timeouts are described in
[Vehicle Controller](project/VEHICLE_CONTROLLER.md).

## Logs and repeatable checks

`tools/mqtt_log.ps1` captures traffic and `tools/mqtt_check.ps1` analyses it.
`tools/robot_decode.ps1` decodes the binary fields described in
[BINARY_FORMAT.md](BINARY_FORMAT.md).
Compare the build stamp and configuration when interpreting a recording.

Host suites are available under `tests/motion`, `tests/vc`,
`tests/obstacle`, `tests/buddy3`, `tests/buddy4_proc`,
`tests/comms` and `tests/recorder`. Run `make -C tests/<suite> test`.
The decoder check is `powershell -File tools/test_robot_decode.ps1`.

For Motion-only firmware, select a test application from
[BUDDY2_MOTION.md](project/BUDDY2_MOTION.md), with `WIFI=none`.
Test applications can intentionally drive the wheels; their build and
completion do not establish a successful full autonomous mission.
