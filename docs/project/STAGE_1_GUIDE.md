# Stage 1 — Windows build, flash, and console guide

Each of Skyler, Suen Hong, Wynn, Zhi Hong, and Nicholas completes this flow
independently on their own Maker Pi Pico/Pico W bench platform. Record evidence
in the Session 2 table in [STARTING_CHECKLIST.md](STARTING_CHECKLIST.md).
The final Robo-Pico platform test is a separate team check.

Use the unchanged supplied demo with `SMP=1 CONSOLE=usb_cdc WIFI=none`.
This checks the RTOS and both cores over USB without requiring WiFi credentials.
The current local build environment uses GCC 15.2.Rel1 and SDK 2.3.1; the
lecturer's documented qualified baseline is GCC 13.2.1 and SDK 2.2.0.
Record versions and agree a shared version set; the local compilation checks
do not establish hardware qualification of the newer versions.

## 1. Prepare the bench platform

- Use a USB data cable and the Pico W's micro-USB connection.
- Disconnect external motors, servos, and sensor wiring for this platform test.
- Do not fit an ESP-01 module on GP16/GP17 during the demo: GP16 is its liveness
  output and Pico W already has an onboard radio.
- The Pico W onboard LED is not the demo LED. An external GP16 LED plus series
  resistor is optional; serial output is the required observation here.
- Assign each physical bench board an ID and record its carrier revision.

## 2. Install or locate tools

Already installed tools need only verification.

1. Install [Git for Windows](https://git-scm.com/install/windows).
2. Obtain the ARM cross-compiler and Pico SDK. The official
   [Raspberry Pi Pico VS Code extension](https://github.com/raspberrypi/pico-vscode)
   can manage tool/SDK installations. This project still builds through GNU Make.
   Identify the actual `arm-none-eabi-gcc.exe` directory and SDK checkout.
3. Install [MSYS2](https://www.msys2.org/) at `C:\msys64`. In its **UCRT64**
   terminal install the host compiler and Make:

   ```sh
   pacman -S --needed make mingw-w64-ucrt-x86_64-gcc
   ```

   [make package](https://packages.msys2.org/packages/make),
   [UCRT64 GCC package](https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-gcc).

4. Confirm SDK `lib/tinyusb/src/tusb.c` exists. If absent, run
   `git submodule update --init lib/tinyusb` from that SDK checkout.

## 3. Clone and select the repository

On a new member's PC, clone into a new directory from PowerShell:

```powershell
git clone https://github.com/sh-yap/AAI06_Pico-Car.git
Set-Location AAI06_Pico-Car
git rev-parse HEAD
git status --short
```

Your existing checkout can be used by opening a PowerShell terminal at its
root. Do not run `git pull` with unresolved local edits just to reproduce this
test. Each member records the tested revision; the team uses the same demo
revision. A public clone will only include committed/pushed files, so new
planning files in a working tree are not automatically shared with teammates.

## 4. Configure PowerShell paths and verify

On the current PC the tested setup is below. Other members replace the two
Pico installation paths with their actual paths; `$env:USERPROFILE` resolves
their own Windows user directory.

```powershell
$taskArmBin = "$env:USERPROFILE/.pico-sdk/toolchain/15_2_Rel1/bin"
$taskSdkPath = "$env:USERPROFILE/.pico-sdk/sdk/2.3.1"
$env:PATH = "$taskArmBin;C:/msys64/ucrt64/bin;C:/msys64/usr/bin;" + $env:PATH

git --version
arm-none-eabi-gcc --version
g++ --version
make --version
Test-Path "$taskSdkPath/lib/tinyusb/src/tusb.c"
```

Every version command must succeed and `Test-Path` must return `True`. Run
this setup in each new PowerShell terminal, or configure equivalent persistent
paths. Forward slashes and quoted paths work with the tested Make command.

## 5. Build and inspect the UF2

From the repository root:

```powershell
Set-Location build_make
make SMP=1 CONSOLE=usb_cdc WIFI=none PICO_SDK_PATH="$taskSdkPath" SHELL=C:/msys64/usr/bin/sh.exe -j8
if ($LASTEXITCODE -ne 0) { throw 'Build failed; do not flash a stale UF2.' }
Get-Item mtk3pico_smp1_usb_cdc.uf2
Set-Location ..
```

Require a successful linker/UF2 conversion, not merely the presence of a file
left from an earlier run. Save the build transcript and tool versions. The
default Make flags do not enable a full warning audit; review any diagnostics
that are emitted and agree the coding baseline separately.

## 6. Flash through BOOTSEL

1. Disconnect the board's USB cable.
2. Hold the **BOOTSEL button on the Pico W** while reconnecting USB.
3. Release after Windows shows the `RPI-RP2` drive.
4. Copy `build_make/mtk3pico_smp1_usb_cdc.uf2` onto that drive using File Explorer.
5. The drive normally disappears as the board reboots into the firmware.
6. Open Device Manager → Ports (COM & LPT) and identify the board's USB serial
   COM port. Compare before/after unplugging if several ports are listed.

The reset button on the carrier is not the BOOTSEL button. If the drive does
not appear, check the cable, USB port, and BOOTSEL procedure first.

## 7. Capture 60 seconds of USB serial

Use a serial terminal at 115200, 8 data bits, no parity, 1 stop bit, no hardware
flow control, with DTR asserted. Close other programs holding the port.
Open promptly after reboot to capture the banner.

Alternatively use this built-in Windows PowerShell capture from the repository
root; replace both example values. This procedure requires the actual board
and has not been hardware-tested by the agent.

```powershell
$taskMember = 'Suen Hong' # Replace with the tester's name.
$taskPortName = 'COM5'   # Replace with the board's COM port.
New-Item -ItemType Directory -Force evidence/platform | Out-Null
$taskLogPath = Join-Path (Get-Location) "evidence/platform/$taskMember-usb-console.txt"
$taskSerial = [System.IO.Ports.SerialPort]::new($taskPortName, 115200)
$taskSerial.DtrEnable = $true
$taskSerial.Handshake = [System.IO.Ports.Handshake]::None
try {
    $taskSerial.Open()
    $taskDeadline = [DateTime]::UtcNow.AddSeconds(60)
    while ([DateTime]::UtcNow -lt $taskDeadline) {
        $taskChunk = $taskSerial.ReadExisting()
        if ($taskChunk.Length -gt 0) {
            [Console]::Write($taskChunk)
            Add-Content -LiteralPath $taskLogPath -Value $taskChunk -NoNewline -Encoding UTF8
        }
        Start-Sleep -Milliseconds 100
    }
} finally {
    if ($taskSerial.IsOpen) { $taskSerial.Close() }
    $taskSerial.Dispose()
}
```

Each new capture appends to the named file. Use a different filename for
retests so that evidence from different attempts is not confused.

Expected example (counts may differ):

```text
=== uT-Kernel 3.0 / RP2040 demo ===
SMP build: 2 processors
producer -> [mbf] -> consumer, 2 credits, 500 ms period
[monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc2
```

Check increasing sequence/producer/consumer counts, processor IDs 1 and 2,
zero reported drops, no stalled/failure messages, and output continuing through
the capture. Producer/consumer counts need not match at every observation.
This 60-second observation is a proposed team platform check, not a lecturer
mandated endurance qualification.

If the banner is missed, reboot and recapture promptly. No output: verify the
flashed profile is `smp1_usb_cdc`, correct COM port, DTR, and no competing serial
monitor. The UART mirror on GP0/GP1 can help investigate early boot failures.

## 8. Record evidence and practise Git

For each member record name, board ID/revision, full Git revision, build
profile, compiler/SDK/Make versions, build result, flash result, COM port,
console log, observation duration, and failures/retests in the Session 2 table.

Store shareable logs under `evidence/platform/`. Keep credentials out of logs.
The team's coding baseline is the checklist in the Stage 1 roadmap; every
member agrees to it before starting their subsystem code.

Each member also demonstrates pull/commit/push and participates in a reviewed
merge. One small branch containing their own evidence file is sufficient Git
practice; review only those intended files. For example, from a clean checkout:

```powershell
git pull --ff-only
git switch -c evidence/stage1-suen-hong
git add -- 'evidence/platform/Suen Hong-usb-console.txt'
git commit -m "Record Suen Hong Stage 1 console evidence"
git push -u origin evidence/stage1-suen-hong
```

Adapt the member/branch names. The team reviewer merges the evidence through
the agreed PR workflow, then members pull the result and confirm the shared
main branch still builds. Do not use ZIP files as the code-sharing workflow.

## 9. Final Robo-Pico platform check

When available, repeat build/flash/observe on the final Pico W mounted on
Robo-Pico, with external actuators disconnected. Resolve GP8/GP9 I2C-versus-motor
initialization before any powered motor test. This Stage 1 check establishes
platform startup/console, not motion, sensor, or mission performance.
If the school board is unavailable, keep that Stage 1 item pending.

## 10. Close Stage 1 only with evidence

- All five members independently build, flash, and observe the same RTOS demo.
- A fresh checkout builds; the common main revision builds.
- Tool/SDK setup is recorded, the required template identity is confirmed, and
  ignored build/credential files are checked.
- Every member can pull/commit/push and participate in the team's merge workflow.
- The team agrees the Stage 1 coding baseline and shares code through Git.
- The basic final Robo-Pico platform test passes.

The local fresh-clone USB build passes, but team flashing and runtime evidence
remain pending. Check only completed items in the authoritative
[Stage 1 checklist](../autonomous_robot_project_roadmap_latest/02_STAGE_1_DEVELOPMENT_ENVIRONMENT.md)
and [project gates](../autonomous_robot_project_roadmap_latest/PROJECT_GATES.md).
