# First work sessions

Complete these alongside the original stage checklists. This work packet
supplies the records those checklists need; it does not replace them.

## Session 1 — Close requirements and organise the team

- [x] Read the supplied briefs and roadmap and write a requirements baseline.
- [x] Document unresolved questions in the roadmap question register.
- [ ] All five members review [REQUIREMENTS.md](REQUIREMENTS.md).
- [x] Record the five Buddy assignments supplied by the team.
- [ ] Appoint a controller/integration coordinator and nominate interface reviewers.
- [x] Confirm all five members have Pico W — confirmed by the user.
- [ ] Record physical board revisions/IDs and the final board's installed Pico/Pico W.
- [x] Inspect the professor-supplied equipment datasheets and record documented specifications below.
- [ ] Measure and photograph the hardware; fill the inventory below.
- [ ] Send the outstanding specification questions to the lecturer and record replies.
- [ ] Agree provisional acceptance criteria, recording any lecturer-dependent items.
- [ ] Close Stage 0/G0 only after the team agrees the requirements and ownership.

| Responsibility | Member | Bench board ID | Interface reviewer |
|---|---|---|---|
| Buddy 1 — Communications | Skyler | Pico W; physical board ID UNKNOWN | UNKNOWN |
| Buddy 2 — Motion | Suen Hong | Pico W; physical board ID UNKNOWN | UNKNOWN |
| Buddy 3 — Line/barcode | Wynn | Pico W; physical board ID UNKNOWN | UNKNOWN |
| Buddy 4 — IMU/terrain | Zhi Hong | Pico W; physical board ID UNKNOWN | UNKNOWN |
| Buddy 5 — Obstacle/servo | Nicholas | Pico W; physical board ID UNKNOWN | UNKNOWN |
| Vehicle Controller / integration coordination | UNKNOWN | Robo-Pico tested with the green controller transferred from Suen Hong's Maker Pi Pico; physical board ID UNKNOWN | UNKNOWN |

The controller responsibility belongs to this five-member team; it does not
imply a sixth member or an additional microcontroller.

Information below comes from supplied documents, user reports, and the two
images attached in the conversation. Its source is identified in each row.
User-reported dimensions and labels have not been independently measured or
tested. A document revision is not proof of the revision printed on a team's
board. `UNKNOWN` means further information is needed.

| Hardware item | Documented information / evidence | Information still needed |
|---|---|---|
| Final Robo-Pico | User photos show the green controller transferred from the Maker Pi Pico into the Robo-Pico socket; the user confirms healthy SMP USB runtime. This is the same previously tested controller, not evidence of a separate school-supplied controller. Cytron ROBO-PICO supports Pico/Pico W. Datasheet Rev 1.0, April 2023; board input specification 3.6–6 V; onboard power switch and single-cell LiPo/Li-Ion charging/protection. [D1](../project_datasheets/Robo%20Pico%20Board%20Datasheet.pdf), [D2](../project_datasheets/Robo_Pico.pdf). | Physical PCB/controller IDs and revisions, local photo archive, permanent controller allocation, and actual vehicle power/wiring arrangement. |
| Five Maker Pi Pico boards | All five members have Pico W (user confirmation). Supplied Maker Pi Pico pinout is for PCB Rev 1.2.0 and above. Pico W has CYW43439 2.4 GHz WiFi, 3.3 V GPIO, and a 21 × 51 mm controller PCB. [D3](../project_datasheets/MakerPiPico-Pinout.png), [D4](../project_datasheets/RP-008312-DS-2-pico-w-datasheet.pdf). | Confirm each carrier's product/revision and label each physical board; record accessory/socket connections. |
| Left/right motors | User reports two motors matching [D11](../project_datasheets/PicoCar.pptx) slide 1. It shows a geared motor/encoder assembly with a **PH2.0-6P** connector. Pins 5 and 6 are labelled `Motor line+` and `Motor line-`. No readable motor model/rating label or motor-model datasheet is supplied. | Model/label or school parts-list entry, rated voltage, running/stall current if specified, gear ratio, physical harness orientation/continuity verification, left/right motor-to-terminal mapping, and bench motor driver. |
| Motor driver on Robo-Pico | M1 uses GP8/GP9; M2 uses GP10/GP11. Maximum continuous current 1 A/channel, peak 1.5 A/channel for less than 5 s; PWM frequency limit 20 kHz. Brake/coast truth table below. [D1] pp. 5, 7–8. | Driver IC marking is not identified in the supplied text; actual motor polarity and selected bench driver remain UNKNOWN. |
| Encoders | [D11] slide 1 confirms an encoder integrated with the shown motor assembly and documents the PH2.0-6P interface: pin 1 encoder ground, pin 2 encoder output A phase, pin 3 encoder output B phase, pin 4 encoder power supply, pin 5 motor line+, pin 6 motor line-. This establishes two labelled encoder phase outputs, but not their electrical levels or resolution. No encoder model/resolution sheet is supplied. TCRT5000's possible encoder applications do not identify this wheel encoder. | Obtain the encoder model, supply voltage, output type/voltage, required pull-ups, supplier PPR/CPR definition, counting convention (x1/x2/x4), and motor-shaft versus wheel-shaft reference. Verify connector orientation and wiring on the physical motor. Wheel counts still need bench calibration. |
| Wheels/chassis | Wheel diameter reported by user: 6 cm = **60 mm**. No wheel-centre separation or chassis dimensions supplied. | Confirm both wheels' rolling diameter; measure wheel-centre separation mm, overall vehicle dimensions, and sensor/servo mounting geometry. |
| Servo | Model reported by user: **T-SG90**. No model-specific voltage/current/pulse specification supplied. Robo-Pico servo signals are GP12–GP15; V+ follows the board's motor/servo supply. [D1] pp. 4, 7. | Supplier/model-specific voltage/current and pulse range, chosen servo port, and mounting. Safe angles/centre/settling time require calibration; do not assume every SG90-labelled variant has identical limits. |
| Battery/power | User reports a **20000 mAh powerbank**, input **DC 5 V / 2.1 A**. User clarification confirms **Output 1: 5 V / 2.1 A** and **Output 2: 5 V / 2.1 A**. Combined simultaneous output current is not specified. Robo-Pico's single-cell battery connector capability does not establish use of a separate LiPo here. USB-only motor/servo supply is specified as VUSB minus 0.4 V; Grove +3V3 total output limit is 300 mA. [D1] pp. 3, 7, 9. | Powerbank brand/model and shared/per-port current limit, actual USB/power connection to the vehicle, device supply wiring, any regulator/boost module, and stop/power-switch arrangement. Treat 20000 mAh as the label rating; its voltage basis and usable output capacity are unconfirmed. |
| Three IR sensors | User identifies **MH-Sensor-Series**. First attached image shows that PCB marking and connector labels **VCC, GND, DO, AO**: labelled digital and analogue outputs. The marking does not establish the exact optical component or module voltage rating. Waveshare's manual describes ST188 + LM393, 3.0–5.3 V, DOUT/AOUT/GND/VCC; TCRT5000 is a separate component sheet. Neither is yet confirmed as the exact datasheet for this MH module. [D5](../project_datasheets/Infrared-Reflective-Sensor-UserManual.pdf), [D6](../project_datasheets/tcrt5000.pdf), [D7](../project_datasheets/LM393-D.PDF). | Confirm all three modules match; identify optical/comparator components or obtain the school module specification; confirm supply/output voltages, selected DO/AO use and GPIOs, spacing, and mounting height. Black/white thresholds require calibration. |
| IMU | OKY3231-2 / GY-511 module with LSM303DLHC, accelerometer + magnetometer, no gyroscope. [D11] slide 2 identifies the pictured board as `GY-511 LSM303DLHC 3-axis E-compass Sensor` and labels INT2, INT1, SDA, SCL, GND, 3.3 V, Vin, and DRDY. The module sheet states 3–5 V input. The chip sheet states Vdd 2.16–3.6 V; do not treat chip/module supply ratings as interchangeable. Acceleration ranges ±2/4/8/16 g; magnetic ranges ±1.3 to ±8.1 gauss; I2C 100/400 kHz. [D8](../project_datasheets/OKY3231-2.pdf), [D9](../project_datasheets/lsm303dlhc.pdf) pp. 1, 10, 12, 21–22. | Confirm the team's physical board matches the pictured GY-511 and record its revision, regulator/pull-up arrangement and bus voltage, chosen SDA/SCL pins, mounted orientation, and calibration. |
| Ultrasonic | HC-SR04: DC 5 V, listed working current 15 mA, range 20–4000 mm, measuring angle 15°, dimensions 45 × 20 × 15 mm. Trigger high for at least 10 us; manual recommends more than 60 ms between measurements. Claimed accuracy up to 3 mm is a supplier specification, not our measured accuracy. [D10](../project_datasheets/HCSR04.pdf) pp. 1–2. | Confirm installed module, actual 5 V supply, Trigger/Echo pins, Echo level adaptation and resistor values, mounting, and known-distance calibration. |

### PicoCar presentation review

[D11](../project_datasheets/PicoCar.pptx) contains four image-only slides:

1. Motor/encoder assembly and PH2.0-6P pinout. This supplies the new Buddy 2
   connector and two-phase encoder information recorded above.
2. GY-511 LSM303DLHC reference image and labelled module pins.
3. Generic Raspberry Pi Pico W GPIO/function pinout.
4. Robo-Pico board-layout reference image showing its motor terminals, motor
   status LEDs/test buttons, servo ports, Pico socket, GPIO/Grove ports,
   buttons, buzzer, NeoPixel, power inputs, and switch.

Slides 3 and 4 are consistent with the supplied Pico W and Robo-Pico
datasheets but do not identify the team's physical board revisions or add new
electrical ratings. Slide 1 is a connector/interface reference, not a complete
motor or encoder datasheet. Do not infer unlisted voltage, current, resolution,
gear ratio, or logic-level specifications from appearance.

### Documented motor and board connections

From [D1](../project_datasheets/Robo%20Pico%20Board%20Datasheet.pdf), PDF pages
4–5 and 8 (D2 carries the same Rev 1.0 specifications with different pagination):

| Robo-Pico driver input A / B | Documented behaviour |
|---|---|
| Low / Low | Brake |
| High / Low | Forward, subject to motor wiring polarity |
| Low / High | Reverse, subject to motor wiring polarity |
| High / High | Coast; outputs high impedance |

Motor 1 A/B is GP8/GP9; motor 2 A/B is GP10/GP11. Servo signal ports are
GP12/GP13/GP14/GP15. Maker/Grove port 2 exposes GP2/GP3 for I2C1. Robo-Pico's
NeoPixel uses GP18, buttons GP20/GP21, and buzzer GP22. Its optional battery
sense jumper connects VBAT/2 to GP28/ADC2, sharing that analogue input.
These are documented connections, not a completed application GPIO assignment.

**USB/flashing connection:** Robo-Pico is a carrier with a central Pi Pico
Socket and no onboard USB port (D1 PDF p. 3 Figure 1). Fit a Pico/Pico W to
the socket; BOOTSEL and USB data access are on that controller (D1 PDF p. 6
Figures 2–3). Maker/Grove ports are peripheral connectors, not USB ports.
The user has now installed the green controller from the Maker Pi Pico and
confirmed the basic Robo-Pico runtime check; see Session 2 below.

The supplied Maker Pi Pico [pinout image](../project_datasheets/MakerPiPico-Pinout.png)
shows NeoPixel GP28, buzzer GP18, buttons GP20–GP22, microSD GP10–GP15,
audio GP18/GP19, and ESP-01 socket signals GP16/GP17. Grove pairs are
GP0/1, GP2/3, GP4/5, GP6/7, GP8/9, and GP26/27. Record which accessories are
actually fitted before using those pins. The image illustrates the original
Pico's GP25 LED; the team's Pico W LED is on CYW43439, not GP25.

**Integration issue to resolve before a powered Robo-Pico motor test:** the
supplied startup code and sample I2C0 driver both configure GP8/GP9 as I2C.
Robo-Pico connects these pins to motor 1. The Stage 2 resource/initialization
plan must remove that conflict, including startup initialization, before
using the drive motor. Merely selecting different IMU application pins is
insufficient. No startup/kernel/driver code is changed by this checklist update.

For LSM303DLHC the documented 7-bit I2C addresses are `0x19` (accelerometer)
and `0x1E` (magnetometer), from D9 pp. 21–22; confirm them on the actual bus.
The HC-SR04 sheet specifies a 5 V module with TTL Echo and does not guarantee
a 3.3 V-limited Echo. Verify a suitable divider/level shifter before connecting
Echo to Pico W. IR and IMU module supply ratings likewise do not prove their
signal outputs/pull-ups are safe at the selected supply voltage.

### Information to supply next

Fill these with labels, measurements, or `UNKNOWN`; calibration can wait until
the respective bench-test stage.

- Controller coordinator and interface reviewer for each Buddy: UNKNOWN.
- Each Maker Pi Pico carrier revision/physical ID and final Robo-Pico revision: UNKNOWN. The transferred green controller is now tested on Robo-Pico; record its ID and permanent allocation.
- Motor model/voltage/current/gear ratio, encoder supply/output levels and resolution, physical harness verification, left/right mapping, and bench motor driver: UNKNOWN. [PicoCar.pptx](../project_datasheets/PicoCar.pptx) slide 1 now documents the PH2.0-6P pinout and A/B encoder phases, so connector function and encoder presence/channel count are no longer unknown.
- Wheel-centre separation and chassis size in mm: UNKNOWN. Wheel diameter is reported as 60 mm; confirm both wheels' rolling diameter during calibration.
- T-SG90 rated supply/current/pulse limits and selected Robo-Pico servo port: UNKNOWN; model has been recorded.
- Powerbank brand/model/current-sharing specification and vehicle power wiring: UNKNOWN. Capacity 20000 mAh, input 5 V / 2.1 A, and two outputs each labelled 5 V / 2.1 A are reported and clarified.
- Whether all three MH-Sensor-Series modules match, their optical/comparator components, rated supply/output voltages, selected DO/AO use, and spacing/height: UNKNOWN. Series marking and DO/AO labels are visible.
- Actual sensor wiring/power table, GY-511 orientation, Echo divider/level shifter, and GP28 battery-sense jumper state: UNKNOWN.
- The motor/encoder interface image is archived in [PicoCar.pptx](../project_datasheets/PicoCar.pptx) slide 1. A local archive filename for the supplied IR image, photos of other hardware/labels, and available multimeter/ruler/USB-UART adapter remain UNKNOWN.

The datasheet folder has nine PDFs, one standalone pinout image, and one
four-slide [PicoCar presentation](../project_datasheets/PicoCar.pptx). The
presentation archives the motor/encoder interface image plus reference images
for the GY-511, Pico W pinout, and Robo-Pico layout. It does not provide an
identified motor or encoder model, motor electrical ratings, encoder electrical
levels/resolution, or servo, powerbank, and wheel/chassis specification sheets.
The supplied IR image is still not archived locally. Model-specific ratings and
physical measurements still need the records listed above.

## Session 2 — Each member builds, flashes, and observes the demo

Follow the [complete Windows Stage 1 guide](STAGE_1_GUIDE.md) for installation,
build, BOOTSEL flashing, console capture, evidence, Git practice, and exit checks.

1. Record compiler, Make, host compiler, SDK version, and firmware revision.
2. Build the unchanged supplied demo with `SMP=1 CONSOLE=uart`.
3. Build `SMP=1 CONSOLE=usb_cdc` with an explicit SDK path if USB serial is needed.
4. After checking that the demo's GPIOs are safe on that board, use BOOTSEL and
   copy the matching UF2 to `RPI-RP2`.
5. Capture the banner and repeated monitor output. On SMP expect
   `made_on=prc1 seen_on=prc2`, progressing counts, and no reported drops/stalls.
6. Repeat independently for every member and bench board. Verify the basic
   platform flow on Robo-Pico, then perform a separate fresh-clone build.

Local build success is recorded in [gate status](PROJECT_GATES.md).
For this machine, the tested PowerShell setup is:

```powershell
# Run from the repository root; adapt these installation paths on other PCs.
$env:PATH = 'C:/Users/yapsu/.pico-sdk/toolchain/15_2_Rel1/bin;C:/msys64/ucrt64/bin;C:/msys64/usr/bin;' + $env:PATH
Set-Location build_make
make SMP=1 CONSOLE=uart SHELL=C:/msys64/usr/bin/sh.exe -j8
make SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=C:/Users/yapsu/.pico-sdk/sdk/2.3.1 SHELL=C:/msys64/usr/bin/sh.exe -j8
```

The commands produce `build_make/mtk3pico_smp1_uart.uf2` and
`build_make/mtk3pico_smp1_usb_cdc.uf2`, respectively. This is the repository's
Make workflow. The locally tested SDK version is not a team version freeze.

| Tester / board | Revision / profile / versions | Build | Flash | Console evidence |
|---|---|---|---|---|
| Skyler / Pico W, ID UNKNOWN | TBD | Pending | Pending | Pending |
| Suen Hong / Pico W, ID UNKNOWN | Intended SMP USB-CDC/WIFI none; review HEAD 476c33b; local GCC 15.2.Rel1/SDK 2.3.1; flashed revision not embedded in log | Pass: user reports supplied build command completed | Pass: user-reported flash supported by running demo | COM6; [review and raw log](../../evidence/platform/STAGE_1_FIRST_BOARD_REVIEW.md); 59 healthy frames, banner/time metadata missing |
| Wynn / Pico W, ID UNKNOWN | TBD | Pending | Pending | Pending |
| Zhi Hong / Pico W, ID UNKNOWN | TBD | Pending | Pending | Pending |
| Nicholas / Pico W, ID UNKNOWN | TBD | Pending | Pending | Pending |
| Suen Hong / Robo-Pico with transferred green controller, IDs UNKNOWN | Same previously flashed SMP USB demo; exact flashed revision not embedded | Pass: reuses previously built firmware | Pass: previously flashed controller transferred to socket | User confirms working; latest screenshot COM6, 8 healthy frames seq 33–61; [review](../../evidence/platform/STAGE_1_ROBO_PICO_REVIEW.md); separate raw log pending |
| Agent / fresh local clone | a3145cb; SMP USB-CDC; GCC 15.2.Rel1; SDK 2.3.1; Make 4.4.1 | Pass: clean local clone, ELF/UF2 generated | N/A | docs/private/stage1_fresh_clone_usb_build.log; hardware N/A |

The user confirms Suen Hong performed the first bench-board test. Physical
board ID/revision and complete capture metadata remain to be recorded.

## Session 3 — Agree the Stage 2 contracts

Create a block diagram, complete resource map, task table, public headers,
units contract, and error/failure definitions before driver implementation.

The intended command flow is:

```text
Line/barcode, terrain, obstacle data/events -> Vehicle Controller
Vehicle Controller -> Motion requests -> Buddy 2 motor/encoder hardware
Subsystem snapshots/events -> Communications -> WiFi/MQTT
```

- [ ] Each request names units, limits, completion/error behaviour, and timeout.
- [ ] Define timestamps, validity, stale-data policy, and event sequence IDs.
- [ ] Motion requests complete asynchronously and can be interrupted by stop.
- [ ] Define bounded queues and queue-full behaviour; safety must not depend on
      waiting behind ordinary queued manoeuvres.
- [ ] Agree each task's priority, period/deadline, stack, affinity, and ISR interaction.
- [ ] Budget kernel objects: current configuration allows four message buffers
      and four mutexes, so do not assume one of each per subsystem fits unchanged.
- [ ] Preserve one owner per peripheral, IRQ, and shared PWM slice configuration.
- [ ] Define arbitration: safety, collision prevention, active manoeuvre, line following.
- [ ] Agree communications ownership: raw lwIP calls remain in the CYW43 service
      context; communications requests cross to that context through a bounded handoff.

Existing reservations to include in the resource map:

| Resource | Current use to account for |
|---|---|
| GP0/GP1 | UART0 console, including the default USB profile's UART mirror |
| GP16 | Demo liveness LED output |
| GP8/GP9 | Inherited I2C0 startup/driver initialization conflicts with Robo-Pico motor 1 |
| GP10/GP11 | Robo-Pico motor 2; shared with Maker Pi Pico microSD wiring on bench carrier |
| GP12–GP15 | Robo-Pico servo ports; shared with Maker Pi Pico microSD wiring on bench carrier |
| GP26/GP27/GP28 | Inherited ADC initialization; Maker Pi Pico NeoPixel uses GP28 and Robo-Pico optional battery sense uses GP28 |
| GP23/GP24/GP25/GP29 | Reserved radio wiring on Pico W |
| Hardware spinlocks 0–2 | Kernel-reserved |
| SysTick and SIO FIFO IRQs | Kernel timekeeping and SMP communication |
| USBCTRL, CYW43 PIO/DMA | Existing services own these when enabled |

This is a reservation list, not a final Robo-Pico pinout. Check schematics
before assigning pins or flashing a demo that drives GP16. Record actual
PIO state machines, DMA channels, timers, and PWM slices during Stage 2.
The current kernel tick is 10 ms; barcode edges and ultrasonic echo timing
must not be treated as measurements from a millisecond task sleep alone.

## Next — Drivers, subsystem algorithms, then integration

Stage 3 begins with `motor_test`, `encoder_test`, `ir_test`, `imu_test`,
`ultrasonic_test`, `servo_test`, and `wifi_test`. Motion owns every drive-motor
write, including its bench test. Verify supplies and the HC-SR04 logic interface
before running electrical tests.

After G3, Buddies 2–5 and Buddy 1 develop their respective Stages 4–8 against
the agreed contracts. Their algorithm work can run concurrently once their
prerequisites are met; stage numbering does not require buddies to wait for
another independent subsystem's algorithm. Bench perception and communications
work can proceed early, while moving tests depend on calibrated motion.

The coordinator validates Stage 9 controller transitions with simulated inputs
before commanding the vehicle. Follow Stage 10's integration order, involving
the controller wherever movement is requested:

1. Motion/encoders, then motion/line.
2. Barcode navigation, then IMU with motion.
3. Stationary scan, then bypass in open space.
4. Bypass plus line recovery.
5. Add WiFi load and verify control timing.

Repeat each integration step 5–10 times as the roadmap specifies. Finish the
Stage 11 mission tests, Stage 12 variations/resource checks, and Stage 13
demonstration. Gather calibration logs and test evidence throughout, using
the roadmap's `evidence/` categories and a platform category for build/flash.

First physical milestone: **all five members independently run and capture
the same RTOS demo**, with requirements and named ownership agreed.
