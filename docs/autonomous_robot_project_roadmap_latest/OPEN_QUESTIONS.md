# Open Questions

Brief review: [requirements baseline](../project/REQUIREMENTS.md).
The checkboxes below track final clarification, including lecturer responses;
they do not mean a subject is absent from the briefs.

## Findings from the supplied briefs

| Topic | Evidence | Remaining clarification |
|---|---|---|
| A/B/C/D | Write-up p. 3 provides the mapping as an example | Freeze final symbols and encoding |
| A/Z samples | Mentioned in roadmap; separate samples not found in workspace inventory | Obtain/identify the official sample patterns |
| U-turn | Write-up pp. 2–3 explicitly include it; Buddy 3 p. 7 list omits it | Retain mission scope; confirm assessment details |
| Humps | Write-up p. 3 asks for height estimation using IMU data | Method, permitted auxiliary inputs, units, and accuracy |
| Turn rate | Briefing p. 7 and write-up p. 8 request it | Accepted estimation method with the supplied no-gyro IMU |
| MQTT | Briefing p. 4 and write-up p. 5 require it | Broker, credentials, topics, QoS, and command semantics |
| Platform | Write-up p. 1 requires Pico/Robo-Pico, RTOS, C, SDK, template, Barr-C | Exact template and required final SMP/profile |
| Demonstration | Briefing p. 10 mentions Week 10 | Calendar date and detailed assessment criteria |
| Motor/encoder interface | `PicoCar.pptx` slide 1 documents a PH2.0-6P connector: encoder GND, A phase, B phase, encoder power, motor+, motor- | Motor/encoder model, motor voltage/current/gear ratio, encoder supply/output levels, pull-ups, PPR/CPR, and shaft reference |

## Clarification register

- [ ] Are final barcode commands definitely A/B/C/D?
- [ ] Why are supplied samples A and Z rather than A/B/C/D?
- [ ] What barcode symbology / width encoding should be used?
- [ ] Is U-turn definitely assessed?
- [ ] What hump-height accuracy/method is expected?
- [ ] How should turn-rate monitoring be implemented with LSM303DLHC?
- [ ] Exact motor model, rated voltage/current, stall current, and gear ratio? The PH2.0-6P motor pins are now documented.
- [ ] Encoder supply voltage, output type/logic level, pull-up requirement, pulses/revolution, x1/x2/x4 convention, and motor-shaft versus wheel-shaft reference? A/B phase pins are now documented.
- [ ] Wheel diameter?
- [ ] Wheel separation?
- [ ] Servo model and pulse/angle range?
- [ ] Battery specification?
- [ ] MQTT broker/settings?
- [ ] Exact required GitHub template?
- [ ] Any missing course dimensions?
- [ ] Which Pico/Pico W and WiFi device is fitted on each development/final board?
- [ ] Is SMP required in the final assessed build, and which compiler/SDK versions should the team freeze?
- [ ] What are start/finish conditions and numerical acceptance criteria?
- [ ] What calendar date is the Week 10 demonstration?

For each answer, record the source/respondent, date, agreed decision, affected
requirement/API, and evidence location. Hardware specifications can be resolved
by inspection/measurement; assessment ambiguities require lecturer clarification.
