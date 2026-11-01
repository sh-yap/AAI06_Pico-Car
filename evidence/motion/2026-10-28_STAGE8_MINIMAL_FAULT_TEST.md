# Stage 8 minimal stop and fault verification - 2026-10-28

## Software evidence

The Buddy 2 host suites pass and cover command validation, encoder count and
invalid-transition faults, motor-brake write failure, latched-fault
acknowledgement/recovery, stop handling and sequence-relative commands. The
Vehicle Controller suite passes 150 checks, including serious-fault priority,
emergency-stop latching and recovery. A previous physical timeout braked both
motors and latched `MOTION_FAULT_TIMEOUT`. Five previous physical stop checks
measured 204 us to 10.690 ms response.

Disconnecting encoders, jamming wheels and inducing brownouts are not repeated
for this minimal progression because they add hardware risk and the applicable
fatal paths already brake both motors in deterministic tests. Formal voltage
and acceptance-limit matrices remain Stage 0/final-integration work.

## One remaining physical check

Use `motion_stage7_test`, select the U-turn (three selection beeps), start with
GP20, and press GP21 while the car is turning. Pass only if it stops
immediately, gives result code 2 and stays stopped.

| Test | Observed stop | Result code | Restarted by itself | Decision |
| --- | --- | ---: | --- | --- |
| GP21 during U-turn | Stopped after approximately half a turn | 2 beeps | No | Pass |

The minimum Stage 8 physical stop check is complete. Together with the host
fault tests and earlier stop-latency evidence, this clears the reduced
progression scope through Stage 8. Formal battery, brownout, disconnected-
encoder and numerical acceptance-limit matrices remain deferred to final
integration because they were intentionally excluded as unnecessary or unsafe
for this catch-up pass.
