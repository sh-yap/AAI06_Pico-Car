# Stage 1 first bench-board runtime evidence

Result: the user-reported build/flash and supplied USB console evidence
demonstrate the supplied SMP producer/consumer demo operating on one bench
board. This is a platform smoke test, not a full kernel qualification or
verification of vehicle subsystems.

## Sources and setup

- Raw capture: [Suen Hong-usb-console.txt](Suen%20Hong-usb-console.txt).
- Two screenshots supplied in the conversation: serial monitor on COM6 and
  PowerShell capture output. Screenshots are not currently archived locally.
- User reports running the supplied build and capture commands on their board.
- Intended profile: `SMP=1 CONSOLE=usb_cdc WIFI=none`; file
  `build_make/mtk3pico_smp1_usb_cdc.uf2`.
- Bench controller: Pico W, based on the user's previous board information.
- Physical board ID/carrier revision: UNKNOWN.
- Tester: Suen Hong, explicitly confirmed by the user.
- Workspace HEAD at review: `476c33ba7b6f54fddba657f9c55914636848df41`.
  Its application/kernel/build sources match `a3145cb`. The console does not
  embed a firmware revision/hash, so the exact flashed revision is not proven
  by the raw capture alone.
- Build command uses the local GCC 15.2.Rel1 / SDK 2.3.1 / Make setup recorded
  in the Stage 1 guide. No new tool-version transcript was supplied by the tester.

## Raw-log findings

- 59 monitor frames, sequence 140 through 368.
- Sequence strictly increases between every consecutive frame.
- Every frame has equal sequence, produced, and consumed counts.
- Every frame reports `dropped=0`, producer `prc1`, and consumer `prc2`.
- No stalled, initialization-failed, or console-dropped message appears in the log.
- Banner is absent: capture begins after startup.
- No timestamps or capture-start/end markers: exact elapsed time and whether
  this contains multiple appended captures are not established.

The serial-monitor screenshot independently shows COM6 and progressing
sequence 73 through 124, zero drops, and the same processor IDs.

## Checklist consequences

The Stage 1 one-board flash and USB output checks can be checked based on
the user's reported test and captured runtime evidence. The other four
member results, physical board metadata, banner/capture-duration record, Git
workflow checks, coding-baseline agreement, and template confirmation remain
pending. The later [Robo-Pico basic runtime test](STAGE_1_ROBO_PICO_REVIEW.md)
passes on user confirmation and screenshot evidence. G1 remains open until every member can build
and flash and the required shared platform checks are satisfied.
