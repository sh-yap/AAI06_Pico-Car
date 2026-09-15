# Project gate status

The [roadmap gate checklist](../autonomous_robot_project_roadmap_latest/PROJECT_GATES.md)
is authoritative. This page records the evidence behind the current position.

## Current position

**G0 is open.** Requirements, questions, and all five named Buddy assignments
are documented. Mission agreement by all members remains pending. The controller
coordinator and physical hardware audit also remain incomplete in Stage 0.
Local Stage 1 builds can proceed while Stage 0 questions are being resolved.
Do not treat local build success as completion of G1.

| Gate | Status | Evidence or next requirement |
|---|---|---|
| G0 — Requirements | In progress | Requirements/question register and five named Buddy owners exist; mission agreement pending |
| G1 — Platform | In progress locally | Three profiles build; clean local clone builds SMP USB-CDC; every-member build/flash/runtime checks pending |
| G2 — Architecture | Pending | Agree APIs, full GPIO/resource map, task ownership, units, and failures |
| G3 — Drivers | Pending | Independent, repeatable tests for every physical device |
| G4 — Motion | Pending | Measured speed/distance/turn accuracy and straight correction |
| G5 — Perception | Pending | Line/barcode, hump, and scan/profile evidence |
| G6 — Controller | Pending | Simulated-input state machine, arbitration, fault transitions |
| G7 — Integration | Pending | Pairwise integration and regression evidence |
| G8 — Mission | Pending | Complete autonomous mission demonstrated |
| G9 — Robustness | Pending | Repeated runs, variations/faults, timing/memory measurements |
| G10 — Demo ready | Pending | Evidence organised, rehearsal, team design explanation |

## Local build evidence — 2026-09-15

Supplied application and kernel sources were unchanged during these builds.
Base revision: `a3145cb` (`Initial Commit + MD stages`).

Environment: Windows, GNU Make 4.4.1 from MSYS2, host `g++` from UCRT64,
Arm GNU Toolchain 15.2.Rel1 (`arm-none-eabi-gcc` 15.2.1).
USB build used the local Pico SDK 2.3.1 checkout.

| Profile | Result | ELF text/data/bss bytes | Local log |
|---|---|---|---|
| `SMP=0 CONSOLE=uart` | ELF and UF2 generated | 40406 / 192 / 9684 | `docs/private/baseline_smp0_build.log` |
| `SMP=1 CONSOLE=uart` | ELF and UF2 generated | 50702 / 204 / 13776 | `docs/private/baseline_smp1_build.log` |
| `SMP=1 CONSOLE=usb_cdc` | ELF and UF2 generated | 63394 / 228 / 19900 | `docs/private/baseline_smp1_usb_build.log` |

These sizes are linker section totals, not measured runtime memory use.
The build logs contain no `warning:` or `error:` diagnostics, but the default
Make C flags do not enable `-Wall -Wextra`; a full warning audit remains pending.
The local versions differ from the lecturer's documented GCC 13.2.1 / SDK 2.2.0
baseline. Successful compilation does not qualify those versions on hardware.

No board was flashed and no sensor/motor/network operation was tested.
Generated files and private logs remain gitignored.
On Windows the host converter is `elf2uf2.exe`; its specific paths are now
ignored too.

## Stage 1 fresh-clone build evidence

A new `git clone --no-hardlinks . docs/private/stage1_fresh_clone` checkout
of committed revision `a3145cb` built `SMP=1 CONSOLE=usb_cdc` against the local
SDK 2.3.1 using the versions above. It generated ELF and UF2 with the same
63394 / 228 / 19900 text/data/bss section totals and no emitted warning/error
diagnostics. Log: `docs/private/stage1_fresh_clone_usb_build.log`.

This was an offline clone of the local committed repository, not a network
clone of GitHub. It excludes uncommitted source changes and old build artifacts;
it does not verify GitHub permissions, a new machine's setup, or hardware.
The clone and its build outputs are under ignored `docs/private/`.

Use [STAGE_1_GUIDE.md](STAGE_1_GUIDE.md) for member build/flash evidence and
the complete Stage 1 exit checks.

## Closing a checklist item

Record the tester, board, firmware revision, build profile, setup/calibration,
expected result, observed result, repetitions, and evidence location.
Use a checked box only when that item's scope is satisfied. For example,
"everyone can flash" needs evidence from all five members.
