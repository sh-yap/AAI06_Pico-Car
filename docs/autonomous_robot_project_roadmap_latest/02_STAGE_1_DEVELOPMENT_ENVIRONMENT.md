# Stage 1 — Common Development Environment

Local build evidence: [docs/project/PROJECT_GATES.md](../project/PROJECT_GATES.md).
Step-by-step Windows flow: [Stage 1 guide](../project/STAGE_1_GUIDE.md).
Checked local items below apply to this Windows workspace and the first
user-tested bench board. The fresh local clone build, one-board flash/USB
runtime check, and basic Robo-Pico runtime check pass; every-member checks
remain pending.

## Goal
Every team member can build and flash the same codebase.

## Toolchain
- [ ] Install compiler/toolchain
- [ ] Configure Pico C SDK
- [ ] Obtain supplied GitHub template
- [x] Confirm micro T-Kernel RTOS integration — local builds plus first bench-board SMP producer/consumer runtime evidence
- [x] Build unmodified project — single-core UART, SMP UART, SMP USB-CDC; sources unchanged
- [x] Flash one Maker Pi Pico — user-reported Pico W bench flash supported by runtime log
- [x] Verify USB serial/debug output — COM6 screenshot and 59 healthy monitor frames; evidence/platform/STAGE_1_FIRST_BOARD_REVIEW.md
- [ ] Repeat on all five Maker Pi Pico boards
- [x] Build and flash a basic Robo-Pico test — previously flashed controller transferred to Robo-Pico; user confirms working output, screenshot sequence 33–61 with zero drops; evidence/platform/STAGE_1_ROBO_PICO_REVIEW.md

## Suggested Git branches
```text
main
develop
feature/motion
feature/line-barcode
feature/imu
feature/obstacle
feature/comms
```

## Git checklist
- [x] Fresh clone builds successfully — clean local committed clone a3145cb builds SMP USB-CDC; GitHub/new-PC checks pending
- [ ] `main` always builds
- [ ] `.gitignore` is correct
- [ ] Everyone can pull/commit/push/merge
- [ ] No ZIP-file code sharing

## Coding baseline
- [ ] Use fixed-width integers where width matters
- [ ] Initialise variables before use
- [ ] Use braces consistently
- [ ] Minimise globals
- [ ] Keep module state private
- [ ] Expose public APIs through headers
- [ ] Avoid long blocking delays
- [ ] Prefer static/bounded memory
- [ ] Avoid unnecessary dynamic allocation

## Exit condition
```text
clone → configure → compile → flash → observe output
```
- [ ] Every member can complete the full flow independently
