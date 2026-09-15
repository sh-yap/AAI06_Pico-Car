# Stage 1 — Common Development Environment

Local build evidence: [docs/project/PROJECT_GATES.md](../project/PROJECT_GATES.md).
Step-by-step Windows flow: [Stage 1 guide](../project/STAGE_1_GUIDE.md).
Checked local items below apply only to this Windows workspace. Every-member,
flashing, and hardware checks remain pending; the fresh local clone build passes.

## Goal
Every team member can build and flash the same codebase.

## Toolchain
- [ ] Install compiler/toolchain
- [ ] Configure Pico C SDK
- [ ] Obtain supplied GitHub template
- [x] Confirm micro T-Kernel RTOS integration — supplied demo compiles/links locally; runtime pending
- [x] Build unmodified project — single-core UART, SMP UART, SMP USB-CDC; sources unchanged
- [ ] Flash one Maker Pi Pico
- [ ] Verify USB serial/debug output
- [ ] Repeat on all five Maker Pi Pico boards
- [ ] Build and flash a basic Robo-Pico test

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
