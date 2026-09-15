# Stage 1 — Common Development Environment

## Goal
Every team member can build and flash the same codebase.

## Toolchain
- [ ] Install compiler/toolchain
- [ ] Configure Pico C SDK
- [ ] Obtain supplied GitHub template
- [ ] Confirm micro T-Kernel RTOS integration
- [ ] Build unmodified project
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
- [ ] Fresh clone builds successfully
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
