# Stage 9 — Vehicle Controller

## Goal
Create the central mission-level coordinator.

## Suggested state machine
```text
INIT
  ↓
READY
  ↓
LINE_FOLLOW
  ├─ barcode  → EXECUTE_NAV_COMMAND → LINE_FOLLOW
  ├─ obstacle → OBSTACLE_STOP
  │              ↓
  │           OBSTACLE_SCAN
  │              ↓
  │           BYPASS
  │              ↓
  │           LINE_RECOVERY
  │              ↓
  │           LINE_FOLLOW
  └─ fatal fault → SAFE_STOP
```

## Tasks
- [ ] Define mission states
- [ ] Define valid transitions
- [ ] Define state-entry actions
- [ ] Define state-exit criteria
- [ ] Define timeouts
- [ ] Define fault handling
- [ ] Define motor-command ownership
- [ ] Define event arbitration

## Recommended priority
```text
fatal safety condition
>
obstacle collision prevention
>
active manoeuvre
>
normal line following
```

## Exit condition
- [ ] Controller works with fake/simulated subsystem inputs
