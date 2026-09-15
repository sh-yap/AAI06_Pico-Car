# Stage 11 — Complete Autonomous Mission

## Test A — Line only
```text
START → LINE → FINISH
```
- [ ] Pass

## Test B — Barcode
```text
LINE → A → LEFT → LINE
```
Then:
- [ ] B / RIGHT
- [ ] C / STRAIGHT
- [ ] D / UTURN

## Test C — Hump
```text
LINE → HUMP → LINE
```
- [ ] Detect hump
- [ ] Record peak
- [ ] Continue correctly

## Test D — Obstacle
```text
LINE → OBSTACLE → BYPASS → REACQUIRE → LINE
```
- [ ] Pass

## Test E — Combined mission
```text
LINE
→ barcode
→ hump
→ obstacle
→ reacquire
→ another barcode
→ finish
```

- [ ] Pass before attempting full course at higher speed
