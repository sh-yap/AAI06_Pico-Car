# Stage 5 — Buddy 3: IR Line Following and Barcode Decoding

## Goal
Create robust line tracking and barcode command generation.

## 5A — IR calibration
For each sensor:
- [ ] Measure white reading
- [ ] Measure black reading
- [ ] Test several sensor heights
- [ ] Test several lighting conditions
- [ ] Record threshold/margin
- [ ] Record sensor-to-sensor variation

## 5B — Line position estimation
Possible output:
```text
LINE_LEFT
LINE_CENTRE
LINE_RIGHT
LINE_LOST
JUNCTION
```

- [ ] Prefer continuous line error if analogue readings support it

## 5C — Basic line following
Progress in order:
```text
stationary detection
→ slow straight line
→ gentle curves
→ normal line
→ line-loss recovery
```

Test:
- [ ] minor lighting changes
- [ ] slight track imperfections
- [ ] speed variation

## 5D — Junction detection
- [ ] Define junction condition
- [ ] Debounce detection
- [ ] Prevent repeated events
- [ ] Test multiple approach speeds

## 5E — Barcode acquisition
Record:
```text
black/white transitions
transition timestamps
encoder position
bar widths
```

- [ ] Move sensor manually across sample first
- [ ] Measure edge-to-edge widths
- [ ] Normalise widths
- [ ] Repeat at multiple speeds

## 5F — Barcode decoding
Required mission commands:
```text
A → LEFT
B → RIGHT
C → STRAIGHT
D → UTURN
```

- [ ] Keep U-turn in software design
- [ ] Do not invent encoding until clarified

## 5G — Navigation command interface
Return:
```text
NAV_NONE
NAV_LEFT
NAV_RIGHT
NAV_STRAIGHT
NAV_UTURN
```

- [ ] Line/barcode module does not directly own motors

## Deliverables
- [ ] Line-following module
- [ ] Barcode decoder
- [ ] Navigation command interface

## Definition of done
- [ ] Line tracking is repeatable
- [ ] Barcode recognition is tested at different speeds
- [ ] Duplicate barcode events are prevented
