# Barcode reads INVALID — debug notes for Buddy 3

Written for: Buddy 3 (line + barcode). From Buddy 1, 30 Oct 2026, from the
`test_integrate` track runs.

## What we see
On the track the car fires `robot/event/barcode` with `code: INVALID`
(2–3 times per run), e.g.:
```
robot/event/barcode {"code":"INVALID","command":"NONE"}
```

## What INVALID actually means (from your `barcode.c`)
`INVALID` is **not** random. The decoder captured **≥ 6 black/white edges**
(`BARCODE_MIN_EDGES`) but they did **not** form a clean Code 39 `*A*`/`*F*`/`*Z*`
character. So the sensor *did* see a stripe pattern — it just couldn't decode it.
It gets there one of three ways:

1. **Distorted stripe widths** — decode relies on even widths (3 widest =
   wide, `C39_MIN_WIDE_PERCENT` 125%). If the car is **wandering as it crosses**,
   the widths come out uneven and the wide/narrow check fails.
2. **Not a real barcode** — a junction or thick line gives ≥6 edges but no valid
   character.
3. **Edge noise / overflow** — spurious transitions, or > 64 edges
   (`barcode_overflow`), force INVALID.

## Most likely cause here
The car still **wanders left/right while line-following** (2-sensor blind spot),
so when it crosses a barcode it isn't going **straight and steady** → the stripe
widths are garbled → INVALID. **Better line-tracking = better barcode reads.**

## What to check / adjust (your side)
1. **Is there a clean Code 39 `*A*`/`*F*`/`*Z*` barcode** where the car reads
   INVALID? Narrow element ~3 mm, wide ~9 mm, with white margins each side
   (your header: total ~141 mm). A mis-printed or non-Code-39 barcode can never
   decode.
2. **Cross it straight and steady.** If the car is correcting/wandering over the
   stripes, widths distort. Place the barcode on a straight, centred section if
   possible, and keep following speed steady across it (it's 70 mm/s now).
3. **Barcode sensor (GP28) height / aim** — too high or low blurs the edges, so
   the decoder adds or drops transitions.
4. **Confirm it isn't firing on a junction** — check *where* on the track each
   INVALID happens (compare the event time to the car's position in the log).

## How to debug it
- In the MQTT log, the **`line` telemetry `sensors` field** shows which line
  sensors are black; around the barcode, watch whether the car is `CENTRE`
  (good) or thrashing `LEFT/RIGHT/LOST` (bad crossing).
- If you can, add a temporary print of `edge_count` and the captured widths in
  `c39_decode()` so you can see whether it's getting the right number of edges
  (should be 10 for one char: `C39_EDGES`) and a sane wide/narrow split.
- Test the decoder **off the car first**: roll the car by hand straight over a
  known barcode at steady speed and confirm it reads A/F/Z. If it reads clean by
  hand but INVALID while driving, it's the **wandering**, not the decoder.

## Not blocking right now
Barcodes only matter at junctions for the A/F/Z turn choice. On a plain
line-follow test, INVALID doesn't stop anything — so this can wait until the
line-tracking is tidy, which will likely fix most of it anyway.
