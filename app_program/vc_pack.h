/** @file vc_pack.h
 *
 * @brief Packed stored copies of the Vehicle Controller's own state (CR1).
 *
 * Only what the VC task keeps in static RAM between two 20 ms steps is
 * packed: see docs/DATA_REPRESENTATION.md sections 5.1, 11 rows 11, 15
 * and 16.  The value types vc_t and obstacle_bypass_plan_t stay full width
 * for the pure logic in vc.c (host-tested through vc_t *).  The VC task
 * unpacks the stored record into a stack working copy, runs vc_step() and
 * the bypass sequence on it, and packs it back.  Only the VC task touches
 * either record, so no lock is needed.
 *
 * Not packed: vc_inputs_t and vc_outputs_t are built on the VC task's
 * stack every step and never stored, so packing them would save no static
 * RAM and is not done.
 *
 * Rules (docs/DATA_REPRESENTATION.md section 0): unsigned words, explicit
 * masks and shifts, no C bit-fields, signed fields two's complement and
 * sign-extended, a field never straddles a word boundary, and an enum code
 * that is not a value the code produces is written as all ones and read
 * back as a fault rather than guessed.  Code 0 is a real state for
 * vc_state_t, so it is not used as the no-data code here (unlike the other
 * packed records in this tree).
 *
 * Controller state record: VC_STATE_PACKED_WORDS = 10 words (40 B).
 *
 * Widths are the reachable range from code.  Two differ from the first
 * proposal in section 5.1, both because the value is read back in a later
 * step and must never be clamped (rule: never clamp a reachable value):
 * search_origin_mdeg is 32 s, not 18 s (see W2), and search_limit_mdeg is
 * 17 U, not 18 U (its only values are VC_SEARCH_LIMIT_MDEG 90 000 and
 * VC_SEARCH_LIMIT_POST_BYPASS_MDEG 45 000).
 *
 *   W0      bits  0..3   state             4 U, vc_state_t 0..10
 *           bits  4..6   stored_nav        3 U, vc_nav_t 0..4
 *           bits  7..8   rescans           2 U, 0..VC_MAX_RESCANS
 *           bit   9      b_obstacle_active
 *           bit   10     b_reversing
 *           bits 11..13  prev_line         3 U, vc_line_t 0..4
 *           bits 14..15  last_side         2 s, -1..+1
 *           bits 16..17  search            2 U, vc_search_t 0..2
 *           bits 18..19  search_dir        2 s, -1..+1
 *           bit   20     b_after_bypass
 *           bits 21..23  cal_phase         3 U, vc_cal_phase_t 0..5
 *           bit   24     b_junction_hold
 *           bit   25     b_junction_track
 *           bit   26     b_junction_seen
 *           bits 27..29  pending_turn      3 U, vc_nav_t 0..4
 *           bit   30     b_right_exit
 *           bit   31     reserved, 0
 *   W1      bits  0..31  entry_ms          32 U, milliseconds, wraps
 *   W2      bits  0..31  search_origin_mdeg 32 s, two's complement
 *                        This is an absolute encoder angle, not a bounded
 *                        offset: vc_task.c computes it as
 *                        (left-right counts since the sweep origin) x
 *                        34 mdeg, and the counts are themselves stored at
 *                        24 s (docs section 1.2), so it reaches about
 *                        +-5.7e8 mdeg.  It is read back in
 *                        step_search_pivot() as
 *                        p_in->cal_angle_mdeg - p_vc->search_origin_mdeg,
 *                        so a narrower field would clamp a reachable value
 *                        and change where the second pivot goes.
 *   W3      bits  0..31  bypass_end_ms     32 U, milliseconds, wraps
 *   W4      bits  0..31  cal_phase_ms      32 U, milliseconds, wraps
 *   W5      bits  0..16  search_limit_mdeg 17 U, 45 000 or 90 000
 *           bits 17..18  steer_side        2 s, -1..+1
 *           bits 19..25  steer_mag         7 U, 0..VC_SEARCH_STEER (127)
 *           bits 26..30  lift_spell_cycles 5 U, 0..VC_LIFT_SPELL_CAP_CYCLES
 *           bit   31     b_lift_spell_cued
 *   W6      bits  0..17  cal_out_mdeg      18 U, 0..262 143
 *           bits 18..19  cal_first_dir     2 s, -1..+1
 *           bits 20..21  cal_dir           2 s, -1..+1
 *           bits 22..31  reserved, 0
 *   W7      bits  0..17  cal_far_mdeg      18 U, 0..262 143
 *                        Both calibration angles are measured from the
 *                        sweep origin while the pivot is bounded by
 *                        VC_CAL_LIMIT_MDEG (45 000) plus the way to the
 *                        other side and the return margin, so 18 U holds
 *                        every value the code can produce.
 *           bits 18..22  lift_spell_max_cycles 5 U, 0..CAP (25)
 *           bits 23..31  reserved, 0
 *   W8      bits  0..23  junction_origin_mm 24 s, travel_mm absolute
 *           bits 24..31  reserved, 0
 *   W9      bits  0..23  junction_seen_mm  24 s, travel_mm absolute
 *                        travel_mm is TBC in section 5.1, so it keeps the
 *                        agreed 24 s.  Both are read back as a difference
 *                        against the next travel_mm reading, so they must
 *                        hold the absolute value without clamping.
 *
 * Bypass plan record: VC_PLAN_PACKED_WORDS = 3 words (12 B).  This is the
 * VC's copy of obstacle_types.h:obstacle_bypass_plan_t (g_plan); the
 * planner in obstacle_plan.c still returns the full-width value type.
 * Widths come from obstacle_plan.c: turn_deg is PLAN_MIN_DEG (5) ..
 * OBSTACLE_PLAN_MAX_DEG (40); side_mm is half_obstacle (100) + margin (30)
 * + half_car (70) = 200; out_mm is (side_mm * 10000 + sin - 1) / sin, at
 * most 2294 mm at 5 degrees; back_deg is 2 x turn_deg, capped at
 * OBSTACLE_PLAN_BACK_MAX_DEG (60), so back_mm is at most 1152 mm;
 * check_mm is out_mm + OBSTACLE_PLAN_CHECK_EXTRA_MM (100), at most
 * 2394 mm; along_mm is x_end - x_out, where x_end is the reading (at most
 * 4095 mm, the packed status end) + axle-to-sensor 65 + obstacle 200 +
 * margin 30 + rear-corner swing 100 + extra 30 = 4520 mm, so 13 U.
 *
 *   W0      bits  0..11  out_mm          12 U, 0..4095
 *           bits 12..23  back_mm         12 U, 0..4095
 *           bits 24..29  turn_deg        6 U, 5..40
 *           bits 30..31  reserved, 0
 *   W1      bits  0..12  along_mm        13 U, 0..8191
 *           bits 13..24  check_mm        12 U, 0..4095
 *           bits 25..27  side            3 U, obstacle_action_t: only
 *                                        TURN_LEFT (2) and TURN_RIGHT (3)
 *                                        are values the planner produces
 *           bits 28..31  reserved, 0
 *   W2      bits  0..7   side_mm         8 U, 0..255 (always 200)
 *           bits  8..13  back_deg        6 U, 10..60
 *           bits 14..31  reserved, 0
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h>, "vc.h"
 * and "obstacle_types.h" before this header.
 */

#ifndef VC_PACK_H
#define VC_PACK_H

/* Words in the stored controller state record. */
#define VC_STATE_PACKED_WORDS  (10U)

/* Words in the stored bypass plan record. */
#define VC_PLAN_PACKED_WORDS   (3U)

/* The stored controller state: the VC task's static copy of vc_t. */
typedef struct
{
    uint32_t word[VC_STATE_PACKED_WORDS];
} vc_packed_t;

/* The stored bypass plan: the VC task's static copy of
 * obstacle_bypass_plan_t. */
typedef struct
{
    uint32_t word[VC_PLAN_PACKED_WORDS];
} vc_plan_packed_t;

/* Copy the controller state into its stored form.  Values outside a field
 * saturate at the field's end; an enum code that is not a value of its enum
 * is stored as all ones, which vc_unpack_state() rejects. */
void
vc_pack_state (vc_t const * p_vc, vc_packed_t * p_packed);

/* Decode the stored controller state.  false if an enum code, a flag bit or
 * a reserved bit is not a value the code produces; p_vc is left untouched
 * then, so a damaged record is never run as a state. */
bool
vc_unpack_state (vc_packed_t const * p_packed, vc_t * p_vc);

/* Copy a bypass plan into its stored form (same saturation rule). */
void
vc_pack_plan (obstacle_bypass_plan_t const * p_plan,
              vc_plan_packed_t *             p_packed);

/* Decode a stored bypass plan.  false if the side is not a turn or a
 * reserved bit is set; p_plan is left untouched then. */
bool
vc_unpack_plan (vc_plan_packed_t const * p_packed,
                obstacle_bypass_plan_t * p_plan);

#endif /* VC_PACK_H */

/*** end of file ***/
