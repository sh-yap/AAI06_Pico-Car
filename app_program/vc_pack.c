/** @file vc_pack.c
 *
 * @brief Packed stored copies of the Vehicle Controller's own state (CR1).
 *
 * The bit layouts are documented in vc_pack.h.  Only the VC task's stored
 * copies are packed; vc.c's logic and its callers keep the full-width value
 * types vc_t and obstacle_bypass_plan_t.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "vc.h"
#include "obstacle_types.h"
#include "vc_pack.h"

/* The record layouts below are fixed to these field widths. */
#if (VC_MAX_RESCANS > 3U)
#error "Packed vc_t assumes rescans fits 2 bits"
#endif

#if (VC_LIFT_SPELL_CAP_CYCLES > 31U)
#error "Packed vc_t assumes the lift spell counter fits 5 bits"
#endif

/* Value counts of the vc.h enums: a code that is not below its count is not
 * a value the code produces (vc.h has no ..._COUNT for most of them). */
#define ST_NAV_CODES        (5U)   /* NONE, LEFT, RIGHT, STRAIGHT, UTURN */
#define ST_LINE_CODES       (5U)   /* LEFT .. JUNCTION                  */
#define ST_SEARCH_CODES     (3U)   /* NONE, FIRST, SECOND               */
#define ST_CAL_PHASE_CODES  (6U)   /* STILL .. FINISH                   */
#define ST_STATE_CODES      (11U)  /* vc_state_t; VC_STATE_COUNT        */

/* Controller state record words. */
#define ST_WORD_FLAGS       (0U)
#define ST_WORD_ENTRY_MS    (1U)
#define ST_WORD_SEARCH_ORIG (2U)
#define ST_WORD_BYPASS_END  (3U)
#define ST_WORD_CAL_PHASE   (4U)
#define ST_WORD_STEER       (5U)
#define ST_WORD_CAL_ANGLES  (6U)
#define ST_WORD_LIFT        (7U)
#define ST_WORD_JUNC_ORIGIN (8U)
#define ST_WORD_JUNC_SEEN   (9U)

/* W0: codes, counters and flags. */
#define ST_STATE_SHIFT      (0U)
#define ST_STATE_BITS       (4U)
#define ST_NAV_SHIFT        (4U)
#define ST_NAV_BITS         (3U)
#define ST_RESCANS_SHIFT    (7U)
#define ST_RESCANS_BITS     (2U)
#define ST_FLAG_OBSTACLE    (9U)
#define ST_FLAG_REVERSING   (10U)
#define ST_PREV_LINE_SHIFT  (11U)
#define ST_PREV_LINE_BITS   (3U)
#define ST_LAST_SIDE_SHIFT  (14U)
#define ST_LAST_SIDE_BITS   (2U)
#define ST_SEARCH_SHIFT     (16U)
#define ST_SEARCH_BITS      (2U)
#define ST_SEARCH_DIR_SHIFT (18U)
#define ST_SEARCH_DIR_BITS  (2U)
#define ST_FLAG_AFTER_BYP  (20U)
#define ST_CAL_PHASE_SHIFT  (21U)
#define ST_CAL_PHASE_BITS   (3U)
#define ST_FLAG_JUNC_HOLD   (24U)
#define ST_FLAG_JUNC_TRACK  (25U)
#define ST_FLAG_JUNC_SEEN   (26U)
#define ST_TURN_SHIFT       (27U)
#define ST_TURN_BITS        (3U)
#define ST_FLAG_RIGHT_EXIT  (30U)
#define ST_FLAGS_RESERVED   (0x80000000U)  /* W0 bit 31.            */

/* W5: the steer ramp, the search limit and the lift spell. */
#define ST_SEARCH_LIMIT_SHIFT  (0U)
#define ST_SEARCH_LIMIT_BITS   (17U)
#define ST_STEER_SIDE_SHIFT    (17U)
#define ST_STEER_SIDE_BITS     (2U)
#define ST_STEER_MAG_SHIFT     (19U)
#define ST_STEER_MAG_BITS      (7U)
#define ST_LIFT_CYCLES_SHIFT   (26U)
#define ST_LIFT_CYCLES_BITS    (5U)
#define ST_FLAG_LIFT_CUED      (31U)

/* W6: sweep angle and direction. */
#define ST_CAL_OUT_SHIFT       (0U)
#define ST_CAL_OUT_BITS        (18U)
#define ST_CAL_FIRST_SHIFT     (18U)
#define ST_CAL_FIRST_BITS      (2U)
#define ST_CAL_DIR_SHIFT       (20U)
#define ST_CAL_DIR_BITS        (2U)
#define ST_CAL_ANGLES_RESERVED (0xFFC00000U)  /* W6 bits 22..31.      */

/* W7: furthest sweep angle and the longest normal both-black spell. */
#define ST_CAL_FAR_SHIFT       (0U)
#define ST_CAL_FAR_BITS        (18U)
#define ST_LIFT_MAX_SHIFT      (18U)
#define ST_LIFT_MAX_BITS       (5U)
#define ST_LIFT_RESERVED       (0xFF800000U)  /* W7 bits 23..31.      */

/* W8, W9: absolute travel origins. */
#define ST_JUNCTION_SHIFT      (0U)
#define ST_JUNCTION_BITS       (24U)
#define ST_JUNCTION_RESERVED   (0xFF000000U)  /* W8/W9 bits 24..31.   */

/* Field ends of the signed fields. */
#define ST_SIDE_MIN       (-1)
#define ST_SIDE_MAX       (1)
#define ST_CAL_DIR_MIN    (-1)
#define ST_CAL_DIR_MAX    (1)
#define ST_JUNCTION_MIN   (-8388608)   /* 24 s, two's complement.       */
#define ST_JUNCTION_MAX   (8388607)

/* Bypass plan record words. */
#define PLAN_WORD_LEGS     (0U)
#define PLAN_WORD_ALONG    (1U)
#define PLAN_WORD_GEOMETRY (2U)

#define PLAN_OUT_SHIFT     (0U)
#define PLAN_OUT_BITS      (12U)
#define PLAN_BACK_MM_SHIFT (12U)
#define PLAN_BACK_MM_BITS  (12U)
#define PLAN_TURN_SHIFT    (24U)
#define PLAN_TURN_BITS     (6U)
#define PLAN_LEGS_RESERVED (0xC0000000U)   /* W0 bits 30..31.       */

#define PLAN_ALONG_SHIFT   (0U)
#define PLAN_ALONG_BITS    (13U)
#define PLAN_CHECK_SHIFT   (13U)
#define PLAN_CHECK_BITS    (12U)
#define PLAN_SIDE_SHIFT    (25U)
#define PLAN_SIDE_BITS     (3U)
#define PLAN_ALONG_RESERVED (0xF0000000U)  /* W1 bits 28..31.       */

#define PLAN_SIDE_MM_SHIFT (0U)
#define PLAN_SIDE_MM_BITS  (8U)
#define PLAN_BACK_DEG_SHIFT (8U)
#define PLAN_BACK_DEG_BITS (6U)
#define PLAN_GEOM_RESERVED (0xFFFFC000U)   /* W2 bits 14..31.       */


/*!
 * @brief Mask for a field of 1..31 bits.
 *
 * @param[in] bits Field width.
 *
 * @return The mask, bit 0 = LSB.
 */
static uint32_t
pack_mask (uint32_t bits)
{
    return ((32U <= bits) ? 0xFFFFFFFFU : ((1U << bits) - 1U));
}

/*!
 * @brief A value placed in its field, saturated at the field's end.
 *
 * @param[in] value Field value.
 * @param[in] bits  Field width.
 *
 * @return value, or the field's largest value if value is too big.
 */
static uint32_t
pack_field (uint32_t value, uint32_t bits)
{
    uint32_t mask = pack_mask(bits);

    return ((value > mask) ? mask : value);
}

/*!
 * @brief A signed value in its field, saturated and two's complement.
 *
 * @param[in] value   Field value.
 * @param[in] minimum Smallest value the field may hold.
 * @param[in] maximum Largest value the field may hold.
 * @param[in] bits    Field width.
 *
 * @return The field bits.
 */
static uint32_t
pack_signed (int32_t value, int32_t minimum, int32_t maximum, uint32_t bits)
{
    int32_t clamped = value;

    if (clamped < minimum)
    {
        clamped = minimum;
    }
    else if (clamped > maximum)
    {
        clamped = maximum;
    }
    else
    {
        /* Already inside the field. */
    }

    return ((uint32_t) clamped & pack_mask(bits));
}

/*!
 * @brief An enum placed in its field: all ones if it is not a code of it.
 *
 * The all-ones code is beyond every enum's count in this record, so
 * vc_unpack_state()/vc_unpack_plan() reject the record instead of running
 * a code the packer never wrote (code 0 is a real state for vc_state_t, so
 * it is not used as the no-data code here).
 *
 * @param[in] value Enum value.
 * @param[in] count Number of codes in the enum.
 * @param[in] bits  Field width.
 *
 * @return value, or the field's largest value if value is not a code.
 */
static uint32_t
pack_enum (uint32_t value, uint32_t count, uint32_t bits)
{
    return ((value < count) ? value : pack_mask(bits));
}

/*!
 * @brief One flag bit.
 *
 * @param[in] b_value The flag.
 *
 * @return 1 when set, 0 when clear.
 */
static uint32_t
pack_flag (bool b_value)
{
    return ((true == b_value) ? 1U : 0U);
}

/*!
 * @brief Read a field back out of a word.
 *
 * @param[in] word  Packed word.
 * @param[in] shift Position of the field's LSB.
 * @param[in] bits  Field width.
 *
 * @return The field, unsigned.
 */
static uint32_t
pack_get (uint32_t word, uint32_t shift, uint32_t bits)
{
    return ((word >> shift) & pack_mask(bits));
}

/*!
 * @brief Read a signed field back out of a word, sign-extended.
 *
 * @param[in] word  Packed word.
 * @param[in] shift Position of the field's LSB.
 * @param[in] bits  Field width, 2..31.
 *
 * @return The field, sign-extended to int32_t.
 */
static int32_t
pack_signed_get (uint32_t word, uint32_t shift, uint32_t bits)
{
    uint32_t mask = pack_mask(bits);
    uint32_t value = (word >> shift) & mask;
    uint32_t sign = (1U << (bits - 1U));

    if (0U != (value & sign))
    {
        return (-(int32_t)((~value & mask) + 1U));
    }

    return ((int32_t) value);
}

/*!
 * @brief A flag read back from its bit.
 *
 * @param[in] word  Packed word.
 * @param[in] bit   Position of the flag.
 *
 * @return true when the bit is set.
 */
static bool
pack_flag_get (uint32_t word, uint32_t bit)
{
    return (0U != pack_get(word, bit, 1U));
}

/*!
 * @brief Copy the controller state into its stored form.
 *
 * @param[in]  p_vc     Controller state (or NULL).
 * @param[out] p_packed Stored record (or NULL).
 */
void
vc_pack_state (vc_t const * p_vc, vc_packed_t * p_packed)
{
    if ((NULL != p_vc) && (NULL != p_packed))
    {
        p_packed->word[ST_WORD_FLAGS] =
            (pack_enum((uint32_t) p_vc->state, ST_STATE_CODES, ST_STATE_BITS)
             << ST_STATE_SHIFT)
            | (pack_enum((uint32_t) p_vc->stored_nav, ST_NAV_CODES,
                         ST_NAV_BITS)
               << ST_NAV_SHIFT)
            | (pack_field((uint32_t) p_vc->rescans, ST_RESCANS_BITS)
               << ST_RESCANS_SHIFT)
            | (pack_flag(p_vc->b_obstacle_active) << ST_FLAG_OBSTACLE)
            | (pack_flag(p_vc->b_reversing) << ST_FLAG_REVERSING)
            | (pack_enum((uint32_t) p_vc->prev_line, ST_LINE_CODES,
                         ST_PREV_LINE_BITS)
               << ST_PREV_LINE_SHIFT)
            | (pack_signed((int32_t) p_vc->last_side, ST_SIDE_MIN,
                           ST_SIDE_MAX, ST_LAST_SIDE_BITS)
               << ST_LAST_SIDE_SHIFT)
            | (pack_enum((uint32_t) p_vc->search, ST_SEARCH_CODES,
                         ST_SEARCH_BITS)
               << ST_SEARCH_SHIFT)
            | (pack_signed(p_vc->search_dir, ST_SIDE_MIN, ST_SIDE_MAX,
                           ST_SEARCH_DIR_BITS)
               << ST_SEARCH_DIR_SHIFT)
            | (pack_flag(p_vc->b_after_bypass) << ST_FLAG_AFTER_BYP)
            | (pack_enum((uint32_t) p_vc->cal_phase, ST_CAL_PHASE_CODES,
                         ST_CAL_PHASE_BITS)
               << ST_CAL_PHASE_SHIFT)
            | (pack_flag(p_vc->b_junction_hold) << ST_FLAG_JUNC_HOLD)
            | (pack_flag(p_vc->b_junction_track) << ST_FLAG_JUNC_TRACK)
            | (pack_flag(p_vc->b_junction_seen) << ST_FLAG_JUNC_SEEN)
            | (pack_enum((uint32_t) p_vc->pending_turn, ST_NAV_CODES,
                         ST_TURN_BITS)
               << ST_TURN_SHIFT)
            | (pack_flag(p_vc->b_right_exit) << ST_FLAG_RIGHT_EXIT);

        p_packed->word[ST_WORD_ENTRY_MS]    = p_vc->entry_ms;
        /* The whole word is the int32_t's two's complement. */
        p_packed->word[ST_WORD_SEARCH_ORIG] =
            (uint32_t) p_vc->search_origin_mdeg;
        p_packed->word[ST_WORD_BYPASS_END]  = p_vc->bypass_end_ms;
        p_packed->word[ST_WORD_CAL_PHASE]   = p_vc->cal_phase_ms;

        p_packed->word[ST_WORD_STEER] =
            (pack_field((uint32_t) p_vc->search_limit_mdeg,
                        ST_SEARCH_LIMIT_BITS)
             << ST_SEARCH_LIMIT_SHIFT)
            | (pack_signed((int32_t) p_vc->steer_side, ST_SIDE_MIN,
                           ST_SIDE_MAX, ST_STEER_SIDE_BITS)
               << ST_STEER_SIDE_SHIFT)
            | (pack_signed((int32_t) p_vc->steer_mag, 0,
                           (int32_t) VC_SEARCH_STEER, ST_STEER_MAG_BITS)
               << ST_STEER_MAG_SHIFT)
            | (pack_field((uint32_t) p_vc->lift_spell_cycles,
                          ST_LIFT_CYCLES_BITS)
               << ST_LIFT_CYCLES_SHIFT)
            | (pack_flag(p_vc->b_lift_spell_cued) << ST_FLAG_LIFT_CUED);

        p_packed->word[ST_WORD_CAL_ANGLES] =
            (pack_field((uint32_t) p_vc->cal_out_mdeg, ST_CAL_OUT_BITS)
             << ST_CAL_OUT_SHIFT)
            | (pack_signed(p_vc->cal_first_dir, ST_CAL_DIR_MIN,
                           ST_CAL_DIR_MAX, ST_CAL_FIRST_BITS)
               << ST_CAL_FIRST_SHIFT)
            | (pack_signed(p_vc->cal_dir, ST_CAL_DIR_MIN, ST_CAL_DIR_MAX,
                           ST_CAL_DIR_BITS)
               << ST_CAL_DIR_SHIFT);

        p_packed->word[ST_WORD_LIFT] =
            (pack_field((uint32_t) p_vc->cal_far_mdeg, ST_CAL_FAR_BITS)
             << ST_CAL_FAR_SHIFT)
            | (pack_field((uint32_t) p_vc->lift_spell_max_cycles,
                          ST_LIFT_MAX_BITS)
               << ST_LIFT_MAX_SHIFT);

        p_packed->word[ST_WORD_JUNC_ORIGIN] =
            (pack_signed(p_vc->junction_origin_mm, ST_JUNCTION_MIN,
                         ST_JUNCTION_MAX, ST_JUNCTION_BITS)
             << ST_JUNCTION_SHIFT);

        p_packed->word[ST_WORD_JUNC_SEEN] =
            (pack_signed(p_vc->junction_seen_mm, ST_JUNCTION_MIN,
                         ST_JUNCTION_MAX, ST_JUNCTION_BITS)
             << ST_JUNCTION_SHIFT);
    }
}

/*!
 * @brief Decode the stored controller state.
 *
 * @param[in]  p_packed Stored record (or NULL).
 * @param[out] p_vc     Controller state, untouched unless true is returned.
 *
 * @return false when an enum code, a counter or a reserved bit is not a
 *         value the code produces.
 */
bool
vc_unpack_state (vc_packed_t const * p_packed, vc_t * p_vc)
{
    bool     b_ok = false;
    uint32_t flags;
    uint32_t steer;
    uint32_t cal;
    uint32_t lift;
    uint32_t state;
    uint32_t nav;
    uint32_t line;
    uint32_t search;
    uint32_t phase;
    uint32_t turn;

    if ((NULL != p_packed) && (NULL != p_vc))
    {
        flags  = p_packed->word[ST_WORD_FLAGS];
        steer  = p_packed->word[ST_WORD_STEER];
        cal    = p_packed->word[ST_WORD_CAL_ANGLES];
        lift   = p_packed->word[ST_WORD_LIFT];

        state  = pack_get(flags, ST_STATE_SHIFT, ST_STATE_BITS);
        nav    = pack_get(flags, ST_NAV_SHIFT, ST_NAV_BITS);
        line   = pack_get(flags, ST_PREV_LINE_SHIFT, ST_PREV_LINE_BITS);
        search = pack_get(flags, ST_SEARCH_SHIFT, ST_SEARCH_BITS);
        phase  = pack_get(flags, ST_CAL_PHASE_SHIFT, ST_CAL_PHASE_BITS);
        turn   = pack_get(flags, ST_TURN_SHIFT, ST_TURN_BITS);

        b_ok = (ST_STATE_CODES > state)
               && (ST_NAV_CODES > nav)
               && (ST_LINE_CODES > line)
               && (ST_SEARCH_CODES > search)
               && (ST_CAL_PHASE_CODES > phase)
               && (ST_NAV_CODES > turn)
               && (0U == (flags & ST_FLAGS_RESERVED))
               && (0U == (cal & ST_CAL_ANGLES_RESERVED))
               && (0U == (lift & ST_LIFT_RESERVED))
               && (0U == (p_packed->word[ST_WORD_JUNC_ORIGIN]
                         & ST_JUNCTION_RESERVED))
               && (0U == (p_packed->word[ST_WORD_JUNC_SEEN]
                          & ST_JUNCTION_RESERVED));

        if (true == b_ok)
        {
            p_vc->state              = (vc_state_t) state;
            p_vc->entry_ms           = p_packed->word[ST_WORD_ENTRY_MS];
            p_vc->stored_nav         = (vc_nav_t) nav;
            p_vc->rescans            =
                (uint8_t) pack_get(flags, ST_RESCANS_SHIFT, ST_RESCANS_BITS);
            p_vc->b_obstacle_active  = pack_flag_get(flags, ST_FLAG_OBSTACLE);
            p_vc->b_reversing        = pack_flag_get(flags, ST_FLAG_REVERSING);
            p_vc->prev_line          = (vc_line_t) line;
            p_vc->last_side          = (int8_t) pack_signed_get(flags,
                                        ST_LAST_SIDE_SHIFT,
                                        ST_LAST_SIDE_BITS);
            p_vc->search             = (vc_search_t) search;
            p_vc->search_dir         = pack_signed_get(flags,
                                        ST_SEARCH_DIR_SHIFT,
                                        ST_SEARCH_DIR_BITS);
            p_vc->search_origin_mdeg = (int32_t)
                                        p_packed->word[ST_WORD_SEARCH_ORIG];
            p_vc->search_limit_mdeg  =
                (int32_t) pack_get(steer, ST_SEARCH_LIMIT_SHIFT,
                                   ST_SEARCH_LIMIT_BITS);
            p_vc->b_after_bypass     = pack_flag_get(flags, ST_FLAG_AFTER_BYP);
            p_vc->bypass_end_ms      = p_packed->word[ST_WORD_BYPASS_END];
            p_vc->cal_phase          = (vc_cal_phase_t) phase;
            p_vc->cal_phase_ms       = p_packed->word[ST_WORD_CAL_PHASE];
            p_vc->cal_out_mdeg       =
                (int32_t) pack_get(cal, ST_CAL_OUT_SHIFT, ST_CAL_OUT_BITS);
            p_vc->cal_far_mdeg       =
                (int32_t) pack_get(lift, ST_CAL_FAR_SHIFT, ST_CAL_FAR_BITS);
            p_vc->cal_first_dir      = pack_signed_get(cal, ST_CAL_FIRST_SHIFT,
                                                       ST_CAL_FIRST_BITS);
            p_vc->cal_dir            = pack_signed_get(cal, ST_CAL_DIR_SHIFT,
                                                       ST_CAL_DIR_BITS);
            p_vc->b_junction_hold    = pack_flag_get(flags, ST_FLAG_JUNC_HOLD);
            p_vc->b_junction_track   = pack_flag_get(flags,
                                                      ST_FLAG_JUNC_TRACK);
            p_vc->b_junction_seen    = pack_flag_get(flags, ST_FLAG_JUNC_SEEN);
            p_vc->junction_origin_mm = pack_signed_get(
                                           p_packed->word[ST_WORD_JUNC_ORIGIN],
                                           ST_JUNCTION_SHIFT,
                                           ST_JUNCTION_BITS);
            p_vc->junction_seen_mm   = pack_signed_get(
                                           p_packed->word[ST_WORD_JUNC_SEEN],
                                           ST_JUNCTION_SHIFT,
                                           ST_JUNCTION_BITS);
            p_vc->pending_turn       = (vc_nav_t) turn;
            p_vc->steer_side         =
                (int8_t) pack_signed_get(steer, ST_STEER_SIDE_SHIFT,
                                         ST_STEER_SIDE_BITS);
            p_vc->b_right_exit       =
                pack_flag_get(flags, ST_FLAG_RIGHT_EXIT);
            p_vc->steer_mag          =
                (int16_t) pack_get(steer, ST_STEER_MAG_SHIFT,
                                   ST_STEER_MAG_BITS);
            p_vc->lift_spell_cycles  =
                (uint8_t) pack_get(steer, ST_LIFT_CYCLES_SHIFT,
                                   ST_LIFT_CYCLES_BITS);
            p_vc->lift_spell_max_cycles =
                (uint8_t) pack_get(lift, ST_LIFT_MAX_SHIFT, ST_LIFT_MAX_BITS);
            p_vc->b_lift_spell_cued  = pack_flag_get(steer, ST_FLAG_LIFT_CUED);
        }
    }

    return (b_ok);
}

/*!
 * @brief Copy a bypass plan into its stored form.
 *
 * @param[in]  p_plan   Bypass plan (or NULL).
 * @param[out] p_packed Stored record (or NULL).
 */
void
vc_pack_plan (obstacle_bypass_plan_t const * p_plan,
              vc_plan_packed_t *             p_packed)
{
    if ((NULL != p_plan) && (NULL != p_packed))
    {
        p_packed->word[PLAN_WORD_LEGS] =
            (pack_field(p_plan->out_mm, PLAN_OUT_BITS) << PLAN_OUT_SHIFT)
            | (pack_field(p_plan->back_mm, PLAN_BACK_MM_BITS)
               << PLAN_BACK_MM_SHIFT)
            | (pack_field(p_plan->turn_deg, PLAN_TURN_BITS)
               << PLAN_TURN_SHIFT);

        p_packed->word[PLAN_WORD_ALONG] =
            (pack_field(p_plan->along_mm, PLAN_ALONG_BITS) << PLAN_ALONG_SHIFT)
            | (pack_field(p_plan->check_mm, PLAN_CHECK_BITS)
               << PLAN_CHECK_SHIFT)
            | (pack_enum((uint32_t) p_plan->side, OBSTACLE_ACTION_COUNT,
                         PLAN_SIDE_BITS)
               << PLAN_SIDE_SHIFT);

        p_packed->word[PLAN_WORD_GEOMETRY] =
            (pack_field(p_plan->side_mm, PLAN_SIDE_MM_BITS)
             << PLAN_SIDE_MM_SHIFT)
            | (pack_field(p_plan->back_deg, PLAN_BACK_DEG_BITS)
               << PLAN_BACK_DEG_SHIFT);
    }
}

/*!
 * @brief Decode a stored bypass plan.
 *
 * @param[in]  p_packed Stored record (or NULL).
 * @param[out] p_plan   Bypass plan, untouched unless true is returned.
 *
 * @return false when the side is not a turn or a reserved bit is set.
 */
bool
vc_unpack_plan (vc_plan_packed_t const * p_packed,
                obstacle_bypass_plan_t * p_plan)
{
    bool                 b_ok = false;
    uint32_t             legs;
    uint32_t             along;
    uint32_t             geometry;
    uint32_t             side;

    if ((NULL != p_packed) && (NULL != p_plan))
    {
        legs     = p_packed->word[PLAN_WORD_LEGS];
        along    = p_packed->word[PLAN_WORD_ALONG];
        geometry = p_packed->word[PLAN_WORD_GEOMETRY];
        side     = pack_get(along, PLAN_SIDE_SHIFT, PLAN_SIDE_BITS);

        /* The planner produces only the two turns for a side. */
        b_ok = ((OBSTACLE_ACTION_TURN_LEFT == side)
                || (OBSTACLE_ACTION_TURN_RIGHT == side))
               && (0U == (legs & PLAN_LEGS_RESERVED))
               && (0U == (along & PLAN_ALONG_RESERVED))
               && (0U == (geometry & PLAN_GEOM_RESERVED));

        if (true == b_ok)
        {
            p_plan->side     = (obstacle_action_t) side;
            p_plan->turn_deg = pack_get(legs, PLAN_TURN_SHIFT,
                                        PLAN_TURN_BITS);
            p_plan->out_mm   = pack_get(legs, PLAN_OUT_SHIFT, PLAN_OUT_BITS);
            p_plan->side_mm  = pack_get(geometry, PLAN_SIDE_MM_SHIFT,
                                        PLAN_SIDE_MM_BITS);
            p_plan->along_mm = pack_get(along, PLAN_ALONG_SHIFT,
                                        PLAN_ALONG_BITS);
            p_plan->back_deg = pack_get(geometry, PLAN_BACK_DEG_SHIFT,
                                        PLAN_BACK_DEG_BITS);
            p_plan->back_mm  = pack_get(legs, PLAN_BACK_MM_SHIFT,
                                        PLAN_BACK_MM_BITS);
            p_plan->check_mm = pack_get(along, PLAN_CHECK_SHIFT,
                                        PLAN_CHECK_BITS);
        }
    }

    return (b_ok);
}

/*** end of file ***/
