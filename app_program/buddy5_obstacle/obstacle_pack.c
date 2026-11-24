/** @file obstacle_pack.c
 *
 * @brief Packed stored copies of the obstacle scan and path results (CR1).
 *
 * The bit layouts are documented in obstacle_pack.h.  Only stored or
 * shared copies are packed; the calculations and the caller's value types
 * stay full width.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "obstacle_types.h"
#include "obstacle_calc.h"
#include "obstacle_pack.h"

/* The record layouts below are fixed to these array sizes. */
#if (OBSTACLE_SCAN_MAX_POINTS != 13U)
#error "Packed scan record assumes 13 scan points"
#endif

#if (OBSTACLE_PATH_MAX_POINTS != 3U)
#error "Packed path record assumes 3 path points"
#endif

/* Value counts of the shared enums (obstacle_types.h): a code that is not
 * below its count is not a value the code produces. */
#define SCAN_CLASS_COUNT   (3U)   /* UNKNOWN, OBSTACLE, BACKGROUND */
#define PATH_VERDICT_COUNT (5U)   /* CLEAR .. KNOWN */

/* Scan record: one point per word, then header, duration and profile. */
#define SCAN_WORD_POINTS     (0U)
#define SCAN_WORD_HEADER     (OBSTACLE_SCAN_MAX_POINTS)
#define SCAN_WORD_DURATION   (OBSTACLE_SCAN_MAX_POINTS + 1U)
#define SCAN_WORD_PROFILE_LO (OBSTACLE_SCAN_MAX_POINTS + 2U)
#define SCAN_WORD_PROFILE_HI (OBSTACLE_SCAN_MAX_POINTS + 3U)

/* One scan point. */
#define SCAN_PT_ANGLE_SHIFT   (0U)
#define SCAN_PT_ANGLE_BITS    (7U)
#define SCAN_PT_DIST_SHIFT    (7U)
#define SCAN_PT_DIST_BITS     (12U)
#define SCAN_PT_VALID_SHIFT   (19U)
#define SCAN_PT_VALID_BITS    (4U)
#define SCAN_PT_SILENT_SHIFT  (23U)
#define SCAN_PT_SILENT_BITS   (4U)
#define SCAN_PT_CLASS_SHIFT   (27U)
#define SCAN_PT_CLASS_BITS    (2U)

/* Scan header and duration words. */
#define SCAN_COUNT_SHIFT      (0U)
#define SCAN_COUNT_BITS       (4U)
#define SCAN_EDGE_BITS        (7U)
#define SCAN_RIGHT_SHIFT      (4U)
#define SCAN_LEFT_SHIFT       (11U)
#define SCAN_DURATION_SHIFT   (0U)
#define SCAN_DURATION_BITS    (16U)

/* Path record: header word, then the remaining point fields. */
#define PATH_WORD_HEADER      (0U)
#define PATH_WORD_1           (1U)
#define PATH_WORD_2           (2U)

#define PATH_VERDICT_SHIFT    (0U)
#define PATH_VERDICT_BITS     (3U)
#define PATH_NEED_SHIFT       (3U)
#define PATH_NEED_BITS        (12U)
#define PATH_KNOWN_SHIFT      (15U)
#define PATH_KNOWN_BITS       (12U)
#define PATH_COUNT_SHIFT      (27U)
#define PATH_COUNT_BITS       (2U)
#define PATH_P0_VERDICT_SHIFT (29U)
#define PATH_P0_VERDICT_BITS  (3U)

#define PATH_P0_ANGLE_SHIFT   (0U)
#define PATH_P0_ANGLE_BITS    (7U)
#define PATH_P0_DIST_SHIFT    (7U)
#define PATH_P0_DIST_BITS     (12U)
#define PATH_P1_DIST_SHIFT    (19U)
#define PATH_P1_DIST_BITS     (12U)

#define PATH_P2_DIST_SHIFT    (0U)
#define PATH_P2_DIST_BITS     (12U)
#define PATH_P1_ANGLE_SHIFT   (12U)
#define PATH_P1_ANGLE_BITS    (7U)
#define PATH_P2_ANGLE_SHIFT   (19U)
#define PATH_P2_ANGLE_BITS    (7U)
#define PATH_P1_VERDICT_SHIFT (26U)
#define PATH_P1_VERDICT_BITS  (3U)
#define PATH_P2_VERDICT_SHIFT (29U)
#define PATH_P2_VERDICT_BITS  (3U)

/* Path-check request word. */
#define REQUEST_NEED_SHIFT    (0U)
#define REQUEST_NEED_BITS     (12U)
#define REQUEST_KNOWN_SHIFT   (12U)
#define REQUEST_KNOWN_BITS    (12U)
#define REQUEST_SIDE_SHIFT    (24U)
#define REQUEST_SIDE_BITS     (3U)

#define EDGE_REQ_KNOWN_SHIFT  (0U)
#define EDGE_REQ_KNOWN_BITS   (12U)
#define EDGE_REQ_TURN_SHIFT   (12U)
#define EDGE_REQ_TURN_BITS    (7U)
#define EDGE_REQ_SIDE_SHIFT   (19U)
#define EDGE_REQ_SIDE_BITS    (3U)

#define EDGE_RESULT_SHIFT     (0U)
#define EDGE_RESULT_BITS      (2U)
#define EDGE_FACE_SHIFT       (2U)
#define EDGE_FACE_BITS        (12U)
#define EDGE_HEARD_SHIFT      (14U)
#define EDGE_HEARD_BITS       (11U)

#define PACK_WORD_MASK        (0xFFFFFFFFULL)


/*!
 * @brief Mask for a field of 1..31 bits.
 *
 * @param[in] bits Field width.
 *
 * @return The mask, bit 0 = LSB.
 */
static uint32_t
pack_mask(uint32_t bits)
{
    return ((1U << bits) - 1U);
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
pack_field(uint32_t value, uint32_t bits)
{
    uint32_t mask = pack_mask(bits);

    return ((value > mask) ? mask : value);
}

/*!
 * @brief An enum placed in its field: 0 (the first code) if it is not one.
 *
 * @param[in] value Enum value.
 * @param[in] count Number of codes in the enum.
 *
 * @return value, or 0 if value is not a code of that enum.
 */
static uint32_t
pack_enum(uint32_t value, uint32_t count)
{
    return ((value < count) ? value : 0U);
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
pack_get(uint32_t word, uint32_t shift, uint32_t bits)
{
    return ((word >> shift) & pack_mask(bits));
}

/*!
 * @brief Copy a scan result into its stored form.
 *
 * @param[in]  p_scan   Scan result (or NULL).
 * @param[out] p_packed Stored record (or NULL).
 */
void
obstacle_pack_scan(obstacle_scan_result_t const * p_scan,
                   obstacle_scan_packed_t *      p_packed)
{
    uint32_t idx;
    uint32_t count;
    uint64_t profile;

    if ((NULL != p_scan) && (NULL != p_packed))
    {
        for (idx = 0U; idx < OBSTACLE_SCAN_MAX_POINTS; idx++)
        {
            obstacle_scan_point_t const * p_point = &p_scan->points[idx];

            p_packed->word[SCAN_WORD_POINTS + idx] =
                (pack_field(p_point->angle_deg, SCAN_PT_ANGLE_BITS)
                 << SCAN_PT_ANGLE_SHIFT)
                | (pack_field(p_point->distance_mm, SCAN_PT_DIST_BITS)
                   << SCAN_PT_DIST_SHIFT)
                | (pack_field(p_point->valid_count, SCAN_PT_VALID_BITS)
                   << SCAN_PT_VALID_SHIFT)
                | (pack_field(p_point->silent_count, SCAN_PT_SILENT_BITS)
                   << SCAN_PT_SILENT_SHIFT)
                | (pack_enum((uint32_t)p_point->point_class, SCAN_CLASS_COUNT)
                   << SCAN_PT_CLASS_SHIFT);
        }

        count = p_scan->count;

        if (count > OBSTACLE_SCAN_MAX_POINTS)
        {
            count = OBSTACLE_SCAN_MAX_POINTS;
        }

        p_packed->word[SCAN_WORD_HEADER] =
            (pack_field(count, SCAN_COUNT_BITS) << SCAN_COUNT_SHIFT)
            | (pack_field(p_scan->right_edge_deg, SCAN_EDGE_BITS)
               << SCAN_RIGHT_SHIFT)
            | (pack_field(p_scan->left_edge_deg, SCAN_EDGE_BITS)
               << SCAN_LEFT_SHIFT);

        p_packed->word[SCAN_WORD_DURATION] =
            (pack_field(p_scan->duration_ms, SCAN_DURATION_BITS)
             << SCAN_DURATION_SHIFT);

        profile = obstacle_calc_pack_profile(&p_scan->profile);
        p_packed->word[SCAN_WORD_PROFILE_LO] =
            (uint32_t)(profile & PACK_WORD_MASK);
        p_packed->word[SCAN_WORD_PROFILE_HI] = (uint32_t)(profile >> 32);
    }
}

/*!
 * @brief Decode a stored scan record.
 *
 * @param[in]  p_packed Stored record (or NULL).
 * @param[out] p_scan   Scan result (or NULL).
 *
 * @return false if the record holds an enum code or a point count that
 *         the code cannot produce; the result is then not a scan.
 */
bool
obstacle_unpack_scan(obstacle_scan_packed_t const * p_packed,
                     obstacle_scan_result_t *      p_scan)
{
    bool     b_valid;
    uint32_t idx;
    uint32_t word;
    uint32_t count;
    uint32_t point_class;

    b_valid = (NULL != p_packed) && (NULL != p_scan);

    if (b_valid)
    {
        count = pack_get(p_packed->word[SCAN_WORD_HEADER], SCAN_COUNT_SHIFT,
                         SCAN_COUNT_BITS);
        b_valid = (count <= OBSTACLE_SCAN_MAX_POINTS);
    }

    for (idx = 0U; b_valid && (idx < OBSTACLE_SCAN_MAX_POINTS); idx++)
    {
        word = p_packed->word[SCAN_WORD_POINTS + idx];
        point_class = pack_get(word, SCAN_PT_CLASS_SHIFT, SCAN_PT_CLASS_BITS);
        b_valid = (point_class < SCAN_CLASS_COUNT);

        p_scan->points[idx].angle_deg =
            pack_get(word, SCAN_PT_ANGLE_SHIFT, SCAN_PT_ANGLE_BITS);
        p_scan->points[idx].distance_mm =
            pack_get(word, SCAN_PT_DIST_SHIFT, SCAN_PT_DIST_BITS);
        p_scan->points[idx].valid_count =
            pack_get(word, SCAN_PT_VALID_SHIFT, SCAN_PT_VALID_BITS);
        p_scan->points[idx].silent_count =
            pack_get(word, SCAN_PT_SILENT_SHIFT, SCAN_PT_SILENT_BITS);
        p_scan->points[idx].point_class = (obstacle_point_class_t)point_class;
    }

    if (b_valid)
    {
        p_scan->count =
            pack_get(p_packed->word[SCAN_WORD_HEADER], SCAN_COUNT_SHIFT,
                     SCAN_COUNT_BITS);
        p_scan->right_edge_deg =
            pack_get(p_packed->word[SCAN_WORD_HEADER], SCAN_RIGHT_SHIFT,
                     SCAN_EDGE_BITS);
        p_scan->left_edge_deg =
            pack_get(p_packed->word[SCAN_WORD_HEADER], SCAN_LEFT_SHIFT,
                     SCAN_EDGE_BITS);
        p_scan->duration_ms =
            pack_get(p_packed->word[SCAN_WORD_DURATION], SCAN_DURATION_SHIFT,
                     SCAN_DURATION_BITS);
        b_valid = obstacle_calc_unpack_profile(
            obstacle_unpack_scan_profile(p_packed), &p_scan->profile);
    }

    return (b_valid);
}

/*!
 * @brief The scan's profile word from a stored record.
 *
 * @param[in] p_packed Stored record (or NULL).
 *
 * @return The 64-bit profile word; 0 if p_packed is NULL.
 */
uint64_t
obstacle_unpack_scan_profile(obstacle_scan_packed_t const * p_packed)
{
    uint64_t word = 0U;

    if (NULL != p_packed)
    {
        word = ((uint64_t)p_packed->word[SCAN_WORD_PROFILE_HI] << 32)
               | (uint64_t)p_packed->word[SCAN_WORD_PROFILE_LO];
    }

    return (word);
}

/*!
 * @brief Copy a path check into its stored form.
 *
 * @param[in]  p_path   Path check (or NULL).
 * @param[out] p_packed Stored record (or NULL).
 */
void
obstacle_pack_path(obstacle_path_t const * p_path,
                   obstacle_path_packed_t * p_packed)
{
    uint32_t count;

    if ((NULL != p_path) && (NULL != p_packed))
    {
        count = p_path->count;

        if (count > OBSTACLE_PATH_MAX_POINTS)
        {
            count = OBSTACLE_PATH_MAX_POINTS;
        }

        p_packed->word[PATH_WORD_HEADER] =
            (pack_enum((uint32_t)p_path->verdict, PATH_VERDICT_COUNT)
             << PATH_VERDICT_SHIFT)
            | (pack_field(p_path->need_mm, PATH_NEED_BITS)
               << PATH_NEED_SHIFT)
            | (pack_field(p_path->known_mm, PATH_KNOWN_BITS)
               << PATH_KNOWN_SHIFT)
            | (pack_field(count, PATH_COUNT_BITS) << PATH_COUNT_SHIFT)
            | (pack_enum((uint32_t)p_path->points[0].verdict,
                         PATH_VERDICT_COUNT)
               << PATH_P0_VERDICT_SHIFT);

        p_packed->word[PATH_WORD_1] =
            (pack_field(p_path->points[0].angle_deg, PATH_P0_ANGLE_BITS)
             << PATH_P0_ANGLE_SHIFT)
            | (pack_field(p_path->points[0].distance_mm, PATH_P0_DIST_BITS)
               << PATH_P0_DIST_SHIFT)
            | (pack_field(p_path->points[1].distance_mm, PATH_P1_DIST_BITS)
               << PATH_P1_DIST_SHIFT);

        p_packed->word[PATH_WORD_2] =
            (pack_field(p_path->points[2].distance_mm, PATH_P2_DIST_BITS)
             << PATH_P2_DIST_SHIFT)
            | (pack_field(p_path->points[1].angle_deg, PATH_P1_ANGLE_BITS)
               << PATH_P1_ANGLE_SHIFT)
            | (pack_field(p_path->points[2].angle_deg, PATH_P2_ANGLE_BITS)
               << PATH_P2_ANGLE_SHIFT)
            | (pack_enum((uint32_t)p_path->points[1].verdict,
                         PATH_VERDICT_COUNT)
               << PATH_P1_VERDICT_SHIFT)
            | (pack_enum((uint32_t)p_path->points[2].verdict,
                         PATH_VERDICT_COUNT)
               << PATH_P2_VERDICT_SHIFT);
    }
}

/*!
 * @brief Decode a stored path-check record.
 *
 * @param[in]  p_packed Stored record (or NULL).
 * @param[out] p_path   Path check (or NULL).
 *
 * @return false if the record holds a verdict code or a count that the
 *         code cannot produce; the result is then not a path check.
 */
bool
obstacle_unpack_path(obstacle_path_packed_t const * p_packed,
                     obstacle_path_t *             p_path)
{
    bool     b_valid;
    uint32_t count;
    uint32_t verdict;
    uint32_t p0_verdict;
    uint32_t p1_verdict;
    uint32_t p2_verdict;

    b_valid = (NULL != p_packed) && (NULL != p_path);

    if (b_valid)
    {
        count = pack_get(p_packed->word[PATH_WORD_HEADER], PATH_COUNT_SHIFT,
                         PATH_COUNT_BITS);
        verdict = pack_get(p_packed->word[PATH_WORD_HEADER],
                           PATH_VERDICT_SHIFT, PATH_VERDICT_BITS);
        p0_verdict = pack_get(p_packed->word[PATH_WORD_HEADER],
                              PATH_P0_VERDICT_SHIFT, PATH_P0_VERDICT_BITS);
        p1_verdict = pack_get(p_packed->word[PATH_WORD_2],
                              PATH_P1_VERDICT_SHIFT, PATH_P1_VERDICT_BITS);
        p2_verdict = pack_get(p_packed->word[PATH_WORD_2],
                              PATH_P2_VERDICT_SHIFT, PATH_P2_VERDICT_BITS);

        b_valid = (count <= OBSTACLE_PATH_MAX_POINTS)
                  && (verdict < PATH_VERDICT_COUNT)
                  && (p0_verdict < PATH_VERDICT_COUNT)
                  && (p1_verdict < PATH_VERDICT_COUNT)
                  && (p2_verdict < PATH_VERDICT_COUNT);
    }

    if (b_valid)
    {
        p_path->verdict = (obstacle_path_verdict_t)verdict;
        p_path->need_mm =
            pack_get(p_packed->word[PATH_WORD_HEADER], PATH_NEED_SHIFT,
                     PATH_NEED_BITS);
        p_path->known_mm =
            pack_get(p_packed->word[PATH_WORD_HEADER], PATH_KNOWN_SHIFT,
                     PATH_KNOWN_BITS);
        p_path->count = count;

        p_path->points[0].angle_deg =
            pack_get(p_packed->word[PATH_WORD_1], PATH_P0_ANGLE_SHIFT,
                     PATH_P0_ANGLE_BITS);
        p_path->points[0].distance_mm =
            pack_get(p_packed->word[PATH_WORD_1], PATH_P0_DIST_SHIFT,
                     PATH_P0_DIST_BITS);
        p_path->points[0].verdict = (obstacle_path_verdict_t)p0_verdict;

        p_path->points[1].angle_deg =
            pack_get(p_packed->word[PATH_WORD_2], PATH_P1_ANGLE_SHIFT,
                     PATH_P1_ANGLE_BITS);
        p_path->points[1].distance_mm =
            pack_get(p_packed->word[PATH_WORD_1], PATH_P1_DIST_SHIFT,
                     PATH_P1_DIST_BITS);
        p_path->points[1].verdict = (obstacle_path_verdict_t)p1_verdict;

        p_path->points[2].angle_deg =
            pack_get(p_packed->word[PATH_WORD_2], PATH_P2_ANGLE_SHIFT,
                     PATH_P2_ANGLE_BITS);
        p_path->points[2].distance_mm =
            pack_get(p_packed->word[PATH_WORD_2], PATH_P2_DIST_SHIFT,
                     PATH_P2_DIST_BITS);
        p_path->points[2].verdict = (obstacle_path_verdict_t)p2_verdict;
    }

    return (b_valid);
}

/*!
 * @brief Pack the controller's path-check request into one word.
 *
 * @param[in] side     The plan's side (TURN_LEFT or TURN_RIGHT).
 * @param[in] need_mm  Free distance needed.
 * @param[in] known_mm The scan's closest distance; 0 if none.
 *
 * @return The request word; the task decodes it with one read.
 */
uint32_t
obstacle_pack_path_request(obstacle_action_t side,
                           uint32_t          need_mm,
                           uint32_t          known_mm)
{
    return ((pack_field(need_mm, REQUEST_NEED_BITS) << REQUEST_NEED_SHIFT)
            | (pack_field(known_mm, REQUEST_KNOWN_BITS) << REQUEST_KNOWN_SHIFT)
            | (pack_enum((uint32_t)side, (uint32_t)OBSTACLE_ACTION_COUNT)
               << REQUEST_SIDE_SHIFT));
}

/*!
 * @brief Decode a path-check request word.
 *
 * @param[in]  word      Request word.
 * @param[out] p_side    The side (or NULL).
 * @param[out] p_need_mm Free distance needed (or NULL).
 * @param[out] p_known_mm The scan's closest distance (or NULL).
 */
void
obstacle_unpack_path_request(uint32_t            word,
                             obstacle_action_t * p_side,
                             uint32_t *          p_need_mm,
                             uint32_t *          p_known_mm)
{
    uint32_t side = pack_get(word, REQUEST_SIDE_SHIFT, REQUEST_SIDE_BITS);

    if (side >= (uint32_t)OBSTACLE_ACTION_COUNT)
    {
        /* Not a code this code can produce: look straight ahead. */
        side = (uint32_t)OBSTACLE_ACTION_CONTINUE;
    }

    if (NULL != p_side)
    {
        *p_side = (obstacle_action_t)side;
    }

    if (NULL != p_need_mm)
    {
        *p_need_mm = pack_get(word, REQUEST_NEED_SHIFT, REQUEST_NEED_BITS);
    }

    if (NULL != p_known_mm)
    {
        *p_known_mm = pack_get(word, REQUEST_KNOWN_SHIFT, REQUEST_KNOWN_BITS);
    }
}

/*!
 * @brief Pack the controller's edge-watch request into one word.
 *
 * @param[in] side     The plan's side (TURN_LEFT or TURN_RIGHT).
 * @param[in] turn_deg The plan's first pivot.
 * @param[in] known_mm The scan's closest distance.
 *
 * @return The request word; the task decodes it with one read.
 */
uint32_t
obstacle_pack_edge_request(obstacle_action_t side,
                           uint32_t          turn_deg,
                           uint32_t          known_mm)
{
    return ((pack_field(known_mm, EDGE_REQ_KNOWN_BITS) << EDGE_REQ_KNOWN_SHIFT)
            | (pack_field(turn_deg, EDGE_REQ_TURN_BITS) << EDGE_REQ_TURN_SHIFT)
            | (pack_enum((uint32_t)side, (uint32_t)OBSTACLE_ACTION_COUNT)
               << EDGE_REQ_SIDE_SHIFT));
}

/*!
 * @brief Decode an edge-watch request word.
 *
 * @param[in]  word       Request word.
 * @param[out] p_side     The side (or NULL); an unknown code decodes to
 *                        CONTINUE, which looks straight ahead.
 * @param[out] p_turn_deg The plan's first pivot (or NULL).
 * @param[out] p_known_mm The scan's closest distance (or NULL).
 */
void
obstacle_unpack_edge_request(uint32_t            word,
                             obstacle_action_t * p_side,
                             uint32_t *          p_turn_deg,
                             uint32_t *          p_known_mm)
{
    uint32_t side = pack_get(word, EDGE_REQ_SIDE_SHIFT, EDGE_REQ_SIDE_BITS);

    if (side >= (uint32_t)OBSTACLE_ACTION_COUNT)
    {
        side = (uint32_t)OBSTACLE_ACTION_CONTINUE;
    }

    if (NULL != p_side)
    {
        *p_side = (obstacle_action_t)side;
    }

    if (NULL != p_turn_deg)
    {
        *p_turn_deg = pack_get(word, EDGE_REQ_TURN_SHIFT, EDGE_REQ_TURN_BITS);
    }

    if (NULL != p_known_mm)
    {
        *p_known_mm = pack_get(word, EDGE_REQ_KNOWN_SHIFT,
                               EDGE_REQ_KNOWN_BITS);
    }
}

/*!
 * @brief Pack the parts of an edge watch the controller reads.
 *
 * @param[in] p_watch Watch state (NULL packs as WATCHING, nothing heard).
 *
 * @return The result word: result, face_mm and heard_mm (negative leg
 *         distances store as 0; values beyond a field saturate).
 */
uint32_t
obstacle_pack_edge(obstacle_edge_watch_t const * p_watch)
{
    uint32_t word = 0U;
    uint32_t heard_mm;

    if (NULL != p_watch)
    {
        /* Cast to uint32_t: heard_mm was just checked non-negative. */
        heard_mm = (p_watch->heard_mm > 0) ? (uint32_t)p_watch->heard_mm : 0U;
        word = (pack_enum((uint32_t)p_watch->result,
                          (uint32_t)OBSTACLE_EDGE_COUNT) << EDGE_RESULT_SHIFT)
               | (pack_field(p_watch->face_mm, EDGE_FACE_BITS)
                  << EDGE_FACE_SHIFT)
               | (pack_field(heard_mm, EDGE_HEARD_BITS) << EDGE_HEARD_SHIFT);
    }

    return (word);
}

/*!
 * @brief Decode an edge-watch result word.
 *
 * @param[in]  word    Result word.
 * @param[out] p_watch result, face_mm and heard_mm; the counters are 0.
 *
 * @return false if p_watch is NULL.  (Every 2-bit code is a result.)
 */
bool
obstacle_unpack_edge(uint32_t word, obstacle_edge_watch_t * p_watch)
{
    bool b_ok = false;

    if (NULL != p_watch)
    {
        p_watch->result = (obstacle_edge_result_t)
                          pack_get(word, EDGE_RESULT_SHIFT, EDGE_RESULT_BITS);
        p_watch->known_mm   = 0U;
        p_watch->face_mm    = pack_get(word, EDGE_FACE_SHIFT, EDGE_FACE_BITS);
        /* Cast to int32_t: at most 11 bits. */
        p_watch->heard_mm   = (int32_t)pack_get(word, EDGE_HEARD_SHIFT,
                                                EDGE_HEARD_BITS);
        p_watch->face_count = 0U;
        p_watch->lost_count = 0U;
        p_watch->readings   = 0U;
        b_ok = true;
    }

    return (b_ok);
}

/*** end of file ***/
