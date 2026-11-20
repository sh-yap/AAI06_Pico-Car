/** @file obstacle_pack.h
 *
 * @brief Packed stored copies of the obstacle scan and path results (CR1).
 *
 * Only the copies that the obstacle task stores in static RAM are packed:
 * see docs/DATA_REPRESENTATION.md sections 3.2 and 3.3.  The value types
 * obstacle_scan_result_t, obstacle_path_t and obstacle_scan_point_t stay
 * full-width for callers and for the calculations; the task packs a
 * working copy on publish and the readers unpack it inside the existing
 * seqlock check.
 *
 * Rules (docs/DATA_REPRESENTATION.md section 0): unsigned words, explicit
 * masks and shifts, no C bit-fields, signed fields sign-extended, a field
 * never straddles a word boundary, and an unknown enum code read back is
 * reported as "no result" rather than guessed.
 *
 * Scan record: OBSTACLE_SCAN_PACKED_WORDS = 17 words (68 B).
 *
 *   W0..W12  one scan point each:
 *              bits  0..6   angle_deg     7 U, absolute degrees
 *              bits  7..18  distance_mm   12 U, saturates at 4095
 *              bits 19..22  valid_count   4 U (SCAN_PINGS_MAX = 8 pings)
 *              bits 23..26  silent_count  4 U
 *              bits 27..28  point_class   2 U, obstacle_point_class_t
 *              bits 29..31  reserved, 0
 *   W13      bits  0..3   count          4 U, 0..OBSTACLE_SCAN_MAX_POINTS
 *            bits  4..10  right_edge_deg 7 U
 *            bits 11..17  left_edge_deg  7 U
 *            bits 18..31  reserved, 0
 *   W14      bits  0..15  duration_ms    16 U, saturates at 65535
 *            bits 16..31  reserved, 0
 *   W15      profile word (obstacle_calc.h) bits 0..31
 *   W16      profile word bits 32..63
 *
 * Path record: OBSTACLE_PATH_PACKED_WORDS = 3 words (12 B).
 *
 *   W0      bits  0..2   verdict      3 U, obstacle_path_verdict_t
 *           bits  3..14  need_mm      12 U, saturates at 4095
 *           bits 15..26  known_mm     12 U, saturates at 4095
 *           bits 27..28  count        2 U, 0..OBSTACLE_PATH_MAX_POINTS
 *           bits 29..31  points[0].verdict
 *   W1      bits  0..6   points[0].angle_deg     7 U
 *           bits  7..18  points[0].distance_mm  12 U
 *           bits 19..30  points[1].distance_mm  12 U
 *           bit  31      reserved, 0
 *   W2      bits  0..11  points[2].distance_mm  12 U
 *           bits 12..18  points[1].angle_deg     7 U
 *           bits 19..25  points[2].angle_deg     7 U
 *           bits 26..28  points[1].verdict      3 U
 *           bits 29..31  points[2].verdict      3 U
 *
 * 95 field bits do not fit three 32-bit words when each point keeps its
 * fields together (a 22-bit point cannot share a word with another), so
 * one point verdict sits in W0 and the remaining point fields are split
 * between W1 and W2.  Every field still lies inside one word.
 *
 * Path-check request word: 1 word (4 B).
 *
 *   bits  0..11  need_mm   12 U, saturates at 4095
 *   bits 12..23  known_mm  12 U, saturates at 4095
 *   bits 24..26  side      3 U, obstacle_action_t (TURN_LEFT/TURN_RIGHT)
 *   bits 27..31  reserved, 0
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_PACK_H
#define OBSTACLE_PACK_H

/* Words in the stored scan record (13 points, 1 header, 1 duration,
 * 2 profile). */
#define OBSTACLE_SCAN_PACKED_WORDS  (17U)

/* Words in the stored path-check record. */
#define OBSTACLE_PATH_PACKED_WORDS  (3U)

/* The stored scan record: the obstacle task's static copy. */
typedef struct
{
    uint32_t word[OBSTACLE_SCAN_PACKED_WORDS];
} obstacle_scan_packed_t;

/* The stored path-check record: the obstacle task's static copy. */
typedef struct
{
    uint32_t word[OBSTACLE_PATH_PACKED_WORDS];
} obstacle_path_packed_t;

/* Copy a scan result into its stored form.  Values outside a field
 * saturate at the field's end; an invalid enum code is stored as 0 and
 * makes obstacle_unpack_scan() report "no result". */
void obstacle_pack_scan(obstacle_scan_result_t const * p_scan,
                        obstacle_scan_packed_t *      p_packed);

/* Decode a stored scan record.  false if an enum code or the point count
 * is not a value the code can produce: treat that as "no scan". */
bool obstacle_unpack_scan(obstacle_scan_packed_t const * p_packed,
                          obstacle_scan_result_t *      p_scan);

/* The scan's 64-bit profile word (obstacle_calc.h) from a stored record,
 * without decoding all 13 points. */
uint64_t obstacle_unpack_scan_profile(obstacle_scan_packed_t const * p_packed);

/* Copy a path check into its stored form (same saturation rule). */
void obstacle_pack_path(obstacle_path_t const * p_path,
                        obstacle_path_packed_t * p_packed);

/* Decode a stored path-check record.  false if a verdict code or the
 * point count is not a value the code can produce. */
bool obstacle_unpack_path(obstacle_path_packed_t const * p_packed,
                          obstacle_path_t *             p_path);

/* Pack the controller's path-check request (side, need_mm, known_mm) into
 * one word, so that it is published with one 32-bit store and the task
 * can never read a mixed set of values. */
uint32_t obstacle_pack_path_request(obstacle_action_t side,
                                    uint32_t          need_mm,
                                    uint32_t          known_mm);

/* Decode a path-check request word.  An unknown side code decodes to
 * OBSTACLE_ACTION_CONTINUE, which asks for straight-ahead readings (see
 * obstacle_scan_path_angle()). */
void obstacle_unpack_path_request(uint32_t            word,
                                  obstacle_action_t * p_side,
                                  uint32_t *          p_need_mm,
                                  uint32_t *          p_known_mm);

#endif /* OBSTACLE_PACK_H */

/*** end of file ***/
