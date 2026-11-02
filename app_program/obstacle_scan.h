/** @file obstacle_scan.h
 *
 * @brief Buddy 5 scan analysis: obstacle profile and recommendation.
 *
 * Pure logic with no hardware or RTOS calls, so it is unit-tested on the
 * host by tests/obstacle.  The obstacle task fills one point per angle
 * with obstacle_scan_reduce(), then calls obstacle_scan_analyse().
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

/* Reduce the pings taken at one angle to a scan point: the median of the
 * OK pings, or "unknown" if fewer than OBSTACLE_SCAN_MIN_VALID were OK. */
void obstacle_scan_reduce(uint32_t                 angle_deg,
                          obstacle_range_t const * p_pings,
                          uint32_t                 ping_count,
                          obstacle_scan_point_t *  p_point);

/* Classify p_result->points[0..count-1] (ascending angles) and fill the
 * profile, the obstacle span edges and the recommended action. */
void obstacle_scan_analyse(obstacle_scan_result_t * p_result);

/* How the bypass side is chosen: OBSTACLE_PREFER_SCAN, _LEFT or _RIGHT.
 * Starts as OBSTACLE_BYPASS_PREFER (obstacle_config.h). */
void obstacle_scan_set_prefer(uint32_t prefer);

/* Path check after the first bypass pivot, one angle at a time:
 *   1. set need_mm, known_mm (the scan's closest distance) and count = 0;
 *   2. for idx = 0 .. OBSTACLE_PATH_MAX_POINTS - 1, take readings at
 *      obstacle_scan_path_angle(side, idx) and call obstacle_scan_path_add();
 *   3. call obstacle_scan_path_verdict(). */
uint32_t obstacle_scan_path_angle(obstacle_action_t side, uint32_t idx);

/* Each angle is CLEAR (at least need_mm), KNOWN (within the
 * OBSTACLE_PATH_KNOWN_* window of known_mm), BLOCKED (anything else
 * nearer), OPEN (no echo) or FAULT (sensor silent). */
void obstacle_scan_path_add(obstacle_path_t *        p_path,
                            uint32_t                 angle_deg,
                            obstacle_range_t const * p_pings,
                            uint32_t                 ping_count);

void obstacle_scan_path_verdict(obstacle_path_t * p_path);

#endif /* OBSTACLE_SCAN_H */

/*** end of file ***/
