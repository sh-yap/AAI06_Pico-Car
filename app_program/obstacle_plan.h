/** @file obstacle_plan.h
 *
 * @brief Buddy 5 bypass planning: the out-along-back path round an obstacle.
 *
 * Pure logic with no hardware or RTOS calls, so it is unit-tested on the
 * host by tests/obstacle (including a corner-by-corner simulation).
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_PLAN_H
#define OBSTACLE_PLAN_H

/* Plan the bypass for an obstacle whose nearest face is distance_mm
 * from the sensor, passing on side (TURN_LEFT or TURN_RIGHT).  false if
 * the side is not a turn or no angle up to OBSTACLE_PLAN_MAX_DEG works. */
bool obstacle_plan_bypass(uint32_t                 distance_mm,
                          obstacle_action_t        side,
                          obstacle_bypass_plan_t * p_plan);

/* Start watching the line sensors at the start of a leg. */
void obstacle_plan_line_reset(obstacle_line_watch_t * p_watch);

/* Each step on the leg: b_black is true while a line sensor sees black
 * now, leg_mm is the distance driven on the leg.  HELD once black has
 * lasted OBSTACLE_PLAN_LINE_HELD_MM, CROSSED when it goes white sooner. */
obstacle_line_seen_t obstacle_plan_line_watch(obstacle_line_watch_t * p_watch,
                                              bool                    b_black,
                                              int32_t                 leg_mm);

#endif /* OBSTACLE_PLAN_H */

/*** end of file ***/
