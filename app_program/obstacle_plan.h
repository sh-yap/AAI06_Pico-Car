/** @file obstacle_plan.h
 *
 * @brief Buddy 5 bypass planning: the "^" path around an obstacle.
 *
 * Pure logic with no hardware or RTOS calls, so it is unit-tested on the
 * host by tests/obstacle (including a corner-by-corner simulation).
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_PLAN_H
#define OBSTACLE_PLAN_H

/* Plan the "^" bypass for an obstacle whose nearest face is distance_mm
 * from the sensor, passing on side (TURN_LEFT or TURN_RIGHT).  false if
 * the side is not a turn or no angle up to OBSTACLE_PLAN_MAX_DEG works. */
bool obstacle_plan_bypass(uint32_t                 distance_mm,
                          obstacle_action_t        side,
                          obstacle_bypass_plan_t * p_plan);

#endif /* OBSTACLE_PLAN_H */

/*** end of file ***/
