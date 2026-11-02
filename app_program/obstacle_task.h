/** @file obstacle_task.h
 *
 * @brief Buddy 5 obstacle task: the interface for the Vehicle Controller.
 *
 * The task ranges straight ahead about every 60 ms (MONITOR state) and
 * publishes:
 *   - a packed 32-bit status word (obstacle_calc.h), read lock-free with
 *     obstacle_task_get_status() or obstacle_task_get_word();
 *   - event-flag bits on obstacle_task_get_flag(), for tk_wai_flg():
 *       OBSTACLE_EVT_AHEAD  set when an obstacle is ahead; stays set until
 *                           the controller calls obstacle_task_ack_ahead()
 *       OBSTACLE_EVT_FAULT  set while the sensor is in SENSOR_FAULT
 *       OBSTACLE_EVT_SCAN_DONE  set when a requested scan has a result;
 *                           cleared when the next scan starts
 *       OBSTACLE_EVT_PATH_DONE  the same for a path check
 *     Only the obstacle task writes these bits.  Wait for them without
 *     TWF_CLR or TWF_BITCLR.
 *
 * Buddy 5 never commands the motors: the controller decides what to do.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h>,
 * <tk/tkernel.h> and "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_TASK_H
#define OBSTACLE_TASK_H

#define OBSTACLE_EVT_AHEAD  (0x00000001U)
#define OBSTACLE_EVT_FAULT  (0x00000002U)
#define OBSTACLE_EVT_SCAN_DONE (0x00000004U)
#define OBSTACLE_EVT_PATH_DONE (0x00000008U)

/* Start the sensor and servo drivers and create the event flag.  Call once
 * from usermain() before obstacle_task_start(). */
bool obstacle_task_init(void);

/* Create and start the task (priority 3, core 1). */
bool obstacle_task_start(void);

/* Event flag carrying the OBSTACLE_EVT_* bits (0 before init). */
ID obstacle_task_get_flag(void);

/* Latest status, decoded.  false if there is none yet or it is older than
 * OBSTACLE_STALE_MS (kernel time): treat that as "no data", not "clear". */
bool obstacle_task_get_status(obstacle_status_t * p_status);

/* Latest packed status word as published, for telemetry (no age check). */
uint32_t obstacle_task_get_word(void);

/* The controller has acted on OBSTACLE_EVT_AHEAD (stopped, will scan).
 * The task clears the bit within one reading, and raises it again if the
 * obstacle is still there. */
void obstacle_task_ack_ahead(void);

/* The car has moved somewhere new (a bypass ended): MONITOR forgets its
 * earlier readings, so its lost-obstacle rule cannot fire on something it
 * saw before.  Takes effect within one reading. */
void obstacle_task_forget(void);

/* Ask for a scan (the car must be stopped).  Takes about 2 s; then
 * OBSTACLE_EVT_SCAN_DONE is set and the result can be read. */
void obstacle_task_request_scan(void);

/* Latest complete scan: every angle, edges, profile and recommendation.
 * false if there is none or one is in progress. */
bool obstacle_task_get_scan(obstacle_scan_result_t * p_scan);

/* Just the profile and recommendation (profile.action) of the latest
 * complete scan.  false if there is none or one is in progress. */
bool obstacle_task_get_profile(obstacle_profile_t * p_profile);

/* After the first bypass pivot: is the leg free for need_mm?  Pass the
 * plan's side and check_mm, and the scan's profile.distance_mm as
 * known_mm so the obstacle that was scanned is not taken for a new one.
 * Looks from the bypass side back to straight ahead; takes about 1.2 s,
 * then OBSTACLE_EVT_PATH_DONE is set and the verdict can be read. */
void obstacle_task_request_path_check(obstacle_action_t side,
                                      uint32_t          need_mm,
                                      uint32_t          known_mm);

/* Latest complete path check.  false if there is none or one is running.
 * Drive the leg only on OBSTACLE_PATH_CLEAR (OPEN is the controller's
 * call: nothing echoed, but an angled face is silent too).  BLOCKED also
 * means the outermost angle could not see past the scanned obstacle. */
bool obstacle_task_get_path(obstacle_path_t * p_path);

#endif /* OBSTACLE_TASK_H */

/*** end of file ***/
