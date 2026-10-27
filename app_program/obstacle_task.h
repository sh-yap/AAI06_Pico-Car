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

#endif /* OBSTACLE_TASK_H */

/*** end of file ***/
