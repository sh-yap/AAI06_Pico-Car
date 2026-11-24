/** @file obstacle_monitor.h
 *
 * @brief Buddy 5 MONITOR-state decisions: obstacle ahead, sensor fault.
 *
 * Pure logic with no hardware or RTOS calls, so it is unit-tested on the
 * host by tests/obstacle.  The obstacle task feeds it one reading at a
 * time.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_MONITOR_H
#define OBSTACLE_MONITOR_H

/* Decision state carried from one reading to the next.  Only the
 * obstacle task may change it; read b_ahead and b_fault for the result. */
typedef struct
{
    bool     b_ahead;       /* Obstacle ahead; held until acknowledged. */
    bool     b_fault;       /* SENSOR_FAULT: no echo for too long.      */
    bool     b_blind;       /* Last valid reading was in the blind zone. */
    uint32_t close_count;   /* Consecutive readings under STOP distance. */
    uint32_t lost_count;    /* Consecutive readings that lost near_mm.  */
    uint32_t near_mm;       /* Last near obstacle (0 = none remembered). */
    uint32_t silent_count;  /* Consecutive NO_ECHO / NOT_READY readings. */
    uint32_t alive_count;   /* Readings with an echo while in fault.     */
} obstacle_monitor_t;

/* Clear all decision state (start-up). */
void obstacle_monitor_reset(obstacle_monitor_t * p_mon);

/* Apply one reading.  p_report receives the distance, status and state
 * to publish; its seq field is left for the caller. */
void obstacle_monitor_step(obstacle_monitor_t *     p_mon,
                           obstacle_range_t const * p_range,
                           obstacle_status_t *      p_report);

/* The controller has acted on OBSTACLE_AHEAD: clear it and forget the
 * remembered obstacle.  It is raised again if the obstacle is still there. */
void obstacle_monitor_ack(obstacle_monitor_t * p_mon);

#endif /* OBSTACLE_MONITOR_H */

/*** end of file ***/
