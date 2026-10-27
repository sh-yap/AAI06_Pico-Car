/** @file obstacle_monitor.c
 *
 * @brief Buddy 5 MONITOR-state decisions: obstacle ahead, sensor fault.
 *
 * Rules, all from the Week 6 statechart or the 2026-10-25 bench tests:
 *   - OBSTACLE_AHEAD after OBSTACLE_STOP_READINGS consecutive valid
 *     readings under OBSTACLE_STOP_DISTANCE_MM (FR-B5-03).
 *   - Blind-zone latch: after a valid reading under
 *     OBSTACLE_BLIND_LATCH_MM, a TIMEOUT means "still blocked" and is
 *     reported as TOO_CLOSE with distance 0.
 *   - Lost-obstacle rule: an obstacle seen nearer than
 *     OBSTACLE_LOST_NEAR_MM that suddenly reads TIMEOUT or much further
 *     for OBSTACLE_LOST_READINGS readings raises OBSTACLE_AHEAD.
 *   - TIMEOUT is never proof that the way is clear: it never clears
 *     OBSTACLE_AHEAD.  Only the controller's acknowledgement does.
 *   - SENSOR_FAULT after OBSTACLE_FAULT_READINGS readings with no echo;
 *     cleared after OBSTACLE_RECOVER_READINGS readings with an echo.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_monitor.h"


/*!
 * @brief Add one to a counter without passing a limit.
 *
 * @param[in,out] p_count Counter.
 * @param[in]     limit   Value at which counting stops.
 *
 * @return true once the counter has reached limit.
 */
static bool
monitor_count_to(uint32_t * p_count, uint32_t limit)
{
    if (*p_count < limit)
    {
        (*p_count)++;
    }

    return (*p_count >= limit);
}

/*!
 * @brief The sensor answered (any reading with an echo).
 *
 * @param[in,out] p_mon Monitor state.
 */
static void
monitor_note_alive(obstacle_monitor_t * p_mon)
{
    p_mon->silent_count = 0U;

    if (p_mon->b_fault)
    {
        if (monitor_count_to(&p_mon->alive_count, OBSTACLE_RECOVER_READINGS))
        {
            p_mon->b_fault = false;
            p_mon->alive_count = 0U;
        }
    }
}

/*!
 * @brief The sensor did not answer (NO_ECHO or NOT_READY).
 *
 * @param[in,out] p_mon Monitor state.
 */
static void
monitor_note_silent(obstacle_monitor_t * p_mon)
{
    p_mon->alive_count = 0U;
    p_mon->close_count = 0U;

    if (monitor_count_to(&p_mon->silent_count, OBSTACLE_FAULT_READINGS))
    {
        p_mon->b_fault = true;
    }
}

/*!
 * @brief One more reading that has lost the remembered near obstacle.
 *
 * @param[in,out] p_mon Monitor state.
 */
static void
monitor_note_lost(obstacle_monitor_t * p_mon)
{
    if (monitor_count_to(&p_mon->lost_count, OBSTACLE_LOST_READINGS))
    {
        p_mon->b_ahead = true;
    }
}

/*!
 * @brief Apply a valid (OK) distance.
 *
 * @param[in,out] p_mon       Monitor state.
 * @param[in]     distance_mm Measured distance.
 */
static void
monitor_on_distance(obstacle_monitor_t * p_mon, uint32_t distance_mm)
{
    p_mon->b_blind = (distance_mm < OBSTACLE_BLIND_LATCH_MM);

    if (distance_mm < OBSTACLE_STOP_DISTANCE_MM)
    {
        if (monitor_count_to(&p_mon->close_count, OBSTACLE_STOP_READINGS))
        {
            p_mon->b_ahead = true;
        }
    }
    else
    {
        p_mon->close_count = 0U;
    }

    if ((0U != p_mon->near_mm) &&
        (distance_mm > (p_mon->near_mm + OBSTACLE_LOST_JUMP_MM)))
    {
        /* Jumped far beyond the near obstacle: lost it, or a stray echo.
         * Keep near_mm; one stray reading is not enough to act on. */
        monitor_note_lost(p_mon);
    }
    else
    {
        p_mon->lost_count = 0U;
        p_mon->near_mm = (distance_mm < OBSTACLE_LOST_NEAR_MM) ?
                         distance_mm : 0U;
    }
}

/*!
 * @brief Apply a reading with an echo but no distance.
 *
 * @param[in,out] p_mon    Monitor state.
 * @param[in]     status   TIMEOUT, TOO_FAR or TOO_CLOSE.
 * @param[out]    p_report Changed to TOO_CLOSE when the blind latch holds.
 */
static void
monitor_on_no_distance(obstacle_monitor_t *    p_mon,
                       obstacle_range_status_t status,
                       obstacle_status_t *     p_report)
{
    p_mon->close_count = 0U;

    if (p_mon->b_blind || (OBSTACLE_RANGE_TOO_CLOSE == status))
    {
        /* Too close to measure: still blocked. */
        p_mon->b_blind = true;
        p_mon->b_ahead = true;
        p_report->distance_mm = 0U;
        p_report->range_status = OBSTACLE_RANGE_TOO_CLOSE;
    }
    else if (0U != p_mon->near_mm)
    {
        monitor_note_lost(p_mon);
    }
    else
    {
        /* Nothing near was seen: this reading says nothing either way. */
    }
}

/*!
 * @brief Clear all decision state.
 *
 * @param[out] p_mon Monitor state.
 */
void
obstacle_monitor_reset(obstacle_monitor_t * p_mon)
{
    if (NULL != p_mon)
    {
        p_mon->b_ahead      = false;
        p_mon->b_fault      = false;
        p_mon->b_blind      = false;
        p_mon->close_count  = 0U;
        p_mon->lost_count   = 0U;
        p_mon->near_mm      = 0U;
        p_mon->silent_count = 0U;
        p_mon->alive_count  = 0U;
    }
}

/*!
 * @brief Apply one ultrasonic reading.
 *
 * @param[in,out] p_mon    Monitor state.
 * @param[in]     p_range  The reading.
 * @param[out]    p_report Distance, status and state to publish (seq is
 *                         not touched).
 */
void
obstacle_monitor_step(obstacle_monitor_t *     p_mon,
                      obstacle_range_t const * p_range,
                      obstacle_status_t *      p_report)
{
    if ((NULL != p_mon) && (NULL != p_range) && (NULL != p_report))
    {
        p_report->distance_mm  = p_range->distance_mm;
        p_report->range_status = p_range->status;

        switch (p_range->status)
        {
            case OBSTACLE_RANGE_OK:
                monitor_note_alive(p_mon);
                monitor_on_distance(p_mon, p_range->distance_mm);
                break;

            case OBSTACLE_RANGE_TIMEOUT:
            case OBSTACLE_RANGE_TOO_FAR:
            case OBSTACLE_RANGE_TOO_CLOSE:
                monitor_note_alive(p_mon);
                monitor_on_no_distance(p_mon, p_range->status, p_report);
                break;

            default:
                /* NO_ECHO or NOT_READY: the sensor did not answer. */
                monitor_note_silent(p_mon);
                break;
        }

        p_report->state = p_mon->b_fault ? OBSTACLE_STATE_SENSOR_FAULT :
                                           OBSTACLE_STATE_MONITOR;
    }
}

/*!
 * @brief The controller has acted on OBSTACLE_AHEAD.
 *
 * The blind latch is kept: if the sensor is still blinded, the next
 * TIMEOUT raises OBSTACLE_AHEAD again at once.
 *
 * @param[in,out] p_mon Monitor state.
 */
void
obstacle_monitor_ack(obstacle_monitor_t * p_mon)
{
    if (NULL != p_mon)
    {
        p_mon->b_ahead     = false;
        p_mon->close_count = 0U;
        p_mon->lost_count  = 0U;
        p_mon->near_mm     = 0U;
    }
}

/*** end of file ***/
