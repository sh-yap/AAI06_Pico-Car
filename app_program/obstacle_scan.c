/** @file obstacle_scan.c
 *
 * @brief Buddy 5 scan analysis: obstacle profile and recommendation.
 *
 * Methods (lecturer's guidance, 2026-10-31):
 *   - repeated, median-filtered pings at each angle after the servo
 *     settles (obstacle_scan_reduce());
 *   - background subtraction: readings well beyond the closest point are
 *     background, not obstacle;
 *   - consecutive-angle detection with hysteresis to find the span;
 *   - conservative margins: unknown angles never count as clear, the
 *     width is the outer bound, and the car's corridor includes a margin.
 *
 * Angles: 90 = straight ahead, above 90 = left, below 90 = right.
 *
 * Each side of the obstacle span is PROVEN clear (background seen beyond
 * it), UNPROVEN (the span reaches the end of the view, so nothing is
 * known) or BLOCKED (a second object beyond it).  The sensor hears a
 * close obstacle 20-30 deg or more off-axis (bench tests 2026-10-31 and
 * 2026-11-01), so with the +-30 deg servo a side is often UNPROVEN.
 *
 * Decision order: sensor fault -> STOP; corridor clear -> CONTINUE;
 * nothing measured -> REVERSE_RETRY; then the side.  With
 * OBSTACLE_PREFER_LEFT / _RIGHT: that side unless it is BLOCKED, then the
 * other (the sensor cannot see the course edge, so the course decides).
 * With OBSTACLE_PREFER_SCAN: a PROVEN side -> turn that way; an UNPROVEN
 * side -> turn that way, right first (after a right pivot the path check
 * sees past the obstacle 5 deg sooner).  Both BLOCKED -> REVERSE_RETRY.
 *
 * PATH CHECK, after the controller's first bypass pivot: readings at
 * OBSTACLE_PATH_MAX_POINTS angles from the bypass side back to straight
 * ahead.  The obstacle that was scanned is still heard at most of them,
 * so a reading near its distance counts as that obstacle, not as a new
 * one.  The outermost angle must see past it.  Blind spot: a second
 * object at the same distance as the first is taken for the first.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"
#include "obstacle_scan.h"

#define SCAN_AHEAD_DEG       (90U)
#define SCAN_SIN_STEP_DEG    (5U)
#define SCAN_MILLI           (1000)
#define SCAN_PINGS_MAX       (8U)
#define SCAN_CORRIDOR_DEG    (15U)    /* Angles that look along the path. */
#define SCAN_NONE            (0xFFFFFFFFU)

/* What is known about one side of the obstacle span. */
typedef enum
{
    SCAN_SIDE_BLOCKED = 0,     /* A second object beyond the span.        */
    SCAN_SIDE_UNPROVEN,        /* The span reaches the end of the view.   */
    SCAN_SIDE_PROVEN           /* Only background beyond the span.        */
} scan_side_t;

#if (OBSTACLE_SCAN_COUNT > OBSTACLE_SCAN_MAX_POINTS)
#error "Too many scan angles for obstacle_scan_result_t"
#endif

/* Which way to go round (OBSTACLE_PREFER_*); see obstacle_scan_set_prefer(). */
static uint32_t scan_prefer = OBSTACLE_BYPASS_PREFER;

/* sin(0, 5, ..., 90 deg) x 1000. */
static int32_t const sin_milli[] =
{
    0, 87, 174, 259, 342, 423, 500, 574, 643, 707,
    766, 819, 866, 906, 940, 966, 985, 996, 1000
};


/*!
 * @brief Angle away from straight ahead, 0..90.
 *
 * @param[in] angle_deg Scan angle.
 *
 * @return |angle_deg - 90|.
 */
static uint32_t
scan_off_deg(uint32_t angle_deg)
{
    uint32_t off_deg;

    if (angle_deg >= SCAN_AHEAD_DEG)
    {
        off_deg = angle_deg - SCAN_AHEAD_DEG;
    }
    else
    {
        off_deg = SCAN_AHEAD_DEG - angle_deg;
    }

    if (off_deg > SCAN_AHEAD_DEG)
    {
        off_deg = SCAN_AHEAD_DEG;
    }

    return (off_deg);
}

/*!
 * @brief sin() x 1000 of an angle, to the nearest 5 degrees.
 *
 * @param[in] off_deg Angle, 0..90.
 *
 * @return sin(off_deg) x 1000.
 */
static int32_t
scan_sin(uint32_t off_deg)
{
    return (sin_milli[(off_deg + (SCAN_SIN_STEP_DEG / 2U))
                      / SCAN_SIN_STEP_DEG]);
}

/*!
 * @brief Sideways position of a point at angle_deg, distance_mm.
 *
 * @param[in] angle_deg   Scan angle.
 * @param[in] distance_mm Distance along the beam.
 *
 * @return Millimetres from the car's centre line, left positive.
 */
static int32_t
scan_lateral_mm(uint32_t angle_deg, uint32_t distance_mm)
{
    /* Cast to int32_t: distances are at most 4095 mm, so the product
     * with 1000 fits easily. */
    int32_t lateral = ((int32_t)distance_mm
                       * scan_sin(scan_off_deg(angle_deg))) / SCAN_MILLI;

    if (angle_deg < SCAN_AHEAD_DEG)
    {
        lateral = -lateral;
    }

    return (lateral);
}

/*!
 * @brief Forward position of a point at angle_deg, distance_mm.
 *
 * @param[in] angle_deg   Scan angle.
 * @param[in] distance_mm Distance along the beam.
 *
 * @return Millimetres ahead of the sensor.
 */
static uint32_t
scan_forward_mm(uint32_t angle_deg, uint32_t distance_mm)
{
    /* Casts: cos(off) = sin(90 - off) is 0..1000 and distance at most
     * 4095, so the result is non-negative and fits in uint32_t. */
    return ((uint32_t)(((int32_t)distance_mm
                        * scan_sin(SCAN_AHEAD_DEG - scan_off_deg(angle_deg)))
                       / SCAN_MILLI));
}

/*!
 * @brief True if the point stops the car driving straight on.
 *
 * Unknown points count as blocking when they look along the path, because
 * an angled face gives no echo.
 *
 * @param[in] p_point Point to test.
 *
 * @return true if it blocks the car's corridor.
 */
static bool
scan_blocks_corridor(obstacle_scan_point_t const * p_point)
{
    bool    b_blocks = false;
    int32_t lateral;

    if (0U == p_point->distance_mm)
    {
        b_blocks = (scan_off_deg(p_point->angle_deg) <= SCAN_CORRIDOR_DEG);
    }
    else
    {
        lateral = scan_lateral_mm(p_point->angle_deg, p_point->distance_mm);

        if (lateral < 0)
        {
            lateral = -lateral;
        }

        /* Cast to uint32_t: lateral was just made non-negative. */
        b_blocks = (((uint32_t)lateral < (OBSTACLE_CAR_HALF_WIDTH_MM
                                          + OBSTACLE_SAFETY_MARGIN_MM)) &&
                    (scan_forward_mm(p_point->angle_deg,
                                     p_point->distance_mm)
                     < OBSTACLE_SCAN_CLEAR_MM));
    }

    return (b_blocks);
}

/*!
 * @brief What is known about one side, beyond the obstacle span.
 *
 * @param[in] p_result Scan with classified points.
 * @param[in] first    First index beyond the span on that side; count
 *                     (or more) if the span reaches the end of the view.
 * @param[in] b_up     true to walk up (left), false to walk down (right).
 *
 * @return PROVEN if every point there is measured background, UNPROVEN
 *         if there are none, BLOCKED if any is not background.
 */
static scan_side_t
scan_side_state(obstacle_scan_result_t const * p_result,
                uint32_t                       first,
                bool                           b_up)
{
    bool        b_clear = true;
    scan_side_t state = SCAN_SIDE_UNPROVEN;
    uint32_t    idx = first;

    if (first < p_result->count)
    {
        state = SCAN_SIDE_PROVEN;
    }

    while (b_clear && (idx < p_result->count))
    {
        b_clear = (OBSTACLE_POINT_BACKGROUND
                   == p_result->points[idx].point_class);

        if (!b_clear)
        {
            state = SCAN_SIDE_BLOCKED;
        }

        if (b_up)
        {
            idx++;
        }
        else if (0U == idx)
        {
            idx = p_result->count;   /* Walked off the right end. */
        }
        else
        {
            idx--;
        }
    }

    return (state);
}

/*!
 * @brief The preferred side unless the scan sees it blocked.
 *
 * @param[in] first         What is known about the preferred side.
 * @param[in] first_action  Turn towards the preferred side.
 * @param[in] second        What is known about the other side.
 * @param[in] second_action Turn towards the other side.
 *
 * @return first_action, else second_action, else REVERSE_RETRY.  An
 *         unproven side counts as free: the path check verifies it after
 *         the pivot.
 */
static obstacle_action_t
scan_preferred(scan_side_t       first,
               obstacle_action_t first_action,
               scan_side_t       second,
               obstacle_action_t second_action)
{
    obstacle_action_t action = OBSTACLE_ACTION_REVERSE_RETRY;

    if (SCAN_SIDE_BLOCKED != first)
    {
        action = first_action;
    }
    else if (SCAN_SIDE_BLOCKED != second)
    {
        action = second_action;
    }
    else
    {
        /* Both blocked: back off and scan again. */
    }

    return (action);
}

/*!
 * @brief Choose how the bypass side is picked.
 *
 * @param[in] prefer OBSTACLE_PREFER_SCAN, _LEFT or _RIGHT; anything else
 *                   is ignored.
 */
void
obstacle_scan_set_prefer(uint32_t prefer)
{
    if ((OBSTACLE_PREFER_SCAN == prefer) || (OBSTACLE_PREFER_LEFT == prefer)
        || (OBSTACLE_PREFER_RIGHT == prefer))
    {
        scan_prefer = prefer;
    }
}

/*!
 * @brief Reduce the pings at one angle to a scan point.
 *
 * @param[in]  angle_deg  Scan angle.
 * @param[in]  p_pings    Readings taken at that angle.
 * @param[in]  ping_count Number of readings (at most 8 are used).
 * @param[out] p_point    Result; class UNKNOWN until analysed.
 */
void
obstacle_scan_reduce(uint32_t                 angle_deg,
                     obstacle_range_t const * p_pings,
                     uint32_t                 ping_count,
                     obstacle_scan_point_t *  p_point)
{
    uint32_t ok_mm[SCAN_PINGS_MAX];
    uint32_t used = ping_count;
    uint32_t idx;

    if ((NULL != p_pings) && (NULL != p_point))
    {
        if (used > SCAN_PINGS_MAX)
        {
            used = SCAN_PINGS_MAX;
        }

        p_point->angle_deg    = angle_deg;
        p_point->distance_mm  = 0U;
        p_point->valid_count  = 0U;
        p_point->silent_count = 0U;
        p_point->point_class  = OBSTACLE_POINT_UNKNOWN;

        for (idx = 0U; idx < used; idx++)
        {
            if (OBSTACLE_RANGE_OK == p_pings[idx].status)
            {
                ok_mm[p_point->valid_count] = p_pings[idx].distance_mm;
                p_point->valid_count++;
            }
            else if ((OBSTACLE_RANGE_NO_ECHO == p_pings[idx].status) ||
                     (OBSTACLE_RANGE_NOT_READY == p_pings[idx].status))
            {
                p_point->silent_count++;
            }
            else
            {
                /* TIMEOUT, TOO_FAR, TOO_CLOSE: answered, no distance. */
            }
        }

        if (p_point->valid_count >= OBSTACLE_SCAN_MIN_VALID)
        {
            p_point->distance_mm = obstacle_calc_median(ok_mm,
                                                        p_point->valid_count);
        }
    }
}

/*!
 * @brief Classify a scan and choose what the controller should do.
 *
 * @param[in,out] p_result Points in, profile and edges out.
 */
void
obstacle_scan_analyse(obstacle_scan_result_t * p_result)
{
    obstacle_profile_t * p_prof;
    uint32_t             idx;
    uint32_t             silent_angles = 0U;
    uint32_t             closest = SCAN_NONE;
    uint32_t             closest_mm = 0U;
    uint32_t             low;
    uint32_t             high;
    bool                 b_path_clear = true;
    scan_side_t          left;
    scan_side_t          right;
    int32_t              centre;

    if ((NULL != p_result) && (p_result->count <= OBSTACLE_SCAN_MAX_POINTS))
    {
        p_prof = &p_result->profile;
        p_prof->distance_mm    = 0U;
        p_prof->angle_deg      = SCAN_AHEAD_DEG;
        p_prof->width_mm       = 0U;
        p_prof->clear_left_mm  = 0U;
        p_prof->clear_right_mm = 0U;
        p_prof->action         = OBSTACLE_ACTION_REVERSE_RETRY;
        p_result->right_edge_deg = 0U;
        p_result->left_edge_deg  = 0U;

        for (idx = 0U; idx < p_result->count; idx++)
        {
            obstacle_scan_point_t const * p_pt = &p_result->points[idx];

            if (p_pt->silent_count >= OBSTACLE_SCAN_MIN_VALID)
            {
                silent_angles++;
            }

            if (scan_blocks_corridor(p_pt))
            {
                b_path_clear = false;
            }

            if ((0U != p_pt->distance_mm) &&
                ((SCAN_NONE == closest) || (p_pt->distance_mm < closest_mm)))
            {
                closest = idx;
                closest_mm = p_pt->distance_mm;
            }
        }

        /* Background subtraction: near the closest point = obstacle. */
        for (idx = 0U; idx < p_result->count; idx++)
        {
            obstacle_scan_point_t * p_pt = &p_result->points[idx];

            if (0U == p_pt->distance_mm)
            {
                p_pt->point_class = OBSTACLE_POINT_UNKNOWN;
            }
            else if (p_pt->distance_mm
                     <= (closest_mm + OBSTACLE_SCAN_SPAN_TOL_MM))
            {
                p_pt->point_class = OBSTACLE_POINT_OBSTACLE;
            }
            else
            {
                p_pt->point_class = OBSTACLE_POINT_BACKGROUND;
            }
        }

        if (SCAN_NONE != closest)
        {
            /* Report the closest point whatever the decision. */
            p_prof->distance_mm = closest_mm;
            p_prof->angle_deg   = p_result->points[closest].angle_deg;
        }

        if (silent_angles >= OBSTACLE_SCAN_FAULT_ANGLES)
        {
            p_prof->action = OBSTACLE_ACTION_STOP;
        }
        else if (b_path_clear)
        {
            p_prof->action = OBSTACLE_ACTION_CONTINUE;
        }
        else if (SCAN_NONE == closest)
        {
            /* Nothing measured anywhere: cannot prove any way is clear. */
            p_prof->action = OBSTACLE_ACTION_REVERSE_RETRY;
        }
        else
        {
            /* Grow the span from the closest point with hysteresis. */
            low = closest;
            while ((low > 0U) &&
                   ((0U == p_result->points[low - 1U].distance_mm) ||
                    (p_result->points[low - 1U].distance_mm
                     <= (closest_mm + OBSTACLE_SCAN_SPAN_HOLD_MM))))
            {
                low--;
                p_result->points[low].point_class = OBSTACLE_POINT_OBSTACLE;
            }

            high = closest;
            while (((high + 1U) < p_result->count) &&
                   ((0U == p_result->points[high + 1U].distance_mm) ||
                    (p_result->points[high + 1U].distance_mm
                     <= (closest_mm + OBSTACLE_SCAN_SPAN_HOLD_MM))))
            {
                high++;
                p_result->points[high].point_class = OBSTACLE_POINT_OBSTACLE;
            }

            p_result->right_edge_deg = p_result->points[low].angle_deg;
            p_result->left_edge_deg  = p_result->points[high].angle_deg;

            /* Width, conservative: between the first background angles
             * on each side; wider than the view if either is missing. */
            if ((low > 0U) && ((high + 1U) < p_result->count))
            {
                /* Cast to uint32_t: left minus right is positive. */
                p_prof->width_mm = (uint32_t)(
                    scan_lateral_mm(p_result->points[high + 1U].angle_deg,
                                    closest_mm)
                    - scan_lateral_mm(p_result->points[low - 1U].angle_deg,
                                      closest_mm));
            }
            else
            {
                p_prof->width_mm = 1023U;
            }

            left = scan_side_state(p_result, high + 1U, true);
            right = (low > 0U) ?
                    scan_side_state(p_result, low - 1U, false) :
                    SCAN_SIDE_UNPROVEN;

            if (SCAN_SIDE_PROVEN == left)
            {
                p_prof->clear_left_mm =
                    p_result->points[p_result->count - 1U].distance_mm;
            }

            if (SCAN_SIDE_PROVEN == right)
            {
                p_prof->clear_right_mm = p_result->points[0].distance_mm;
            }

            /* Both sides clear: go away from the obstacle's centre. */
            centre = scan_lateral_mm(p_result->points[low].angle_deg,
                                     closest_mm)
                     + scan_lateral_mm(p_result->points[high].angle_deg,
                                       closest_mm);

            if (OBSTACLE_PREFER_LEFT == scan_prefer)
            {
                p_prof->action = scan_preferred(left, OBSTACLE_ACTION_TURN_LEFT,
                                                right,
                                                OBSTACLE_ACTION_TURN_RIGHT);
            }
            else if (OBSTACLE_PREFER_RIGHT == scan_prefer)
            {
                p_prof->action = scan_preferred(right,
                                                OBSTACLE_ACTION_TURN_RIGHT,
                                                left, OBSTACLE_ACTION_TURN_LEFT);
            }
            else if ((SCAN_SIDE_PROVEN == left) && (SCAN_SIDE_PROVEN == right))
            {
                if (centre > 0)
                {
                    p_prof->action = OBSTACLE_ACTION_TURN_RIGHT;
                }
                else if (centre < 0)
                {
                    p_prof->action = OBSTACLE_ACTION_TURN_LEFT;
                }
                else if (p_prof->clear_right_mm > p_prof->clear_left_mm)
                {
                    p_prof->action = OBSTACLE_ACTION_TURN_RIGHT;
                }
                else
                {
                    p_prof->action = OBSTACLE_ACTION_TURN_LEFT;
                }
            }
            else if (SCAN_SIDE_PROVEN == left)
            {
                p_prof->action = OBSTACLE_ACTION_TURN_LEFT;
            }
            else if (SCAN_SIDE_PROVEN == right)
            {
                p_prof->action = OBSTACLE_ACTION_TURN_RIGHT;
            }
            else if (SCAN_SIDE_UNPROVEN == right)
            {
                /* To be verified by the path check after the pivot. */
                p_prof->action = OBSTACLE_ACTION_TURN_RIGHT;
            }
            else if (SCAN_SIDE_UNPROVEN == left)
            {
                p_prof->action = OBSTACLE_ACTION_TURN_LEFT;
            }
            else
            {
                p_prof->action = OBSTACLE_ACTION_REVERSE_RETRY;
            }
        }
    }
}

/*!
 * @brief What one measured distance means for a path check.
 *
 * @param[in] distance_mm Measured distance (not 0).
 * @param[in] p_path      Path check holding need_mm and known_mm.
 *
 * @return CLEAR if at least need_mm, KNOWN if it is the scanned obstacle,
 *         otherwise BLOCKED.
 */
static obstacle_path_verdict_t
scan_path_judge(uint32_t distance_mm, obstacle_path_t const * p_path)
{
    obstacle_path_verdict_t verdict = OBSTACLE_PATH_BLOCKED;

    if (distance_mm >= p_path->need_mm)
    {
        verdict = OBSTACLE_PATH_CLEAR;
    }
    else if ((0U != p_path->known_mm) &&
             ((distance_mm + OBSTACLE_PATH_KNOWN_BELOW_MM) >= p_path->known_mm)
             && (distance_mm <= (p_path->known_mm
                                 + OBSTACLE_PATH_KNOWN_ABOVE_MM)))
    {
        verdict = OBSTACLE_PATH_KNOWN;
    }
    else
    {
        /* Something else, nearer than need_mm. */
    }

    return (verdict);
}

/*!
 * @brief Servo angle for one step of a path check.
 *
 * @param[in] side TURN_RIGHT or TURN_LEFT; anything else looks straight
 *                 ahead.
 * @param[in] idx  0 = outermost .. OBSTACLE_PATH_MAX_POINTS - 1 = ahead.
 *
 * @return Servo angle in degrees.
 */
uint32_t
obstacle_scan_path_angle(obstacle_action_t side, uint32_t idx)
{
    uint32_t angle_deg = SCAN_AHEAD_DEG;
    uint32_t step = idx;

    if (step >= OBSTACLE_PATH_MAX_POINTS)
    {
        step = OBSTACLE_PATH_MAX_POINTS - 1U;
    }

    if (OBSTACLE_ACTION_TURN_RIGHT == side)
    {
        angle_deg = OBSTACLE_PATH_RIGHT_OUTER_DEG
                    + (((SCAN_AHEAD_DEG - OBSTACLE_PATH_RIGHT_OUTER_DEG) * step)
                       / (OBSTACLE_PATH_MAX_POINTS - 1U));
    }
    else if (OBSTACLE_ACTION_TURN_LEFT == side)
    {
        angle_deg = OBSTACLE_PATH_LEFT_OUTER_DEG
                    - (((OBSTACLE_PATH_LEFT_OUTER_DEG - SCAN_AHEAD_DEG) * step)
                       / (OBSTACLE_PATH_MAX_POINTS - 1U));
    }
    else
    {
        /* Not a bypass: straight ahead. */
    }

    return (angle_deg);
}

/*!
 * @brief Judge the pings at one path-check angle and append the result.
 *
 * @param[in,out] p_path     need_mm and known_mm in; points[count] out.
 * @param[in]     angle_deg  Servo angle of the readings.
 * @param[in]     p_pings    Readings taken at that angle.
 * @param[in]     ping_count Number of readings (at most 8 are used).
 */
void
obstacle_scan_path_add(obstacle_path_t *        p_path,
                       uint32_t                 angle_deg,
                       obstacle_range_t const * p_pings,
                       uint32_t                 ping_count)
{
    obstacle_scan_point_t   point;
    obstacle_path_point_t * p_out;
    obstacle_path_verdict_t single;
    uint32_t                idx;
    uint32_t                used = ping_count;
    bool                    b_too_close = false;

    if ((NULL != p_path) && (NULL != p_pings) &&
        (p_path->count < OBSTACLE_PATH_MAX_POINTS))
    {
        if (used > SCAN_PINGS_MAX)
        {
            used = SCAN_PINGS_MAX;
        }

        for (idx = 0U; idx < used; idx++)
        {
            if (OBSTACLE_RANGE_TOO_CLOSE == p_pings[idx].status)
            {
                b_too_close = true;
            }
        }

        obstacle_scan_reduce(angle_deg, p_pings, used, &point);
        p_out = &p_path->points[p_path->count];
        p_out->angle_deg   = angle_deg;
        p_out->distance_mm = point.distance_mm;

        if (b_too_close)
        {
            p_out->verdict = OBSTACLE_PATH_BLOCKED;
        }
        else if (0U != point.distance_mm)
        {
            p_out->verdict = scan_path_judge(point.distance_mm, p_path);
        }
        else if (point.silent_count >= OBSTACLE_SCAN_MIN_VALID)
        {
            p_out->verdict = OBSTACLE_PATH_FAULT;
        }
        else
        {
            /* At most one reading.  Trust it if it is near; a single far
             * reading is not enough to call the way clear. */
            p_out->verdict = OBSTACLE_PATH_OPEN;

            for (idx = 0U; idx < used; idx++)
            {
                if (OBSTACLE_RANGE_OK == p_pings[idx].status)
                {
                    single = scan_path_judge(p_pings[idx].distance_mm, p_path);

                    if (OBSTACLE_PATH_CLEAR != single)
                    {
                        p_out->verdict = single;
                    }
                }
            }
        }

        p_path->count++;
    }
}

/*!
 * @brief Combine the path-check angles into one verdict.
 *
 * BLOCKED if any angle saw something new, FAULT if any was silent,
 * BLOCKED if the outermost angle still heard the known obstacle (nothing
 * beside it was seen), OPEN if any angle had no echo, otherwise CLEAR.
 *
 * @param[in,out] p_path Points in (outermost first), verdict out.
 */
void
obstacle_scan_path_verdict(obstacle_path_t * p_path)
{
    bool     b_blocked = false;
    bool     b_fault = false;
    bool     b_open = false;
    uint32_t idx;

    if (NULL != p_path)
    {
        for (idx = 0U; (idx < p_path->count) &&
                       (idx < OBSTACLE_PATH_MAX_POINTS); idx++)
        {
            switch (p_path->points[idx].verdict)
            {
                case OBSTACLE_PATH_BLOCKED:
                    b_blocked = true;
                    break;

                case OBSTACLE_PATH_FAULT:
                    b_fault = true;
                    break;

                case OBSTACLE_PATH_OPEN:
                    b_open = true;
                    break;

                default:
                    /* CLEAR or KNOWN: no objection. */
                    break;
            }
        }

        if (0U == p_path->count)
        {
            p_path->verdict = OBSTACLE_PATH_FAULT;
        }
        else if (b_blocked)
        {
            p_path->verdict = OBSTACLE_PATH_BLOCKED;
        }
        else if (b_fault)
        {
            p_path->verdict = OBSTACLE_PATH_FAULT;
        }
        else if (OBSTACLE_PATH_KNOWN == p_path->points[0].verdict)
        {
            p_path->verdict = OBSTACLE_PATH_BLOCKED;
        }
        else if (b_open)
        {
            p_path->verdict = OBSTACLE_PATH_OPEN;
        }
        else
        {
            p_path->verdict = OBSTACLE_PATH_CLEAR;
        }
    }
}

/*** end of file ***/
