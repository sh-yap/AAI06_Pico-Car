/** @file obstacle_plan.c
 *
 * @brief Buddy 5 bypass planning: the "^" path around an obstacle.
 *
 * Coordinates: x forward along the line, y sideways towards the bypass
 * side, origin at the rear axle (the pivot point) where the car stopped.
 * The obstacle is assumed to be the brief's largest (OBSTACLE_MAX_WIDTH_MM
 * square) centred on the line, its near face distance_mm ahead of the
 * sensor.  The lecturer's guidance allows planning with the maximum width
 * instead of measuring it exactly.
 *
 * Path: pivot theta, drive leg 1 to the apex, pivot -2 theta, drive leg 2
 * back to the line, pivot theta.
 *   - theta is the smallest angle at which the car's inner side clears the
 *     obstacle's near corner by OBSTACLE_PLAN_MARGIN_MM, measured square to
 *     the car's side:
 *         x_near * sin(theta) - half_obstacle * cos(theta) - half_width
 *                                                          >= margin
 *     The inner side is one straight line, so clearing the near corner
 *     clears the whole obstacle on leg 1; leg 2 is further out (it starts
 *     beyond the obstacle), so it clears too.
 *   - The apex is far enough past the obstacle's back face that the rear
 *     corners, which swing round the axle at the apex pivot, cannot reach
 *     it.  The front corners stay ahead of the axle for pivots up to 2 x
 *     OBSTACLE_PLAN_MAX_DEG, so they cannot either.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_plan.h"

#define PLAN_UNIT           (10000U)  /* sin_tenk[] scale.             */
#define PLAN_MIN_DEG        (5U)
#define PLAN_HALF_CAR_MM    (OBSTACLE_CAR_HALF_WIDTH_MM)

#if (OBSTACLE_PLAN_MAX_DEG > 45U)
#error "OBSTACLE_PLAN_MAX_DEG above 45 breaks the front-corner argument"
#endif

/* sin(0..90 deg) x 10000, 1 degree steps. */
static uint32_t const sin_tenk[] =
{
    0, 175, 349, 523, 698, 872, 1045, 1219, 1392, 1564,
    1736, 1908, 2079, 2250, 2419, 2588, 2756, 2924, 3090, 3256,
    3420, 3584, 3746, 3907, 4067, 4226, 4384, 4540, 4695, 4848,
    5000, 5150, 5299, 5446, 5592, 5736, 5878, 6018, 6157, 6293,
    6428, 6561, 6691, 6820, 6947, 7071, 7193, 7314, 7431, 7547,
    7660, 7771, 7880, 7986, 8090, 8192, 8290, 8387, 8480, 8572,
    8660, 8746, 8829, 8910, 8988, 9063, 9135, 9205, 9272, 9336,
    9397, 9455, 9511, 9563, 9613, 9659, 9703, 9744, 9781, 9816,
    9848, 9877, 9903, 9925, 9945, 9962, 9976, 9986, 9994, 9998,
    10000
};


/*!
 * @brief Integer square root, rounded up.
 *
 * @param[in] value Number to take the root of (small: a few 10^5).
 *
 * @return The smallest r with r * r >= value.
 */
static uint32_t
plan_sqrt_up(uint32_t value)
{
    uint32_t root = 0U;

    while ((root * root) < value)
    {
        root++;
    }

    return (root);
}

/*!
 * @brief Plan the "^" bypass.
 *
 * @param[in]  distance_mm Sensor to the obstacle's near face.
 * @param[in]  side        OBSTACLE_ACTION_TURN_LEFT or _TURN_RIGHT.
 * @param[out] p_plan      The plan; written only when true is returned.
 *
 * @return false for any other side, a zero distance, or if no pivot up to
 *         OBSTACLE_PLAN_MAX_DEG clears the obstacle.
 */
bool
obstacle_plan_bypass(uint32_t                 distance_mm,
                     obstacle_action_t        side,
                     obstacle_bypass_plan_t * p_plan)
{
    bool     b_found = false;
    uint32_t x_near = distance_mm + OBSTACLE_AXLE_TO_SENSOR_MM;
    uint32_t half_obstacle = OBSTACLE_MAX_WIDTH_MM / 2U;
    uint32_t theta = PLAN_MIN_DEG;
    uint32_t sin_t = 0U;
    uint32_t cos_t = PLAN_UNIT;
    uint32_t x_apex;

    if ((NULL != p_plan) && (0U != distance_mm) &&
        ((OBSTACLE_ACTION_TURN_LEFT == side) ||
         (OBSTACLE_ACTION_TURN_RIGHT == side)))
    {
        while ((!b_found) && (theta <= OBSTACLE_PLAN_MAX_DEG))
        {
            sin_t = sin_tenk[theta];
            cos_t = sin_tenk[90U - theta];

            /* Distance from the near corner to the car's inner side:
             * x_near sin(theta) - half_obstacle cos(theta) - half_car
             * >= margin, multiplied by 10000. */
            b_found = ((x_near * sin_t)
                       >= ((half_obstacle * cos_t)
                           + ((PLAN_HALF_CAR_MM + OBSTACLE_PLAN_MARGIN_MM)
                              * PLAN_UNIT)));

            if (!b_found)
            {
                theta++;
            }
        }

        if (b_found)
        {
            /* Rear axle past the back face by the rear corners' swing. */
            x_apex = x_near + OBSTACLE_MAX_WIDTH_MM + OBSTACLE_PLAN_MARGIN_MM
                     + plan_sqrt_up((OBSTACLE_REAR_OVERHANG_MM
                                     * OBSTACLE_REAR_OVERHANG_MM)
                                    + (PLAN_HALF_CAR_MM * PLAN_HALF_CAR_MM));

            p_plan->side     = side;
            p_plan->turn_deg = theta;
            p_plan->leg_mm   = ((x_apex * PLAN_UNIT) + cos_t - 1U) / cos_t;
            p_plan->apex_mm  = (x_apex * sin_t) / cos_t;
            p_plan->check_mm = p_plan->leg_mm + OBSTACLE_PLAN_CHECK_EXTRA_MM;
        }
    }

    return (b_found);
}

/*** end of file ***/
