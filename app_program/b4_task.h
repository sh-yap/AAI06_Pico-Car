/** @file b4_task.h
 *
 * @brief Buddy 4 production task and its 32-bit terrain word.
 *
 * The task samples the accelerometer at 100 Hz, runs b4_proc, and sends
 * the Vehicle Controller one terrain word every second sample, 50 Hz. It
 * is the only writer of Buddy 4 state. The word follows the Phase C shared
 * data definitions, section 6:
 *
 *   bits  0-1   health, as b4_types_health_t
 *   bits  2-3   turn, as b4_types_turn_t
 *   bits  4-13  current hump height, 0.1 mm steps, 0 to 800
 *   bits 14-23  highest valid hump height this run, 0.1 mm steps
 *   bit  24     height valid
 *   bits 25-28  hump count, wraps
 *   bits 29-31  impact count, wraps
 *
 * NOTE: The layout has no ramp-active bit, so its meaning is carried by
 * the existing fields. While a hump is in progress the current height is
 * the live estimate, at least 0.1 mm, with the valid bit clear. After a
 * hump completes it is the final height with the valid bit set, or zero
 * with the bit clear when the height could not be measured. Freshness is
 * not in the word; the receiver times each word it gets.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_TASK_H
#define B4_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "b4_types.h"

/** @brief One decoded terrain word.
 */
typedef struct
{
    b4_types_health_t health;
    b4_types_turn_t   turn;
    uint16_t          current_height_dmm;
    uint16_t          highest_height_dmm;
    bool              b_height_valid;
    uint8_t           hump_count;
    uint8_t           impact_count;
} b4_task_terrain_t;

/** @brief Create the word buffer and request flag, then start the task.
 *
 * A missing IMU does not fail the start. The task keeps retrying the IMU
 * and reports FAULT health until it opens.
 *
 * @return True when the task is running.
 */
bool
b4_task_start (void);

/** @brief Take the oldest unread terrain word without waiting.
 *
 * @param p_word The destination for the word.
 * @return True when a word was read.
 */
bool
b4_task_receive_word (uint32_t * p_word);

/** @brief Decode a terrain word.
 *
 * @param word The packed word.
 * @param p_terrain The destination for the decoded fields.
 */
void
b4_task_unpack (uint32_t word, b4_task_terrain_t * p_terrain);

/** @brief Report whether a decoded word shows a hump in progress.
 *
 * @param p_terrain The decoded word.
 * @return True when the valid bit is clear and the height is above zero.
 */
bool
b4_task_is_ramp_active (b4_task_terrain_t const * p_terrain);

/** @brief Ask the task to clear the run maximum before the next sample.
 */
void
b4_task_request_run_reset (void);

#endif /* B4_TASK_H */
