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
 * The Vehicle Controller sends a second word the other way, once per 20 ms
 * cycle, so Buddy 4 learns its noise only while the car drives. The B4_TASK_
 * DRIVE macros below define the layout for both sides:
 *
 *   bit  0      driving, the VC is following, turning, bypassing or
 *               reacquiring the line, or handling a junction
 *   bit  1      moving, the VC's own stationary test is false
 *   bits 2-12   left wheel speed, signed 11 bits, mm/s, saturated +-1023
 *   bits 13-23  right wheel speed, same format
 *   bits 24-27  sequence, counts up each VC cycle and wraps
 *   bits 28-31  reserved, zero
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_TASK_H
#define B4_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "b4_types.h"

/* Drive word layout, shared by the Vehicle Controller (packs) and the Buddy 4
 * task (unpacks). Speeds are two's complement in 11 bits. */
#define B4_TASK_DRIVE_DRIVING_BIT   (0x00000001U)
#define B4_TASK_DRIVE_MOVING_BIT    (0x00000002U)
#define B4_TASK_DRIVE_LEFT_SHIFT    (2U)
#define B4_TASK_DRIVE_RIGHT_SHIFT   (13U)
#define B4_TASK_DRIVE_SPEED_MASK    (0x7FFU)
#define B4_TASK_DRIVE_SPEED_SIGN    (0x400U)
#define B4_TASK_DRIVE_SPEED_LIMIT   (1023)      /* mm/s, saturation.  */
#define B4_TASK_DRIVE_SEQ_SHIFT     (24U)
#define B4_TASK_DRIVE_SEQ_MASK      (0xFU)

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

/** @brief Hand Buddy 4 the latest drive word.
 *
 * Called by the Vehicle Controller task once per cycle. The word is stored
 * in one 32-bit variable with a single writer, the VC task, and a single
 * reader, the Buddy 4 task. An aligned 32-bit store is atomic on the
 * RP2040, so neither task needs a lock or can block the other.
 *
 * @param word The packed drive word, as laid out by the B4_TASK_DRIVE macros.
 */
void
b4_task_set_drive (uint32_t word);

/** @brief Ask the task to clear the run maximum before the next sample.
 */
void
b4_task_request_run_reset (void);

/** @brief START calibration steps, requested by the Vehicle Controller.
 */
typedef enum
{
    B4_TASK_CAL_LEVEL = 0,   /**< Car is still: take the level reference
                                  again.                                  */
    B4_TASK_CAL_MEASURE,     /**< Car is pivoting: measure the noise.     */
    B4_TASK_CAL_END          /**< Sweep over: seed the noise estimate.    */
} b4_task_cal_request_t;

/** @brief Where the START calibration stands. */
typedef enum
{
    B4_TASK_CAL_IDLE = 0,
    B4_TASK_CAL_MEASURING,   /**< Not enough samples yet.                 */
    B4_TASK_CAL_ENOUGH,      /**< Enough samples; the sweep may end.      */
    B4_TASK_CAL_DONE,        /**< The estimate is seeded from the sweep.     */
    B4_TASK_CAL_FAILED       /**< Too few samples, or no valid seed.       */
} b4_task_cal_state_t;

/** @brief Ask the task to take one calibration step before its next sample.
 *
 * @param request The step.
 */
void
b4_task_request_calibration (b4_task_cal_request_t request);

/** @brief Report where the calibration stands.
 *
 * @param p_floor_mg Receives the floor in milli-g, or NULL. Zero until the
 *        state is DONE.
 * @return The state.
 */
b4_task_cal_state_t
b4_task_get_calibration (uint32_t * p_floor_mg);

/** @brief Noise samples the last calibration averaged (0 until it ends).
 *
 * @return The count, at most 20000.
 */
uint32_t
b4_task_get_calibration_samples (void);

#endif /* B4_TASK_H */
