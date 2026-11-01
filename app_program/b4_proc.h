/** @file b4_proc.h
 *
 * @brief Hardware-independent Buddy 4 sample processing interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_PROC_H
#define B4_PROC_H

#include <stdbool.h>
#include <stdint.h>

#include "b4_cfg.h"
#include "b4_types.h"

/** Initialize filtering and terrain analysis with a validated configuration. */
b4_types_result_t
b4_proc_init (const b4_cfg_config_t * p_config);

/** Set the drive input for the next sample.
 *
 * b_learn is true only while the Vehicle Controller reports the car both
 * driving and moving and its word is fresh. The noise median learns on those
 * samples only, one step per millimetre of wheel travel. speed_mm_s is the
 * mean of the absolute wheel speeds in millimetres per second, so a pivot
 * counts as travel. With the time since the previous sample it gives the
 * travel, and it is reported in the status.
 * Call it before each b4_proc_process(). */
void
b4_proc_set_drive (bool b_learn, int32_t speed_mm_s);

/** Process one calibrated accelerometer sample. */
b4_types_result_t
b4_proc_process (const b4_types_sample_t * p_sample);

/** Process one magnetometer sample and update turning classification.
 *
 * Pass an invalid sample after a failed read so the classifier reports an
 * unavailable direction instead of reusing the last one. */
b4_types_result_t
b4_proc_process_magnetometer (const b4_types_mag_sample_t * p_mag_sample);

/** Copy the latest status and mark it stale when its age exceeds the limit. */
b4_types_result_t
b4_proc_get_status (uint32_t now_ms, b4_types_status_t * p_status);

/** Reset run-level peak tracking and sample history. */
b4_types_result_t
b4_proc_reset_run_stats (void);

/** Clear the run maximum and latest result but keep the baseline and the
 * noise estimate. The learning count restarts at zero, so no hump can start
 * until the estimate has had its learning hold of 2^n steps, 2^n mm of
 * driving. */
b4_types_result_t
b4_proc_clear_run_maximum (void);

/** Forget the flat-ground reference so it is taken again from the next
 * settled samples, restart the learning count, and clear any sweep
 * measurement. Health stays below READY until the car has been still long
 * enough to confirm level. */
b4_types_result_t
b4_proc_calibrate_level (void);

/** Start measuring the sweep noise: from now on every sample the noise
 * estimate may use, driving or not, is also added to a plain mean. */
b4_types_result_t
b4_proc_calibrate_measure (void);

/** Return how many samples the measurement holds. */
uint32_t
b4_proc_calibrate_samples (void);

/** Stop measuring and seed the noise estimate with the mean.
 *
 * Fails when the measurement holds fewer than min_samples, or the mean is
 * not a valid seed (above 500 milli-g). The estimate stays as it was on
 * failure. The seed is a starting value, not a floor. *p_floor_mg receives
 * the measured mean in milli-g either way, 0 if nothing was measured. */
b4_types_result_t
b4_proc_calibrate_apply (uint32_t min_samples, uint32_t * p_floor_mg);

#endif /* B4_PROC_H */
