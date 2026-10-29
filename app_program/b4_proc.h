/** @file b4_proc.h
 *
 * @brief Hardware-independent Buddy 4 sample processing interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_PROC_H
#define B4_PROC_H

#include <stdint.h>

#include "b4_cfg.h"
#include "b4_types.h"

/** Initialize filtering and terrain analysis with a validated configuration. */
b4_types_result_t
b4_proc_init (const b4_cfg_config_t * p_config);

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

/** Clear the run maximum and latest result but keep the baseline. */
b4_types_result_t
b4_proc_clear_run_maximum (void);

#endif /* B4_PROC_H */
