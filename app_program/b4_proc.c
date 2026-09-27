/** @file b4_proc.c
 *
 * @brief Fixed-point filtering and Buddy 4 terrain classification.
 *
 * State transitions follow the event, guard, action, and next-state pattern
 * taught in INF2004 Lecture 4.2, State Machines. Thresholds remain provisional
 * until stationary and vehicle traces are recorded on the bench.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "b4_proc.h"

#define B4_PROC_MAX_SAMPLE_MG       4000
#define B4_PROC_MAX_COUNTER          255U

typedef struct
{
    b4_cfg_config_t config;
    b4_types_status_t status;
    int32_t previous_x_mg;
    int32_t previous_y_mg;
    int32_t previous_z_mg;
    uint32_t last_impact_timestamp_ms;
    uint8_t hump_confirm_count;
    uint8_t hump_complete_count;
    bool b_initialized;
    bool b_filter_ready;
    bool b_previous_sample_valid;
    bool b_have_last_impact;
} b4_proc_state_t;

static b4_proc_state_t g_b4_proc_state = {0};

/** @brief Return an absolute milli-g value without signed overflow.
 *
 * @param value_mg The signed acceleration value in milli-g.
 * @return The absolute acceleration value as unsigned milli-g.
 */
static uint32_t
b4_proc_abs_mg (int32_t value_mg);

/** @brief Read one axis from the processing status.
 *
 * @param p_status The status snapshot containing filtered axes.
 * @param axis The selected axis number.
 * @return The filtered acceleration for the selected axis.
 */
static int32_t
b4_proc_status_axis_mg (const b4_types_status_t * p_status,
                        uint8_t axis);

/** @brief Apply one step of a fixed-point low-pass filter.
 *
 * @param sample_mg The newest acceleration sample.
 * @param previous_mg The previously filtered acceleration.
 * @param divisor The configured smoothing divisor.
 * @return The updated filtered acceleration.
 */
static int32_t
b4_proc_filter_value (int32_t sample_mg,
                      int32_t previous_mg,
                      uint8_t divisor);

/** @brief Check that a sample is valid and within the processing range.
 *
 * @param p_sample The sample to check.
 * @return True when the sample can be processed.
 */
static bool
b4_proc_sample_is_valid (const b4_types_sample_t * p_sample);

/** @brief Update the filtered acceleration axes from one sample.
 *
 * @param p_sample The calibrated sample to filter.
 */
static void
b4_proc_update_filter (const b4_types_sample_t * p_sample);

/** @brief Apply a configured positive or negative axis direction.
 *
 * @param value_mg The acceleration value in milli-g.
 * @param sign The configured axis sign.
 * @return The direction-adjusted acceleration.
 */
static int32_t
b4_proc_oriented_value (int32_t value_mg, int8_t sign);

/** @brief Classify level, forward, and lateral tilt from filtered axes.
 */
static void
b4_proc_update_tilt (void);

/** @brief Find the largest raw axis change since the previous sample.
 *
 * @param p_sample The newest calibrated sample.
 * @return The largest absolute axis change in milli-g.
 */
static uint32_t
b4_proc_sample_delta_mg (const b4_types_sample_t * p_sample);

/** @brief Detect an impact and apply the configured refractory interval.
 *
 * @param sample_delta_mg The largest raw axis change in milli-g.
 * @param timestamp_ms The sample time in monotonic milliseconds.
 * @return True when this sample starts an impact event.
 */
static bool
b4_proc_detect_impact (uint32_t sample_delta_mg,
                       uint32_t timestamp_ms);

/** @brief Clear the active hump state without changing recorded peak history.
 */
static void
b4_proc_reset_active_hump (void);

/** @brief Advance the hump detector by one vertical-deviation sample.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_advance_hump (uint32_t amplitude_mg);

/** @brief Classify motion from impact, hump state, and sample change.
 *
 * @param sample_delta_mg The largest raw axis change in milli-g.
 */
static void
b4_proc_update_motion (uint32_t sample_delta_mg);

/** @brief Start a hump candidate above the configured threshold.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_start_hump (uint32_t amplitude_mg);

/** @brief Confirm or cancel a candidate hump.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_candidate (uint32_t amplitude_mg);

/** @brief Track the rising side and provisional peak of a hump.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_ascending (uint32_t amplitude_mg);

/** @brief Confirm whether a provisional peak starts descending.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_peak (uint32_t amplitude_mg);

/** @brief Track descent and confirm return below the completion threshold.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_descending (uint32_t amplitude_mg);

/** @brief Publish hump completion and retain a higher peak if present.
 */
static void
b4_proc_complete_hump (void);

/** @brief Initialize filtering and terrain analysis from a validated config.
 *
 * @param p_config The processing configuration.
 * @return The initialization result.
 */
b4_types_result_t
b4_proc_init (const b4_cfg_config_t * p_config)
{
    b4_types_result_t result = B4_RESULT_INVALID_ARGUMENT;

    g_b4_proc_state.b_initialized = false;

    if (NULL != p_config)
    {
        result = b4_cfg_validate(p_config);
        if (B4_RESULT_OK == result)
        {
            b4_proc_state_t initial_state = {0};

            g_b4_proc_state = initial_state;
            g_b4_proc_state.config = *p_config;
            g_b4_proc_state.status.health = B4_HEALTH_READY;
            g_b4_proc_state.status.fault = B4_FAULT_NONE;
            g_b4_proc_state.status.tilt = B4_TILT_UNKNOWN;
            g_b4_proc_state.status.motion = B4_MOTION_QUIET;
            g_b4_proc_state.status.hump_state = B4_HUMP_NORMAL;
            g_b4_proc_state.status.b_sample_stale = true;
            g_b4_proc_state.b_initialized = true;
        }
        else
        {
            g_b4_proc_state.status.health = B4_HEALTH_FAULT;
            g_b4_proc_state.status.fault = B4_FAULT_HARDWARE_CONFIG;
        }
    }
    else
    {
        g_b4_proc_state.b_initialized = false;
        g_b4_proc_state.status.health = B4_HEALTH_FAULT;
        g_b4_proc_state.status.fault = B4_FAULT_HARDWARE_CONFIG;
    }

    return (result);
}

/** @brief Filter, classify, and publish one calibrated sample.
 *
 * @param p_sample The calibrated accelerometer sample.
 * @return The processing result.
 */
b4_types_result_t
b4_proc_process (const b4_types_sample_t * p_sample)
{
    b4_types_result_t result = B4_RESULT_OK;
    uint32_t sample_delta_mg = 0U;
    int32_t vertical_mg = 0;

    if (NULL == p_sample)
    {
        result = B4_RESULT_INVALID_ARGUMENT;
    }
    else if (false == g_b4_proc_state.b_initialized)
    {
        result = B4_RESULT_NOT_INITIALIZED;
    }
    else if (false == b4_proc_sample_is_valid(p_sample))
    {
        result = B4_RESULT_INVALID_SAMPLE;
        g_b4_proc_state.status.health = B4_HEALTH_DEGRADED;
        g_b4_proc_state.status.fault = B4_FAULT_INVALID_SAMPLE;
        g_b4_proc_state.status.sample.b_valid = false;
        g_b4_proc_state.status.b_sample_stale = true;
        g_b4_proc_state.status.b_impact_event = false;
        g_b4_proc_state.status.b_hump_started_event = false;
        g_b4_proc_state.status.b_hump_completed_event = false;
        g_b4_proc_state.status.tilt = B4_TILT_UNKNOWN;
        g_b4_proc_state.status.motion = B4_MOTION_QUIET;
        b4_proc_reset_active_hump();
        g_b4_proc_state.b_filter_ready = false;
        g_b4_proc_state.b_previous_sample_valid = false;
    }
    else
    {
        g_b4_proc_state.status.sample = *p_sample;
        g_b4_proc_state.status.health = B4_HEALTH_READY;
        g_b4_proc_state.status.fault = B4_FAULT_NONE;
        g_b4_proc_state.status.b_sample_stale = false;
        g_b4_proc_state.status.b_impact_event = false;
        g_b4_proc_state.status.b_hump_started_event = false;
        g_b4_proc_state.status.b_hump_completed_event = false;

        sample_delta_mg = b4_proc_sample_delta_mg(p_sample);
        g_b4_proc_state.status.b_impact_event = b4_proc_detect_impact(
            sample_delta_mg, p_sample->timestamp_ms);
        b4_proc_update_filter(p_sample);
        b4_proc_update_tilt();

        vertical_mg = b4_proc_status_axis_mg(
            &g_b4_proc_state.status,
            g_b4_proc_state.config.vertical_axis);
        vertical_mg = b4_proc_oriented_value(
            vertical_mg, g_b4_proc_state.config.vertical_sign);
        g_b4_proc_state.status.vertical_deviation_mg =
            vertical_mg -
            (int32_t)g_b4_proc_state.config.gravity_reference_mg;

        if (false == g_b4_proc_state.status.b_impact_event)
        {
            b4_proc_advance_hump(b4_proc_abs_mg(
                g_b4_proc_state.status.vertical_deviation_mg));
        }

        b4_proc_update_motion(sample_delta_mg);
        g_b4_proc_state.previous_x_mg = p_sample->accel_x_mg;
        g_b4_proc_state.previous_y_mg = p_sample->accel_y_mg;
        g_b4_proc_state.previous_z_mg = p_sample->accel_z_mg;
        g_b4_proc_state.b_previous_sample_valid = true;
    }

    return (result);
}

/** @brief Return a status snapshot and report stale samples.
 *
 * @param now_ms The current monotonic time in milliseconds.
 * @param p_status The destination for the status snapshot.
 * @return The status result, including any stale-sample condition.
 */
b4_types_result_t
b4_proc_get_status (uint32_t now_ms, b4_types_status_t * p_status)
{
    b4_types_result_t result = B4_RESULT_OK;
    uint32_t sample_age_ms = 0U;

    if (NULL == p_status)
    {
        result = B4_RESULT_INVALID_ARGUMENT;
    }
    else
    {
        *p_status = g_b4_proc_state.status;

        if (false == g_b4_proc_state.b_initialized)
        {
            p_status->health = B4_HEALTH_NOT_INITIALIZED;
            p_status->fault = B4_FAULT_NONE;
            result = B4_RESULT_NOT_INITIALIZED;
        }
        else if (false == p_status->sample.b_valid)
        {
            p_status->b_sample_stale = true;
            p_status->health = B4_HEALTH_DEGRADED;
            p_status->fault = B4_FAULT_STALE_SAMPLE;
            result = B4_RESULT_STALE_SAMPLE;
        }
        else
        {
            sample_age_ms = now_ms - p_status->sample.timestamp_ms;
            if (sample_age_ms > g_b4_proc_state.config.stale_after_ms)
            {
                p_status->b_sample_stale = true;
                p_status->health = B4_HEALTH_DEGRADED;
                p_status->fault = B4_FAULT_STALE_SAMPLE;
                result = B4_RESULT_STALE_SAMPLE;
            }
            else
            {
                p_status->b_sample_stale = false;
            }
        }
    }

    return (result);
}

/** @brief Reset run-level peak tracking and sample history.
 *
 * @return The reset result.
 */
b4_types_result_t
b4_proc_reset_run_stats (void)
{
    b4_types_result_t result = B4_RESULT_OK;

    if (false == g_b4_proc_state.b_initialized)
    {
        result = B4_RESULT_NOT_INITIALIZED;
    }
    else
    {
        g_b4_proc_state.status.highest_hump_peak_mg = 0U;
        g_b4_proc_state.status.current_hump_peak_mg = 0U;
        g_b4_proc_state.status.sample.b_valid = false;
        g_b4_proc_state.status.b_sample_stale = true;
        g_b4_proc_state.status.b_impact_event = false;
        g_b4_proc_state.status.b_hump_started_event = false;
        g_b4_proc_state.status.b_hump_completed_event = false;
        g_b4_proc_state.status.fault = B4_FAULT_NONE;
        g_b4_proc_state.status.health = B4_HEALTH_READY;
        g_b4_proc_state.status.tilt = B4_TILT_UNKNOWN;
        g_b4_proc_state.status.motion = B4_MOTION_QUIET;
        g_b4_proc_state.b_filter_ready = false;
        g_b4_proc_state.b_previous_sample_valid = false;
        g_b4_proc_state.b_have_last_impact = false;
        g_b4_proc_state.last_impact_timestamp_ms = 0U;
        b4_proc_reset_active_hump();
    }

    return (result);
}

static uint32_t
b4_proc_abs_mg (int32_t value_mg)
{
    uint32_t magnitude_mg = 0U;

    if (value_mg < 0)
    {
        magnitude_mg = (uint32_t)(-(int64_t)value_mg);
    }
    else
    {
        magnitude_mg = (uint32_t)value_mg;
    }

    return (magnitude_mg);
}

static int32_t
b4_proc_status_axis_mg (const b4_types_status_t * p_status, uint8_t axis)
{
    int32_t axis_mg = 0;

    switch (axis)
    {
        case B4_CFG_AXIS_X:
            axis_mg = p_status->filtered_x_mg;
            break;
        case B4_CFG_AXIS_Y:
            axis_mg = p_status->filtered_y_mg;
            break;
        case B4_CFG_AXIS_Z:
            axis_mg = p_status->filtered_z_mg;
            break;
        default:
            axis_mg = 0;
            break;
    }

    return (axis_mg);
}

static int32_t
b4_proc_filter_value (int32_t sample_mg,
                      int32_t previous_mg,
                      uint8_t divisor)
{
    int32_t delta_mg = sample_mg - previous_mg;
    int32_t filtered_mg = previous_mg +
                          (delta_mg / (int32_t)divisor);

    return (filtered_mg);
}

static bool
b4_proc_sample_is_valid (const b4_types_sample_t * p_sample)
{
    bool b_in_range = false;

    if ((true == p_sample->b_valid) &&
        (p_sample->accel_x_mg <= B4_PROC_MAX_SAMPLE_MG) &&
        (p_sample->accel_x_mg >= -B4_PROC_MAX_SAMPLE_MG) &&
        (p_sample->accel_y_mg <= B4_PROC_MAX_SAMPLE_MG) &&
        (p_sample->accel_y_mg >= -B4_PROC_MAX_SAMPLE_MG) &&
        (p_sample->accel_z_mg <= B4_PROC_MAX_SAMPLE_MG) &&
        (p_sample->accel_z_mg >= -B4_PROC_MAX_SAMPLE_MG))
    {
        b_in_range = true;
    }

    return (b_in_range);
}

static void
b4_proc_update_filter (const b4_types_sample_t * p_sample)
{
    if (false == g_b4_proc_state.b_filter_ready)
    {
        g_b4_proc_state.status.filtered_x_mg = p_sample->accel_x_mg;
        g_b4_proc_state.status.filtered_y_mg = p_sample->accel_y_mg;
        g_b4_proc_state.status.filtered_z_mg = p_sample->accel_z_mg;
        g_b4_proc_state.b_filter_ready = true;
    }
    else
    {
        g_b4_proc_state.status.filtered_x_mg = b4_proc_filter_value(
            p_sample->accel_x_mg,
            g_b4_proc_state.status.filtered_x_mg,
            g_b4_proc_state.config.filter_divisor);
        g_b4_proc_state.status.filtered_y_mg = b4_proc_filter_value(
            p_sample->accel_y_mg,
            g_b4_proc_state.status.filtered_y_mg,
            g_b4_proc_state.config.filter_divisor);
        g_b4_proc_state.status.filtered_z_mg = b4_proc_filter_value(
            p_sample->accel_z_mg,
            g_b4_proc_state.status.filtered_z_mg,
            g_b4_proc_state.config.filter_divisor);
    }
}

static int32_t
b4_proc_oriented_value (int32_t value_mg, int8_t sign)
{
    int32_t oriented_mg = value_mg * (int32_t)sign;

    return (oriented_mg);
}

static void
b4_proc_update_tilt (void)
{
    int32_t forward_mg = b4_proc_status_axis_mg(
        &g_b4_proc_state.status,
        g_b4_proc_state.config.forward_axis);
    int32_t lateral_mg = b4_proc_status_axis_mg(
        &g_b4_proc_state.status,
        g_b4_proc_state.config.lateral_axis);
    uint32_t forward_abs_mg = 0U;
    uint32_t lateral_abs_mg = 0U;
    bool b_forward_tilt = false;
    bool b_lateral_tilt = false;

    forward_mg = b4_proc_oriented_value(
        forward_mg, g_b4_proc_state.config.forward_sign);
    lateral_mg = b4_proc_oriented_value(
        lateral_mg, g_b4_proc_state.config.lateral_sign);
    forward_abs_mg = b4_proc_abs_mg(forward_mg);
    lateral_abs_mg = b4_proc_abs_mg(lateral_mg);
    b_forward_tilt =
        (forward_abs_mg >= g_b4_proc_state.config.tilt_threshold_mg);
    b_lateral_tilt =
        (lateral_abs_mg >= g_b4_proc_state.config.tilt_threshold_mg);

    if ((true == b_forward_tilt) && (true == b_lateral_tilt))
    {
        g_b4_proc_state.status.tilt = B4_TILT_COMPOUND;
    }
    else if (true == b_forward_tilt)
    {
        if (forward_mg > 0)
        {
            g_b4_proc_state.status.tilt = B4_TILT_FORWARD;
        }
        else
        {
            g_b4_proc_state.status.tilt = B4_TILT_BACKWARD;
        }
    }
    else if (true == b_lateral_tilt)
    {
        if (lateral_mg > 0)
        {
            g_b4_proc_state.status.tilt = B4_TILT_RIGHT;
        }
        else
        {
            g_b4_proc_state.status.tilt = B4_TILT_LEFT;
        }
    }
    else
    {
        g_b4_proc_state.status.tilt = B4_TILT_LEVEL;
    }
}

static uint32_t
b4_proc_sample_delta_mg (const b4_types_sample_t * p_sample)
{
    uint32_t max_delta_mg = 0U;
    uint32_t axis_delta_mg = 0U;

    if (true == g_b4_proc_state.b_previous_sample_valid)
    {
        axis_delta_mg = b4_proc_abs_mg(
            p_sample->accel_x_mg - g_b4_proc_state.previous_x_mg);
        if (axis_delta_mg > max_delta_mg)
        {
            max_delta_mg = axis_delta_mg;
        }

        axis_delta_mg = b4_proc_abs_mg(
            p_sample->accel_y_mg - g_b4_proc_state.previous_y_mg);
        if (axis_delta_mg > max_delta_mg)
        {
            max_delta_mg = axis_delta_mg;
        }

        axis_delta_mg = b4_proc_abs_mg(
            p_sample->accel_z_mg - g_b4_proc_state.previous_z_mg);
        if (axis_delta_mg > max_delta_mg)
        {
            max_delta_mg = axis_delta_mg;
        }
    }

    return (max_delta_mg);
}

static bool
b4_proc_detect_impact (uint32_t sample_delta_mg, uint32_t timestamp_ms)
{
    bool b_detected = false;
    bool b_threshold_crossed = false;
    bool b_refractory_elapsed = false;

    if (sample_delta_mg >=
        g_b4_proc_state.config.impact_delta_threshold_mg)
    {
        b_threshold_crossed = true;
    }

    if (false == g_b4_proc_state.b_have_last_impact)
    {
        b_refractory_elapsed = true;
    }
    else if ((timestamp_ms -
              g_b4_proc_state.last_impact_timestamp_ms) >=
             g_b4_proc_state.config.impact_refractory_ms)
    {
        b_refractory_elapsed = true;
    }
    else
    {
        b_refractory_elapsed = false;
    }

    if ((true == b_threshold_crossed) &&
        (true == b_refractory_elapsed))
    {
        b_detected = true;
        g_b4_proc_state.last_impact_timestamp_ms = timestamp_ms;
        g_b4_proc_state.b_have_last_impact = true;
    }

    return (b_detected);
}

static void
b4_proc_reset_active_hump (void)
{
    g_b4_proc_state.status.hump_state = B4_HUMP_NORMAL;
    g_b4_proc_state.status.current_hump_peak_mg = 0U;
    g_b4_proc_state.hump_confirm_count = 0U;
    g_b4_proc_state.hump_complete_count = 0U;
}

static void
b4_proc_advance_hump (uint32_t amplitude_mg)
{
    if (B4_HUMP_COMPLETE == g_b4_proc_state.status.hump_state)
    {
        b4_proc_reset_active_hump();
    }

    switch (g_b4_proc_state.status.hump_state)
    {
        case B4_HUMP_NORMAL:
            b4_proc_start_hump(amplitude_mg);
            break;
        case B4_HUMP_CANDIDATE:
            b4_proc_update_hump_candidate(amplitude_mg);
            break;
        case B4_HUMP_ASCENDING:
            b4_proc_update_hump_ascending(amplitude_mg);
            break;
        case B4_HUMP_PEAK:
            b4_proc_update_hump_peak(amplitude_mg);
            break;
        case B4_HUMP_DESCENDING:
            b4_proc_update_hump_descending(amplitude_mg);
            break;
        case B4_HUMP_COMPLETE:
            b4_proc_reset_active_hump();
            break;
        default:
            b4_proc_reset_active_hump();
            break;
    }
}

/** @brief Start a hump candidate above the configured threshold.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_start_hump (uint32_t amplitude_mg)
{
    if (amplitude_mg >=
        g_b4_proc_state.config.hump_start_threshold_mg)
    {
        g_b4_proc_state.status.current_hump_peak_mg = amplitude_mg;
        g_b4_proc_state.status.hump_state = B4_HUMP_CANDIDATE;
        g_b4_proc_state.hump_confirm_count = 1U;
    }
}

/** @brief Confirm or cancel a candidate hump.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_candidate (uint32_t amplitude_mg)
{
    if (amplitude_mg >=
        g_b4_proc_state.config.hump_start_threshold_mg)
    {
        if (amplitude_mg >
            g_b4_proc_state.status.current_hump_peak_mg)
        {
            g_b4_proc_state.status.current_hump_peak_mg = amplitude_mg;
        }

        if (g_b4_proc_state.hump_confirm_count < B4_PROC_MAX_COUNTER)
        {
            g_b4_proc_state.hump_confirm_count++;
        }

        if (g_b4_proc_state.hump_confirm_count >=
            g_b4_proc_state.config.hump_confirm_samples)
        {
            g_b4_proc_state.status.hump_state = B4_HUMP_ASCENDING;
            g_b4_proc_state.status.b_hump_started_event = true;
        }
    }
    else
    {
        b4_proc_reset_active_hump();
    }
}

/** @brief Track the rising side and provisional peak of a hump.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_ascending (uint32_t amplitude_mg)
{
    uint32_t peak_mg =
        g_b4_proc_state.status.current_hump_peak_mg;
    uint32_t hysteresis_mg =
        g_b4_proc_state.config.hump_hysteresis_mg;

    if (amplitude_mg > peak_mg)
    {
        g_b4_proc_state.status.current_hump_peak_mg = amplitude_mg;
        peak_mg = amplitude_mg;
    }

    if ((amplitude_mg < peak_mg) &&
        ((peak_mg - amplitude_mg) >= hysteresis_mg))
    {
        g_b4_proc_state.status.hump_state = B4_HUMP_PEAK;
    }
}

/** @brief Confirm whether a provisional peak starts descending.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_peak (uint32_t amplitude_mg)
{
    uint32_t peak_mg =
        g_b4_proc_state.status.current_hump_peak_mg;
    uint32_t hysteresis_mg =
        g_b4_proc_state.config.hump_hysteresis_mg;

    if (amplitude_mg > peak_mg)
    {
        g_b4_proc_state.status.current_hump_peak_mg = amplitude_mg;
        g_b4_proc_state.status.hump_state = B4_HUMP_ASCENDING;
    }
    else if ((amplitude_mg < peak_mg) &&
             ((peak_mg - amplitude_mg) >= hysteresis_mg))
    {
        g_b4_proc_state.status.hump_state = B4_HUMP_DESCENDING;
    }
    else
    {
        g_b4_proc_state.status.hump_state = B4_HUMP_PEAK;
    }
}

/** @brief Track descent and confirm return below the completion threshold.
 *
 * @param amplitude_mg The absolute vertical deviation in milli-g.
 */
static void
b4_proc_update_hump_descending (uint32_t amplitude_mg)
{
    if (amplitude_mg >
        g_b4_proc_state.status.current_hump_peak_mg)
    {
        g_b4_proc_state.status.current_hump_peak_mg = amplitude_mg;
    }

    if (amplitude_mg <=
        g_b4_proc_state.config.hump_complete_threshold_mg)
    {
        if (g_b4_proc_state.hump_complete_count < B4_PROC_MAX_COUNTER)
        {
            g_b4_proc_state.hump_complete_count++;
        }
    }
    else
    {
        g_b4_proc_state.hump_complete_count = 0U;
    }

    if (g_b4_proc_state.hump_complete_count >=
        g_b4_proc_state.config.hump_complete_samples)
    {
        b4_proc_complete_hump();
    }
}

/** @brief Publish hump completion and retain a higher peak if present.
 */
static void
b4_proc_complete_hump (void)
{
    g_b4_proc_state.status.hump_state = B4_HUMP_COMPLETE;
    g_b4_proc_state.status.b_hump_completed_event = true;

    if (g_b4_proc_state.status.current_hump_peak_mg >
        g_b4_proc_state.status.highest_hump_peak_mg)
    {
        g_b4_proc_state.status.highest_hump_peak_mg =
            g_b4_proc_state.status.current_hump_peak_mg;
    }
}

static void
b4_proc_update_motion (uint32_t sample_delta_mg)
{
    if (true == g_b4_proc_state.status.b_impact_event)
    {
        g_b4_proc_state.status.motion = B4_MOTION_IMPACT;
    }
    else if ((B4_HUMP_CANDIDATE ==
              g_b4_proc_state.status.hump_state) ||
             (B4_HUMP_ASCENDING ==
              g_b4_proc_state.status.hump_state) ||
             (B4_HUMP_PEAK == g_b4_proc_state.status.hump_state))
    {
        g_b4_proc_state.status.motion = B4_MOTION_CLIMBING_HUMP;
    }
    else if (B4_HUMP_DESCENDING ==
             g_b4_proc_state.status.hump_state)
    {
        g_b4_proc_state.status.motion = B4_MOTION_DESCENDING_HUMP;
    }
    else if (sample_delta_mg >=
             g_b4_proc_state.config.motion_threshold_mg)
    {
        g_b4_proc_state.status.motion = B4_MOTION_ACCELERATING;
    }
    else
    {
        g_b4_proc_state.status.motion = B4_MOTION_QUIET;
    }
}
