/** @file b4_proc.c
 *
 * @brief Fixed-point filtering and Buddy 4 terrain classification.
 *
 * State transitions follow the event, guard, action, and next-state pattern
 * taught in INF2004 Lecture 4.2, State Machines.
 *
 * A hump raises one contact while the others stay on the floor, so the
 * chassis tilts about the line through those pivot contacts. With the right
 * drive wheel on the ramp and the left wheel and caster on the floor, the
 * axle line rises by the ramp height across the track, so the lateral axis
 * measures gravity times the sine of that tilt wherever the caster and IMU
 * sit. For a lever L, the track, and an incline change a in milli-g:
 *
 *     height = L * a / g
 *
 * The ramp crest is a point, so the wheel never rests at full height. The
 * peak height is the highest value of a short moving mean of the filtered
 * incline during the hump, not a settled window and not an integral of
 * motion acceleration. Over a pointed crest the axle still rises the full
 * ramp height. The estimate assumes equal tyre deformation on both sides.
 *
 * A bracket catch also tilts the car, but about the drive wheels, so it
 * does not match the wheel-lift geometry. A sample is consistent when the
 * pitch change matches the pitch that the roll change predicts and the
 * total acceleration stays at its flat-ground value. A hump may start and
 * be confirmed only on consistent samples, and the height peak updates only
 * when the whole moving-mean window is consistent. Once confirmed, a hump
 * still runs to completion on every sample, so it always ends. A hump with
 * no fully consistent window completes as invalid.
 *
 * A gap in the sample stream or an invalid sample aborts a hump in progress
 * as invalid, and health stays DEGRADED until fresh level samples confirm
 * the car is back on flat ground.
 *
 * The start, completion, and hysteresis thresholds are multiples of a
 * running estimate of the noise in the incline offset, so they follow the
 * rolling noise of the car instead of a fixed height. The estimate learns
 * only from samples on flat ground, and no hump may start until it has had
 * a warm-up of samples after the run starts.
 *
 * The multiples remain provisional until flat driving traces set them.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "b4_proc.h"

#define B4_PROC_MAX_SAMPLE_MG       4000
#define B4_PROC_MAX_COUNTER          255U
#define B4_PROC_NOISE_SCALE          256
#define B4_PROC_NOISE_HALF_SCALE     128
#define B4_PROC_K_TENTHS_PER_ONE      10U
#define B4_PROC_MIN_THRESHOLD_MG       1U
#define B4_PROC_CAL_MAX_SAMPLES    20000U

/** @brief Private terrain state, including unpublished filter history.
 */
typedef struct
{
    b4_cfg_config_t   config;
    b4_types_status_t status;
    int32_t           previous_x_mg;
    int32_t           previous_y_mg;
    int32_t           previous_z_mg;
    int32_t           incline_window_mg[B4_CFG_MAX_SETTLE_SAMPLES];
    int16_t           forward_window_mg[B4_CFG_MAX_SETTLE_SAMPLES];
    int16_t           lateral_window_mg[B4_CFG_MAX_SETTLE_SAMPLES];
    int16_t           vertical_window_mg[B4_CFG_MAX_SETTLE_SAMPLES];
    int32_t           incline_baseline_mg;
    int32_t           lateral_mg;
    int32_t           lateral_baseline_mg;
    int32_t           noise_q8;
    uint32_t          magnitude_baseline_mg;
    uint32_t          active_peak_height_um;
    uint32_t          noise_warmup_count;
    uint32_t          cal_sum_mg;
    uint32_t          cal_count;
    uint32_t          start_threshold_mg;
    uint32_t          complete_threshold_mg;
    uint32_t          last_impact_timestamp_ms;
    uint32_t          previous_timestamp_ms;
    uint32_t          mag_last_timestamp_ms;
    int64_t           turn_cross_accum;
    int16_t           mag_previous_x;
    int16_t           mag_previous_y;
    uint8_t           hump_confirm_count;
    uint8_t           hump_complete_count;
    uint8_t           level_count;
    uint8_t           baseline_step_count;
    uint8_t           tilt_consistent_count;
    uint8_t           launch_settle_count;
    uint8_t           incline_window_count;
    uint8_t           incline_window_next;
    bool              b_initialized;
    bool              b_filter_ready;
    bool              b_previous_sample_valid;
    bool              b_have_last_impact;
    bool              b_incline_baseline_valid;
    bool              b_active_height_measured;
    bool              b_active_height_tainted;
    bool              b_level_since_gap;
    bool              b_launch_settled;
    bool              b_mag_previous_valid;
    bool              b_mag_seen;
    bool              b_cal_measuring;
} b4_proc_state_t;

static b4_proc_state_t g_b4_proc_state = {0};

/** @brief Return an absolute milli-g value without signed overflow.
 *
 * @param value_mg The signed acceleration value in milli-g.
 * @return The absolute acceleration value as unsigned milli-g.
 */
static uint32_t
b4_proc_abs_mg (int32_t value_mg);

/** @brief Return the magnitude of the difference between two values.
 *
 * @param left_mg The first acceleration in milli-g.
 * @param right_mg The second acceleration in milli-g.
 * @return The unsigned difference in milli-g.
 */
static uint32_t
b4_proc_difference_mg (int32_t left_mg, int32_t right_mg);

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

/** @brief Orient the pitch and lateral axes and add the axes to history.
 *
 * The three filtered axes are written at the slot that the next call to
 * b4_proc_push_incline fills, so all four windows share one index and one
 * count. It must run before b4_proc_push_incline.
 */
static void
b4_proc_update_attitude (void);

/** @brief Return the mean of the newest samples of one axis window.
 *
 * @param p_window The axis history, B4_CFG_MAX_SETTLE_SAMPLES entries.
 * @param window_samples The number of newest samples to average, between
 *                       2 and the history size.
 * @return The mean of the sensor axis in milli-g, before orientation.
 */
static int32_t
b4_proc_axis_mean_mg (const int16_t * p_window, uint8_t window_samples);

/** @brief Return the integer square root of a value, rounded down.
 *
 * @param value The value to root.
 * @return The largest integer whose square does not exceed the value.
 */
static uint32_t
b4_proc_isqrt (uint64_t value);

/** @brief Move a baseline one milli-g toward a value.
 *
 * @param p_baseline The baseline to adapt.
 * @param value_mg The newest value in milli-g.
 */
static void
b4_proc_step_toward (int32_t * p_baseline, int32_t value_mg);

/** @brief Capture the pitch, lateral, and magnitude baselines.
 *
 * Each uses the mean of the settled window, like the incline baseline.
 */
static void
b4_proc_capture_attitude_baselines (void);

/** @brief Compute the tilt residuals and the consistency flag.
 *
 * The oriented roll change gives the pitch that the wheel-lift model
 * predicts. The residuals and the flag are zero and false while no
 * baseline exists. The result is reported only in this step and does not
 * drive the hump detector yet.
 */
static void
b4_proc_update_residuals (void);

/** @brief Clear the residuals, the flag, and the consistent-sample count.
 */
static void
b4_proc_clear_residuals (void);

/** @brief Add the newest filtered incline to the settled-window history.
 *
 * @param incline_mg The filtered hump-axis acceleration in milli-g.
 */
static void
b4_proc_push_incline (int32_t incline_mg);

/** @brief Return the peak-to-peak spread of the newest settled window.
 *
 * @return The spread in milli-g, or a value above every threshold when the
 *         window is not full yet.
 */
static uint32_t
b4_proc_incline_spread_mg (void);

/** @brief Return the mean of the newest incline samples.
 *
 * Averaging keeps sample noise from biasing the baseline and the peak
 * height upwards.
 *
 * @param window_samples The number of newest samples to average, between
 *                       2 and the history size.
 * @return The mean incline in milli-g.
 */
static int32_t
b4_proc_incline_mean_mg (uint8_t window_samples);

/** @brief Report whether the newest settled window is quiet enough.
 *
 * @return True when the window is full and its spread is within the limit.
 */
static bool
b4_proc_incline_is_settled (void);

/** @brief Adapt the flat-ground incline baseline while no hump is active.
 *
 * The baseline is captured from the first settled window mean and then
 * follows slow attitude changes. It freezes as soon as the incline moves
 * further from the baseline than the gate, so a climb cannot drag the
 * reference along.
 *
 * @param incline_mg The newest filtered incline in milli-g.
 * @param b_settled True when the settled window is quiet enough.
 */
static void
b4_proc_update_baseline (int32_t incline_mg, bool b_settled);

/** @brief Return the incline offset from the flat-ground baseline.
 *
 * @return The signed offset in milli-g, or zero while no baseline exists.
 */
static int32_t
b4_proc_incline_delta_mg (void);

/** @brief Convert an incline offset to a support-height estimate.
 *
 * @param delta_mg The signed incline offset in milli-g.
 * @return The absolute height difference in micrometres.
 */
static uint32_t
b4_proc_height_um (int32_t delta_mg);

/** @brief Track the tallest moving-mean height of the active hump.
 *
 * The height uses the moving mean, not the newest sample. It updates only
 * when every sample in the moving-mean window was consistent with the
 * wheel-lift geometry, so bracket-catch samples never reach the height.
 */
static void
b4_proc_update_height (void);

/** @brief Return the noise estimate in milli-g, never below the floor.
 *
 * @return The rounded noise estimate in milli-g.
 */
static uint32_t
b4_proc_noise_mg (void);

/** @brief Derive the live hump thresholds from the noise estimate.
 *
 * Stores the start and completion thresholds, which every check of one
 * sample then shares, and publishes the noise and detectable height.
 */
static void
b4_proc_refresh_thresholds (void);

/** @brief Update the noise estimate and the warm-up count for one sample.
 *
 * The estimate follows the incline offset only on flat ground, and each
 * input is clipped at the completion threshold. The estimate rises and
 * falls at different rates and never goes below the floor. The warm-up
 * counts every processed sample.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_noise (uint32_t amplitude_mg);

/** @brief Abort a hump when the sample stream breaks.
 *
 * A hump that had started is published as an invalid result, the run
 * maximum is kept, and level ground must be confirmed again.
 */
static void
b4_proc_abort_hump (void);

/** @brief Count fresh level samples and set health from the result.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_level (uint32_t amplitude_mg);

/** @brief Track whether the car is steady since the run started.
 *
 * Counts consecutive samples that are consistent with the wheel-lift
 * geometry and level, and latches the launch as settled once the count
 * reaches the height mean window. A hump may not start before then.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_launch (uint32_t amplitude_mg);

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

/** @brief Advance the hump detector by one incline-amplitude sample.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 * @param b_settled True when the settled window is quiet enough.
 */
static void
b4_proc_advance_hump (uint32_t amplitude_mg, bool b_settled);

/** @brief Classify motion from impact, hump state, and sample change.
 *
 * @param sample_delta_mg The largest raw axis change in milli-g.
 */
static void
b4_proc_update_motion (uint32_t sample_delta_mg);

/** @brief Start a hump candidate above the threshold from level ground.
 *
 * The current sample must also be consistent with the wheel-lift geometry,
 * the car must have settled since the run started, and the noise estimate
 * must have had its warm-up, so the roll of a launch cannot pass for a
 * ramp.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_start_hump (uint32_t amplitude_mg);

/** @brief Confirm or cancel a candidate hump.
 *
 * A sample below the threshold or inconsistent with the wheel-lift geometry
 * cancels the candidate, so confirmation needs consecutive samples that are
 * both above the threshold and consistent.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_hump_candidate (uint32_t amplitude_mg);

/** @brief Track the rising side and provisional peak of a hump.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_hump_ascending (uint32_t amplitude_mg);

/** @brief Confirm whether a provisional peak starts descending.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_hump_peak (uint32_t amplitude_mg);

/** @brief Track descent and confirm return below the completion threshold.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 * @param b_settled True when the settled window is quiet enough.
 */
static void
b4_proc_update_hump_descending (uint32_t amplitude_mg, bool b_settled);

/** @brief Publish hump completion and retain higher recorded peaks.
 */
static void
b4_proc_complete_hump (void);

/** @brief Limit the signed turn accumulator to the status field width.
 *
 * @param value The signed cross-product accumulator.
 * @return The accumulator clamped to the int32_t range.
 */
static int32_t
b4_proc_clamp_cross (int64_t value);

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
            /* Validation bounds the floor to 500 milli-g, so the scaled
             * value fits int32_t. */
            g_b4_proc_state.noise_q8 =
                (int32_t)p_config->noise_floor_mg * B4_PROC_NOISE_SCALE;
            b4_proc_refresh_thresholds();
            g_b4_proc_state.status.health = B4_HEALTH_READY;
            g_b4_proc_state.status.fault = B4_FAULT_NONE;
            g_b4_proc_state.status.tilt = B4_TILT_UNKNOWN;
            g_b4_proc_state.status.motion = B4_MOTION_QUIET;
            g_b4_proc_state.status.hump_state = B4_HUMP_NORMAL;
            g_b4_proc_state.status.turn = B4_TURN_UNKNOWN;
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
    uint32_t amplitude_mg = 0U;
    int32_t vertical_mg = 0;
    int32_t incline_mg = 0;
    int32_t incline_delta_mg = 0;
    bool b_settled = false;

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
        b4_proc_abort_hump();
        g_b4_proc_state.b_previous_sample_valid = false;
    }
    else
    {
        /* A late sample means the stream stalled, so anything measured
         * across the gap cannot be trusted. */
        if ((true == g_b4_proc_state.b_previous_sample_valid) &&
            ((p_sample->timestamp_ms -
              g_b4_proc_state.previous_timestamp_ms) >
             g_b4_proc_state.config.stale_after_ms))
        {
            b4_proc_abort_hump();
            g_b4_proc_state.b_previous_sample_valid = false;
        }

        g_b4_proc_state.status.sample = *p_sample;
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

        /* The hump-axis inclination carries the hump height, so it is
         * filtered, zeroed against the flat-ground baseline, and kept in a
         * settled-window history before it drives the detector. Only its
         * magnitude is used, so it needs no orientation sign. */
        incline_mg = b4_proc_status_axis_mg(
            &g_b4_proc_state.status,
            g_b4_proc_state.config.hump_axis);
        g_b4_proc_state.status.roll_mg = incline_mg;
        b4_proc_update_attitude();
        b4_proc_push_incline(incline_mg);
        b_settled = b4_proc_incline_is_settled();
        b4_proc_update_baseline(incline_mg, b_settled);
        g_b4_proc_state.status.roll_baseline_mg =
            g_b4_proc_state.incline_baseline_mg;
        b4_proc_update_residuals();
        incline_delta_mg = b4_proc_incline_delta_mg();
        amplitude_mg = b4_proc_abs_mg(incline_delta_mg);
        g_b4_proc_state.status.incline_delta_mg = incline_delta_mg;
        b4_proc_update_noise(amplitude_mg);

        if (false == g_b4_proc_state.status.b_impact_event)
        {
            b4_proc_advance_hump(amplitude_mg, b_settled);
            b4_proc_update_height();
        }
        else if ((B4_HUMP_NORMAL != g_b4_proc_state.status.hump_state) &&
                 (B4_HUMP_COMPLETE != g_b4_proc_state.status.hump_state))
        {
            /* The spike is already in the filter and the height window, so
             * this crossing's height cannot be trusted. */
            g_b4_proc_state.b_active_height_tainted = true;
        }
        else
        {
            /* No hump in progress: the spike only delays the next one. */
        }

        b4_proc_update_level(amplitude_mg);
        b4_proc_update_launch(amplitude_mg);
        b4_proc_update_motion(sample_delta_mg);
        g_b4_proc_state.previous_timestamp_ms = p_sample->timestamp_ms;
        g_b4_proc_state.previous_x_mg = p_sample->accel_x_mg;
        g_b4_proc_state.previous_y_mg = p_sample->accel_y_mg;
        g_b4_proc_state.previous_z_mg = p_sample->accel_z_mg;
        g_b4_proc_state.b_previous_sample_valid = true;
    }

    return (result);
}

/** @brief Process one magnetometer sample and update turning classification.
 *
 * @param p_mag_sample The magnetometer sample, which may be invalid when the
 *                     read itself failed.
 * @return The processing result.
 */
b4_types_result_t
b4_proc_process_magnetometer (const b4_types_mag_sample_t * p_mag_sample)
{
    b4_types_result_t result = B4_RESULT_OK;

    if (NULL == p_mag_sample)
    {
        result = B4_RESULT_INVALID_ARGUMENT;
    }
    else if (false == g_b4_proc_state.b_initialized)
    {
        result = B4_RESULT_NOT_INITIALIZED;
    }
    else
    {
        g_b4_proc_state.status.mag_sample = *p_mag_sample;

        if (false == p_mag_sample->b_valid)
        {
            g_b4_proc_state.status.b_mag_valid = false;
            g_b4_proc_state.status.turn = B4_TURN_UNKNOWN;
            g_b4_proc_state.b_mag_previous_valid = false;
            result = B4_RESULT_INVALID_SAMPLE;
        }
        else
        {
            if (true == g_b4_proc_state.b_mag_previous_valid)
            {
                /* The cross product of two horizontal field vectors is
                 * positive for a counter-clockwise (left) heading change on
                 * a level car, independent of any constant hard-iron
                 * offset. A slow leak keeps the sum centred while the car
                 * drives straight. */
                int32_t cross = ((int32_t)g_b4_proc_state.mag_previous_x *
                                 (int32_t)p_mag_sample->mag_y) -
                                ((int32_t)p_mag_sample->mag_x *
                                 (int32_t)g_b4_proc_state.mag_previous_y);
                int64_t divisor = (int64_t)
                    g_b4_proc_state.config.turn_window_samples;

                g_b4_proc_state.turn_cross_accum -=
                    (g_b4_proc_state.turn_cross_accum / divisor);
                g_b4_proc_state.turn_cross_accum += (int64_t)cross;
            }

            g_b4_proc_state.mag_previous_x = p_mag_sample->mag_x;
            g_b4_proc_state.mag_previous_y = p_mag_sample->mag_y;
            g_b4_proc_state.b_mag_previous_valid = true;
            g_b4_proc_state.b_mag_seen = true;
            g_b4_proc_state.mag_last_timestamp_ms = p_mag_sample->timestamp_ms;
            g_b4_proc_state.status.b_mag_valid = true;
            g_b4_proc_state.status.turn_cross = b4_proc_clamp_cross(
                g_b4_proc_state.turn_cross_accum);

            if (g_b4_proc_state.turn_cross_accum >
                (int64_t)g_b4_proc_state.config.turn_cross_threshold)
            {
                g_b4_proc_state.status.turn =
                    (0 < g_b4_proc_state.config.turn_sign) ?
                    B4_TURN_LEFT : B4_TURN_RIGHT;
            }
            else if (g_b4_proc_state.turn_cross_accum <
                     -(int64_t)g_b4_proc_state.config.turn_cross_threshold)
            {
                g_b4_proc_state.status.turn =
                    (0 < g_b4_proc_state.config.turn_sign) ?
                    B4_TURN_RIGHT : B4_TURN_LEFT;
            }
            else
            {
                g_b4_proc_state.status.turn = B4_TURN_STRAIGHT;
            }
        }
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

        /* A heading that stopped updating is not a heading. */
        if ((false == g_b4_proc_state.b_mag_seen) ||
            ((now_ms - g_b4_proc_state.mag_last_timestamp_ms) >
             g_b4_proc_state.config.mag_stale_after_ms))
        {
            p_status->b_mag_valid = false;
            p_status->turn = B4_TURN_UNKNOWN;
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
        g_b4_proc_state.status.hump_peak_height_um = 0U;
        g_b4_proc_state.status.highest_hump_height_um = 0U;
        g_b4_proc_state.status.b_hump_height_valid = false;
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

        /* A reset also discards the attitude reference and the settled
         * window, because a calibration between runs changes the sensor
         * bias and would otherwise leave a height offset in the baseline. */
        g_b4_proc_state.incline_baseline_mg = 0;
        g_b4_proc_state.incline_window_count = 0U;
        g_b4_proc_state.incline_window_next = 0U;
        g_b4_proc_state.b_incline_baseline_valid = false;
        g_b4_proc_state.level_count = 0U;
        g_b4_proc_state.b_level_since_gap = false;
        g_b4_proc_state.launch_settle_count = 0U;
        g_b4_proc_state.b_launch_settled = false;
        g_b4_proc_state.noise_warmup_count = 0U;
        g_b4_proc_state.noise_q8 =
            (int32_t)g_b4_proc_state.config.noise_floor_mg *
            B4_PROC_NOISE_SCALE;
        b4_proc_refresh_thresholds();
        g_b4_proc_state.status.noise_mg = 0U;
        g_b4_proc_state.status.incline_delta_mg = 0;
        g_b4_proc_state.status.detectable_height_um = 0U;
        g_b4_proc_state.turn_cross_accum = 0;
        g_b4_proc_state.b_mag_previous_valid = false;
        g_b4_proc_state.status.roll_mg = 0;
        g_b4_proc_state.status.roll_baseline_mg = 0;
        g_b4_proc_state.status.pitch_mg = 0;
        g_b4_proc_state.status.pitch_baseline_mg = 0;
        g_b4_proc_state.lateral_mg = 0;
        g_b4_proc_state.lateral_baseline_mg = 0;
        g_b4_proc_state.magnitude_baseline_mg = 0U;
        b4_proc_clear_residuals();
        g_b4_proc_state.status.turn_cross = 0;
        g_b4_proc_state.status.turn = B4_TURN_UNKNOWN;
    }

    return (result);
}

/** @brief Clear the run maximum and the latest result for a new run.
 *
 * The flat-ground baseline and level confirmation are kept, so a car that
 * is ready at the start line stays ready. The launch hold and the noise
 * warm-up restart, so the launch itself is measured as noise and no hump
 * can start until the warm-up and a steady settle have passed. The noise
 * estimate is kept as the best guess for the first moving samples.
 *
 * @return The clear result.
 */
b4_types_result_t
b4_proc_clear_run_maximum (void)
{
    b4_types_result_t result = B4_RESULT_OK;

    if (false == g_b4_proc_state.b_initialized)
    {
        result = B4_RESULT_NOT_INITIALIZED;
    }
    else
    {
        g_b4_proc_state.status.highest_hump_peak_mg = 0U;
        g_b4_proc_state.status.highest_hump_height_um = 0U;
        g_b4_proc_state.status.hump_peak_height_um = 0U;
        g_b4_proc_state.status.b_hump_height_valid = false;
        g_b4_proc_state.launch_settle_count = 0U;
        g_b4_proc_state.b_launch_settled = false;
        g_b4_proc_state.noise_warmup_count = 0U;
    }

    return (result);
}

b4_types_result_t
b4_proc_calibrate_level (void)
{
    b4_types_result_t result = B4_RESULT_OK;

    if (false == g_b4_proc_state.b_initialized)
    {
        result = B4_RESULT_NOT_INITIALIZED;
    }
    else
    {
        /* The same restart as after a data gap, plus the baseline: it is
         * captured again from the next settled window, so a car put down
         * at a new tilt gets a new level reference. */
        g_b4_proc_state.incline_window_count = 0U;
        g_b4_proc_state.incline_window_next = 0U;
        g_b4_proc_state.incline_baseline_mg = 0;
        g_b4_proc_state.b_incline_baseline_valid = false;
        g_b4_proc_state.level_count = 0U;
        g_b4_proc_state.b_level_since_gap = false;
        g_b4_proc_state.launch_settle_count = 0U;
        g_b4_proc_state.b_launch_settled = false;
        g_b4_proc_state.cal_sum_mg = 0U;
        g_b4_proc_state.cal_count = 0U;
        g_b4_proc_state.b_cal_measuring = false;
        b4_proc_clear_residuals();
    }

    return (result);
}

b4_types_result_t
b4_proc_calibrate_measure (void)
{
    b4_types_result_t result = B4_RESULT_OK;

    if (false == g_b4_proc_state.b_initialized)
    {
        result = B4_RESULT_NOT_INITIALIZED;
    }
    else
    {
        g_b4_proc_state.cal_sum_mg = 0U;
        g_b4_proc_state.cal_count = 0U;
        g_b4_proc_state.b_cal_measuring = true;
    }

    return (result);
}

uint32_t
b4_proc_calibrate_samples (void)
{
    return (g_b4_proc_state.cal_count);
}

b4_types_result_t
b4_proc_calibrate_apply (uint32_t min_samples, uint32_t * p_floor_mg)
{
    b4_types_result_t result = B4_RESULT_INVALID_ARGUMENT;
    uint32_t old_floor_mg = 0U;
    uint32_t floor_mg = 0U;

    if (NULL != p_floor_mg)
    {
        *p_floor_mg = 0U;
        result = B4_RESULT_NOT_INITIALIZED;

        if (true == g_b4_proc_state.b_initialized)
        {
            g_b4_proc_state.b_cal_measuring = false;
            result = B4_RESULT_TIMEOUT;

            if ((0U != g_b4_proc_state.cal_count) &&
                (g_b4_proc_state.cal_count >= min_samples))
            {
                /* The sum is at most 8000 milli-g times the sample cap,
                 * 1.6e8, so the rounded mean cannot overflow. */
                floor_mg = (g_b4_proc_state.cal_sum_mg +
                            (g_b4_proc_state.cal_count / 2U)) /
                           g_b4_proc_state.cal_count;
                if (0U == floor_mg)
                {
                    /* A zero floor is invalid and a flat trace is not
                     * a reason to fail: take the smallest floor. */
                    floor_mg = 1U;
                }
                *p_floor_mg = floor_mg;

                old_floor_mg = g_b4_proc_state.config.noise_floor_mg;
                g_b4_proc_state.config.noise_floor_mg = floor_mg;
                result = b4_cfg_validate(&g_b4_proc_state.config);

                if (B4_RESULT_OK == result)
                {
                    /* The estimate starts at the new floor, as at init. */
                    g_b4_proc_state.noise_q8 =
                        (int32_t)floor_mg * B4_PROC_NOISE_SCALE;
                    b4_proc_refresh_thresholds();
                }
                else
                {
                    g_b4_proc_state.config.noise_floor_mg = old_floor_mg;
                }
            }
        }
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

static uint32_t
b4_proc_difference_mg (int32_t left_mg, int32_t right_mg)
{
    uint32_t difference_mg = 0U;

    difference_mg = b4_proc_abs_mg(left_mg - right_mg);

    return (difference_mg);
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

static void
b4_proc_update_attitude (void)
{
    uint8_t slot = g_b4_proc_state.incline_window_next;
    int32_t forward_mg = b4_proc_status_axis_mg(
        &g_b4_proc_state.status,
        g_b4_proc_state.config.forward_axis);
    int32_t lateral_mg = b4_proc_status_axis_mg(
        &g_b4_proc_state.status,
        g_b4_proc_state.config.lateral_axis);
    int32_t vertical_mg = b4_proc_status_axis_mg(
        &g_b4_proc_state.status,
        g_b4_proc_state.config.vertical_axis);

    /* The three history slots hold the sensor axes before orientation. The
     * filter output stays within the 4000 milli-g sample limit, so the
     * narrowing conversions to int16_t are exact. */
    g_b4_proc_state.forward_window_mg[slot] = (int16_t)forward_mg;
    g_b4_proc_state.lateral_window_mg[slot] = (int16_t)lateral_mg;
    g_b4_proc_state.vertical_window_mg[slot] = (int16_t)vertical_mg;

    g_b4_proc_state.status.pitch_mg = b4_proc_oriented_value(
        forward_mg, g_b4_proc_state.config.forward_sign);
    g_b4_proc_state.lateral_mg = b4_proc_oriented_value(
        lateral_mg, g_b4_proc_state.config.lateral_sign);
}

static int32_t
b4_proc_axis_mean_mg (const int16_t * p_window, uint8_t window_samples)
{
    uint8_t index = g_b4_proc_state.incline_window_next;
    int32_t sum_mg = 0;

    /* Start from the oldest of the newest window_samples entries, exactly as
     * the incline mean does. Each entry is within 4000 milli-g and the
     * window holds at most 100 entries, so the sum stays within 400000. */
    index = (uint8_t)((index + B4_CFG_MAX_SETTLE_SAMPLES - window_samples) %
                      B4_CFG_MAX_SETTLE_SAMPLES);

    for (uint8_t sample = 0U; sample < window_samples; sample++)
    {
        sum_mg += p_window[index];
        index = (uint8_t)((index + 1U) % B4_CFG_MAX_SETTLE_SAMPLES);
    }

    /* Validation keeps window_samples between 2 and 100, so the divisor is
     * never zero and fits int32_t. */
    return (sum_mg / (int32_t)window_samples);
}

static uint32_t
b4_proc_isqrt (uint64_t value)
{
    uint64_t remainder = value;
    uint64_t root = 0U;
    uint64_t bit = (uint64_t)1U << 62;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (0U != bit)
    {
        if (remainder >= (root + bit))
        {
            remainder -= (root + bit);
            root = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }

        bit >>= 2;
    }

    /* The caller passes a sum of three squares of values within 4000, so
     * the root is below 7000 and the narrowing conversion is exact. */
    return ((uint32_t)root);
}

static void
b4_proc_step_toward (int32_t * p_baseline, int32_t value_mg)
{
    if (value_mg > *p_baseline)
    {
        (*p_baseline)++;
    }
    else if (value_mg < *p_baseline)
    {
        (*p_baseline)--;
    }
    else
    {
        /* Already on the baseline. */
    }
}

static void
b4_proc_capture_attitude_baselines (void)
{
    uint8_t window_samples = g_b4_proc_state.config.height_settle_samples;
    int64_t forward_mg = b4_proc_axis_mean_mg(
        g_b4_proc_state.forward_window_mg, window_samples);
    int64_t lateral_mg = b4_proc_axis_mean_mg(
        g_b4_proc_state.lateral_window_mg, window_samples);
    int64_t vertical_mg = b4_proc_axis_mean_mg(
        g_b4_proc_state.vertical_window_mg, window_samples);

    /* The means are within 4000 milli-g, so the int32_t narrowing is exact
     * and the sum of squares fits int64_t. */
    g_b4_proc_state.status.pitch_baseline_mg = b4_proc_oriented_value(
        (int32_t)forward_mg, g_b4_proc_state.config.forward_sign);
    g_b4_proc_state.lateral_baseline_mg = b4_proc_oriented_value(
        (int32_t)lateral_mg, g_b4_proc_state.config.lateral_sign);
    g_b4_proc_state.magnitude_baseline_mg = b4_proc_isqrt(
        (uint64_t)((forward_mg * forward_mg) +
                   (lateral_mg * lateral_mg) +
                   (vertical_mg * vertical_mg)));
}

static void
b4_proc_clear_residuals (void)
{
    g_b4_proc_state.status.roll_change_mg = 0;
    g_b4_proc_state.status.tilt_residual_mg = 0;
    g_b4_proc_state.status.magnitude_residual_mg = 0;
    g_b4_proc_state.status.b_tilt_consistent = false;
    g_b4_proc_state.tilt_consistent_count = 0U;
}

static void
b4_proc_update_residuals (void)
{
    uint8_t mean_samples = g_b4_proc_state.config.height_mean_samples;
    int64_t forward_mean_mg = 0;
    int64_t lateral_mean_mg = 0;
    int64_t vertical_mean_mg = 0;
    int64_t roll_mg = 0;
    int64_t roll_abs_mg = 0;
    int64_t pitch_change_mg = 0;
    int64_t predicted_mg = 0;
    int64_t numerator = 0;
    int64_t denominator = 1;
    int64_t magnitude_mg = 0;
    int64_t sum_squares = 0;
    bool b_consistent = false;

    /* NOTE: The check uses the means of the newest height_mean_samples
     * entries, not the instantaneous filtered values. Vibration swings both
     * ways and mostly cancels over the window, while a snag pushes one way
     * for longer and survives the average. Until the windows hold that many
     * entries the sample counts as not consistent. */
    if ((false == g_b4_proc_state.b_incline_baseline_valid) ||
        (g_b4_proc_state.incline_window_count < mean_samples))
    {
        b4_proc_clear_residuals();
    }
    else
    {
        forward_mean_mg = b4_proc_axis_mean_mg(
            g_b4_proc_state.forward_window_mg, mean_samples);
        lateral_mean_mg = b4_proc_axis_mean_mg(
            g_b4_proc_state.lateral_window_mg, mean_samples);
        vertical_mean_mg = b4_proc_axis_mean_mg(
            g_b4_proc_state.vertical_window_mg, mean_samples);

        /* Every input is within 4000 milli-g and every geometry value is
         * below 300000 micrometres, so the int64_t products cannot
         * overflow. The micrometre units cancel in the quotient. */
        roll_mg = (int64_t)b4_proc_oriented_value(
                      (int32_t)lateral_mean_mg,
                      g_b4_proc_state.config.lateral_sign) -
                  (int64_t)g_b4_proc_state.lateral_baseline_mg;
        pitch_change_mg = (int64_t)b4_proc_oriented_value(
                              (int32_t)forward_mean_mg,
                              g_b4_proc_state.config.forward_sign) -
                          (int64_t)g_b4_proc_state.status.pitch_baseline_mg;
        roll_abs_mg = (roll_mg < 0) ? -roll_mg : roll_mg;

        /* NOTE: Predicted pitch is -(|r| * track / 2 + r * caster_lateral)
         * over the caster forward distance. The factor of two moves into
         * the divisor so integer division loses no half unit. */
        numerator = (roll_abs_mg *
                     (int64_t)g_b4_proc_state.config.hump_lever_um) +
                    (2 * roll_mg *
                     (int64_t)g_b4_proc_state.config.caster_lateral_um);
        denominator = 2 * (int64_t)g_b4_proc_state.config.caster_forward_um;
        predicted_mg = -(numerator / denominator);

        sum_squares = (forward_mean_mg * forward_mean_mg) +
                      (lateral_mean_mg * lateral_mean_mg) +
                      (vertical_mean_mg * vertical_mean_mg);
        magnitude_mg = (int64_t)b4_proc_isqrt((uint64_t)sum_squares) -
                       (int64_t)g_b4_proc_state.magnitude_baseline_mg;

        /* The values are bounded by a few times 4000 milli-g, so the
         * narrowing conversions to int32_t are exact. */
        g_b4_proc_state.status.roll_change_mg = (int32_t)roll_mg;
        g_b4_proc_state.status.tilt_residual_mg =
            (int32_t)(pitch_change_mg - predicted_mg);
        g_b4_proc_state.status.magnitude_residual_mg = (int32_t)magnitude_mg;

        b_consistent =
            (b4_proc_abs_mg(g_b4_proc_state.status.tilt_residual_mg) <=
             g_b4_proc_state.config.tilt_residual_band_mg) &&
            (b4_proc_abs_mg(g_b4_proc_state.status.magnitude_residual_mg) <=
             g_b4_proc_state.config.tilt_magnitude_band_mg);
        g_b4_proc_state.status.b_tilt_consistent = b_consistent;

        if (true == b_consistent)
        {
            if (g_b4_proc_state.tilt_consistent_count < B4_PROC_MAX_COUNTER)
            {
                g_b4_proc_state.tilt_consistent_count++;
            }
        }
        else
        {
            g_b4_proc_state.tilt_consistent_count = 0U;
        }
    }
}

static void
b4_proc_push_incline (int32_t incline_mg)
{
    g_b4_proc_state.incline_window_mg[g_b4_proc_state.incline_window_next] =
        incline_mg;
    /* The modulo result is below the 100-entry window size, so this
     * narrowing conversion is exact. */
    g_b4_proc_state.incline_window_next = (uint8_t)(
        (g_b4_proc_state.incline_window_next + 1U) %
        B4_CFG_MAX_SETTLE_SAMPLES);

    if (g_b4_proc_state.incline_window_count < B4_CFG_MAX_SETTLE_SAMPLES)
    {
        g_b4_proc_state.incline_window_count++;
    }
}

static uint32_t
b4_proc_incline_spread_mg (void)
{
    uint8_t window_samples = g_b4_proc_state.config.height_settle_samples;
    uint8_t index = g_b4_proc_state.incline_window_next;
    int32_t min_mg = 0;
    int32_t max_mg = 0;

    /* Start from the oldest of the newest window_samples entries. Both the
     * index sum and the modulo result are below the 100-entry window size,
     * so this narrowing conversion is exact. */
    index = (uint8_t)((index + B4_CFG_MAX_SETTLE_SAMPLES - window_samples) %
                      B4_CFG_MAX_SETTLE_SAMPLES);

    for (uint8_t sample = 0U; sample < window_samples; sample++)
    {
        int32_t incline_mg = g_b4_proc_state.incline_window_mg[index];

        if (0U == sample)
        {
            min_mg = incline_mg;
            max_mg = incline_mg;
        }
        else if (incline_mg < min_mg)
        {
            min_mg = incline_mg;
        }
        else if (incline_mg > max_mg)
        {
            max_mg = incline_mg;
        }

        index = (uint8_t)((index + 1U) % B4_CFG_MAX_SETTLE_SAMPLES);
    }

    return (b4_proc_difference_mg(max_mg, min_mg));
}

static int32_t
b4_proc_incline_mean_mg (uint8_t window_samples)
{
    uint8_t index = g_b4_proc_state.incline_window_next;
    int32_t sum_mg = 0;

    /* Start from the oldest of the newest window_samples entries, exactly as
     * the spread does. Each entry is within 4000 milli-g and the window holds
     * at most 100 entries, so the sum stays within 400000. */
    index = (uint8_t)((index + B4_CFG_MAX_SETTLE_SAMPLES - window_samples) %
                      B4_CFG_MAX_SETTLE_SAMPLES);

    for (uint8_t sample = 0U; sample < window_samples; sample++)
    {
        sum_mg += g_b4_proc_state.incline_window_mg[index];
        index = (uint8_t)((index + 1U) % B4_CFG_MAX_SETTLE_SAMPLES);
    }

    /* Validation keeps window_samples between 2 and 100, so the divisor is
     * never zero and fits int32_t. */
    return (sum_mg / (int32_t)window_samples);
}

static bool
b4_proc_incline_is_settled (void)
{
    bool b_settled = false;
    uint32_t spread_mg = 0U;

    if (g_b4_proc_state.incline_window_count >=
        g_b4_proc_state.config.height_settle_samples)
    {
        spread_mg = b4_proc_incline_spread_mg();
        if (spread_mg <= g_b4_proc_state.config.height_settle_band_mg)
        {
            b_settled = true;
        }
    }

    return (b_settled);
}

static void
b4_proc_update_baseline (int32_t incline_mg, bool b_settled)
{
    if (false == g_b4_proc_state.b_incline_baseline_valid)
    {
        if (true == b_settled)
        {
            g_b4_proc_state.incline_baseline_mg = b4_proc_incline_mean_mg(
                g_b4_proc_state.config.height_settle_samples);
            b4_proc_capture_attitude_baselines();
            g_b4_proc_state.b_incline_baseline_valid = true;
        }
    }
    else if ((true == b_settled) &&
             (B4_HUMP_NORMAL == g_b4_proc_state.status.hump_state))
    {
        uint32_t delta_mg = b4_proc_difference_mg(
            incline_mg, g_b4_proc_state.incline_baseline_mg);

        if (delta_mg <= g_b4_proc_state.config.height_baseline_gate_mg)
        {
            /* NOTE: Step 1 mg once per divisor settled samples. Dividing
             * the gated delta instead always rounded to zero. */
            g_b4_proc_state.baseline_step_count++;

            if (g_b4_proc_state.baseline_step_count >=
                g_b4_proc_state.config.height_baseline_divisor)
            {
                g_b4_proc_state.baseline_step_count = 0U;
                b4_proc_step_toward(&g_b4_proc_state.status.pitch_baseline_mg,
                                    g_b4_proc_state.status.pitch_mg);
                b4_proc_step_toward(&g_b4_proc_state.lateral_baseline_mg,
                                    g_b4_proc_state.lateral_mg);

                if (incline_mg > g_b4_proc_state.incline_baseline_mg)
                {
                    g_b4_proc_state.incline_baseline_mg++;
                }
                else if (incline_mg < g_b4_proc_state.incline_baseline_mg)
                {
                    g_b4_proc_state.incline_baseline_mg--;
                }
                else
                {
                    /* Already on the baseline. */
                }
            }
        }
    }
    else
    {
        /* Settled off baseline, or a hump is in progress: the reference
         * must not follow the climb. */
    }
}

static int32_t
b4_proc_incline_delta_mg (void)
{
    int32_t delta_mg = 0;
    uint8_t mean_samples = g_b4_proc_state.config.height_mean_samples;

    /* NOTE: The offset is the mean of the newest height_mean_samples
     * entries, the same value the height uses, so a brief single-wheel
     * wobble cannot reach the hump thresholds. Until that many entries
     * exist the offset is zero, as with no baseline. */
    if ((true == g_b4_proc_state.b_incline_baseline_valid) &&
        (g_b4_proc_state.incline_window_count >= mean_samples))
    {
        delta_mg = b4_proc_incline_mean_mg(mean_samples) -
                   g_b4_proc_state.incline_baseline_mg;
    }

    return (delta_mg);
}

static uint32_t
b4_proc_height_um (int32_t delta_mg)
{
    /* Validation bounds the lever to 300 mm, and the offset between two
     * values within 4000 milli-g stays within 8000, so the product stays
     * below 2.4e9 in int64_t and the quotient fits uint32_t. The quotient is
     * made non-negative below before the conversion. */
    int64_t height_um = ((int64_t)g_b4_proc_state.config.hump_lever_um *
                         (int64_t)delta_mg) /
                        (int64_t)g_b4_proc_state.config.gravity_reference_mg;

    if (height_um < 0)
    {
        height_um = -height_um;
    }

    return ((uint32_t)height_um);
}

static void
b4_proc_update_height (void)
{
    uint8_t mean_samples = g_b4_proc_state.config.height_mean_samples;

    if ((true == g_b4_proc_state.b_incline_baseline_valid) &&
        (g_b4_proc_state.incline_window_count >= mean_samples) &&
        (g_b4_proc_state.tilt_consistent_count >= mean_samples) &&
        (B4_HUMP_NORMAL != g_b4_proc_state.status.hump_state) &&
        (B4_HUMP_COMPLETE != g_b4_proc_state.status.hump_state))
    {
        int32_t delta_mg = b4_proc_incline_mean_mg(mean_samples) -
                           g_b4_proc_state.incline_baseline_mg;
        uint32_t height_um = b4_proc_height_um(delta_mg);

        if (height_um > g_b4_proc_state.active_peak_height_um)
        {
            g_b4_proc_state.active_peak_height_um = height_um;
        }

        g_b4_proc_state.b_active_height_measured = true;
        g_b4_proc_state.status.active_hump_height_um =
            g_b4_proc_state.active_peak_height_um;
    }
}

static void
b4_proc_abort_hump (void)
{
    b4_types_hump_state_t state = g_b4_proc_state.status.hump_state;

    /* A candidate is not yet a hump, so only a confirmed one produces an
     * invalid result. The run maximum is never touched here. */
    if ((B4_HUMP_ASCENDING == state) ||
        (B4_HUMP_PEAK == state) ||
        (B4_HUMP_DESCENDING == state))
    {
        g_b4_proc_state.status.hump_peak_height_um = 0U;
        g_b4_proc_state.status.b_hump_height_valid = false;
    }

    b4_proc_reset_active_hump();

    /* Filter and moving-mean history from before the break would blend
     * old and new attitudes, so both restart. The baseline is kept. */
    g_b4_proc_state.b_filter_ready = false;
    g_b4_proc_state.incline_window_count = 0U;
    g_b4_proc_state.incline_window_next = 0U;
    g_b4_proc_state.level_count = 0U;
    g_b4_proc_state.b_level_since_gap = false;
    g_b4_proc_state.launch_settle_count = 0U;
    g_b4_proc_state.b_launch_settled = false;
    g_b4_proc_state.noise_warmup_count = 0U;
    b4_proc_clear_residuals();
}

static uint32_t
b4_proc_noise_mg (void)
{
    /* The estimate is a weighted mean of non-negative amplitudes, so it is
     * never negative and the unsigned conversion is exact. Adding half a
     * scale step rounds to the nearest milli-g. */
    uint32_t noise_mg = (uint32_t)(g_b4_proc_state.noise_q8 +
                                   B4_PROC_NOISE_HALF_SCALE) /
                        (uint32_t)B4_PROC_NOISE_SCALE;

    if (noise_mg < g_b4_proc_state.config.noise_floor_mg)
    {
        noise_mg = g_b4_proc_state.config.noise_floor_mg;
    }

    return (noise_mg);
}

static void
b4_proc_refresh_thresholds (void)
{
    uint32_t noise_mg = b4_proc_noise_mg();
    uint32_t start_mg = 0U;
    uint32_t complete_mg = 0U;

    /* The estimate is within the 8000 milli-g span of two samples and the
     * factors are validated to at most 200 tenths, so each product stays
     * below 1.6e6 and fits uint32_t. */
    start_mg = (noise_mg * g_b4_proc_state.config.hump_start_k_tenths) /
               B4_PROC_K_TENTHS_PER_ONE;
    complete_mg = (noise_mg *
                   g_b4_proc_state.config.hump_complete_k_tenths) /
                  B4_PROC_K_TENTHS_PER_ONE;

    /* A zero threshold would count every sample as a hump. */
    if (start_mg < B4_PROC_MIN_THRESHOLD_MG)
    {
        start_mg = B4_PROC_MIN_THRESHOLD_MG;
    }

    if (complete_mg < B4_PROC_MIN_THRESHOLD_MG)
    {
        complete_mg = B4_PROC_MIN_THRESHOLD_MG;
    }

    g_b4_proc_state.start_threshold_mg = start_mg;
    g_b4_proc_state.complete_threshold_mg = complete_mg;
    g_b4_proc_state.status.noise_mg = noise_mg;
    /* The start threshold is at most 8000 times 20, or 160000 milli-g, so
     * the conversion cast is exact and the height product stays within
     * int64_t. */
    g_b4_proc_state.status.detectable_height_um =
        b4_proc_height_um((int32_t)start_mg);
}

static void
b4_proc_update_noise (uint32_t amplitude_mg)
{
    uint8_t mean_samples = g_b4_proc_state.config.height_mean_samples;

    /* NOTE: The input is clipped at the completion threshold, which is
     * k_complete times the estimate, instead of being rejected above it.
     * The clip bounds the growth per sample to (k_complete - 1) divided
     * by noise_rise_samples, about 0.1 percent, so one second of a ramp
     * edge raises the threshold by about 10 percent at most. The rise is
     * slower than the fall, so a rising edge or a snag cannot pull the
     * threshold up, while a rougher surface is still learned within
     * seconds. The clip is a multiple above the estimate, so the estimate
     * cannot lock low. */
    if ((true == g_b4_proc_state.b_incline_baseline_valid) &&
        (g_b4_proc_state.incline_window_count >= mean_samples) &&
        (B4_HUMP_NORMAL == g_b4_proc_state.status.hump_state) &&
        (false == g_b4_proc_state.status.b_impact_event))
    {
        uint32_t input_mg = amplitude_mg;
        int32_t tau = 0;
        int32_t target_q8 = 0;
        int32_t floor_q8 = 0;

        /* The calibration mean takes the same samples as the estimate
         * and is not clipped: it must learn the floor from the sweep,
         * not from the old floor. */
        if ((true == g_b4_proc_state.b_cal_measuring) &&
            (g_b4_proc_state.cal_count < B4_PROC_CAL_MAX_SAMPLES))
        {
            g_b4_proc_state.cal_sum_mg += amplitude_mg;
            g_b4_proc_state.cal_count++;
        }

        if (input_mg > g_b4_proc_state.complete_threshold_mg)
        {
            input_mg = g_b4_proc_state.complete_threshold_mg;
        }

        /* The amplitude is the difference of two means within the 4000
         * milli-g sample range, so it is at most 8000 and the scaled
         * target is at most 2.05e6. The estimate stays between the floor
         * and the largest target, so the difference stays within
         * +/-2.05e6 and the sum cannot overflow int32_t. The time
         * constants are validated to be 1 to 10000, so the divisor is
         * never zero. */
        target_q8 = (int32_t)input_mg * B4_PROC_NOISE_SCALE;

        if (target_q8 > g_b4_proc_state.noise_q8)
        {
            tau = (int32_t)g_b4_proc_state.config.noise_rise_samples;
        }
        else
        {
            tau = (int32_t)g_b4_proc_state.config.noise_fall_samples;
        }

        g_b4_proc_state.noise_q8 +=
            (target_q8 - g_b4_proc_state.noise_q8) / tau;

        /* The floor is validated to at most the settle band limit, so the
         * scaled floor is small and the cast is exact. */
        floor_q8 = (int32_t)g_b4_proc_state.config.noise_floor_mg *
                   B4_PROC_NOISE_SCALE;

        if (g_b4_proc_state.noise_q8 < floor_q8)
        {
            g_b4_proc_state.noise_q8 = floor_q8;
        }
    }

    /* The warm-up counts samples since the run started, not accepted
     * ones, so a rough launch cannot hold detection off forever. */
    if (g_b4_proc_state.noise_warmup_count < B4_PROC_MAX_COUNTER)
    {
        g_b4_proc_state.noise_warmup_count++;
    }

    b4_proc_refresh_thresholds();
}

static void
b4_proc_update_level (uint32_t amplitude_mg)
{
    b4_types_hump_state_t state = g_b4_proc_state.status.hump_state;
    bool b_level =
        (true == g_b4_proc_state.b_incline_baseline_valid) &&
        ((B4_HUMP_NORMAL == state) || (B4_HUMP_COMPLETE == state)) &&
        (amplitude_mg <= g_b4_proc_state.complete_threshold_mg);

    if (true == b_level)
    {
        if (g_b4_proc_state.level_count < B4_PROC_MAX_COUNTER)
        {
            g_b4_proc_state.level_count++;
        }
    }
    else
    {
        g_b4_proc_state.level_count = 0U;
    }

    /* Once confirmed, a later hump does not withdraw readiness. Only a
     * break in the sample stream does, through b4_proc_abort_hump. */
    if (g_b4_proc_state.level_count >=
        g_b4_proc_state.config.hump_complete_samples)
    {
        g_b4_proc_state.b_level_since_gap = true;
    }

    if (true == g_b4_proc_state.b_level_since_gap)
    {
        g_b4_proc_state.status.health = B4_HEALTH_READY;
    }
    else
    {
        g_b4_proc_state.status.health = B4_HEALTH_DEGRADED;
    }
}

static void
b4_proc_update_launch (uint32_t amplitude_mg)
{
    b4_types_hump_state_t state = g_b4_proc_state.status.hump_state;
    bool b_steady =
        (true == g_b4_proc_state.status.b_tilt_consistent) &&
        ((B4_HUMP_NORMAL == state) || (B4_HUMP_COMPLETE == state)) &&
        (amplitude_mg <= g_b4_proc_state.complete_threshold_mg);

    if (true == b_steady)
    {
        if (g_b4_proc_state.launch_settle_count < B4_PROC_MAX_COUNTER)
        {
            g_b4_proc_state.launch_settle_count++;
        }
    }
    else
    {
        g_b4_proc_state.launch_settle_count = 0U;
    }

    /* Once settled the flag stays set until the next run start or data
     * gap, so a later crossing is never held back by its own tilt. */
    if (g_b4_proc_state.launch_settle_count >=
        g_b4_proc_state.config.height_mean_samples)
    {
        g_b4_proc_state.b_launch_settled = true;
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
    g_b4_proc_state.active_peak_height_um = 0U;
    g_b4_proc_state.status.active_hump_height_um = 0U;
    g_b4_proc_state.b_active_height_measured = false;
    g_b4_proc_state.b_active_height_tainted = false;
}

static void
b4_proc_advance_hump (uint32_t amplitude_mg, bool b_settled)
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
            b4_proc_update_hump_descending(amplitude_mg, b_settled);
            break;
        case B4_HUMP_COMPLETE:
            b4_proc_reset_active_hump();
            break;
        default:
            b4_proc_reset_active_hump();
            break;
    }
}

/** @brief Start a hump candidate above the threshold from level ground.
 *
 * The current sample must also be consistent with the wheel-lift geometry,
 * the car must have settled since the run started, and the noise estimate
 * must have had its warm-up, so the roll of a launch cannot pass for a
 * ramp.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_start_hump (uint32_t amplitude_mg)
{
    /* NOTE: After a data gap the car may already be partway over a ramp,
     * so a hump may start only from confirmed level ground. Otherwise the
     * rest of the crossing would publish a partial height as valid. */
    if ((true == g_b4_proc_state.b_level_since_gap) &&
        (true == g_b4_proc_state.b_launch_settled) &&
        (g_b4_proc_state.noise_warmup_count >=
         g_b4_proc_state.config.noise_warmup_samples) &&
        (true == g_b4_proc_state.status.b_tilt_consistent) &&
        (amplitude_mg >= g_b4_proc_state.start_threshold_mg))
    {
        g_b4_proc_state.status.current_hump_peak_mg = amplitude_mg;
        g_b4_proc_state.b_active_height_measured = false;
        g_b4_proc_state.b_active_height_tainted = false;
        g_b4_proc_state.active_peak_height_um = 0U;
        g_b4_proc_state.status.hump_state = B4_HUMP_CANDIDATE;
        g_b4_proc_state.hump_confirm_count = 1U;
    }
}

/** @brief Confirm or cancel a candidate hump.
 *
 * A sample below the threshold or inconsistent with the wheel-lift geometry
 * cancels the candidate, so confirmation needs consecutive samples that are
 * both above the threshold and consistent.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_hump_candidate (uint32_t amplitude_mg)
{
    if ((amplitude_mg >=
         g_b4_proc_state.start_threshold_mg) &&
        (true == g_b4_proc_state.status.b_tilt_consistent))
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
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_hump_ascending (uint32_t amplitude_mg)
{
    uint32_t peak_mg =
        g_b4_proc_state.status.current_hump_peak_mg;
    uint32_t hysteresis_mg =
        g_b4_proc_state.complete_threshold_mg;

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
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_hump_peak (uint32_t amplitude_mg)
{
    uint32_t peak_mg =
        g_b4_proc_state.status.current_hump_peak_mg;
    uint32_t hysteresis_mg =
        g_b4_proc_state.complete_threshold_mg;

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
 * @param amplitude_mg The absolute incline offset in milli-g.
 * @param b_settled True when the settled window is quiet enough.
 */
static void
b4_proc_update_hump_descending (uint32_t amplitude_mg, bool b_settled)
{
    bool b_back_down = false;

    if (amplitude_mg >
        g_b4_proc_state.status.current_hump_peak_mg)
    {
        g_b4_proc_state.status.current_hump_peak_mg = amplitude_mg;
    }

    /* NOTE: A car resting between the completion and start thresholds is
     * also back down. Without this, a settled offset in that band, for
     * example a stale baseline, kept the detector descending forever. */
    if (amplitude_mg <=
        g_b4_proc_state.complete_threshold_mg)
    {
        b_back_down = true;
    }
    else if ((true == b_settled) &&
             (amplitude_mg <
              g_b4_proc_state.start_threshold_mg))
    {
        b_back_down = true;
    }
    else
    {
        b_back_down = false;
    }

    if (true == b_back_down)
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

/** @brief Publish hump completion and retain higher recorded peaks.
 */
static void
b4_proc_complete_hump (void)
{
    bool b_height_valid = (g_b4_proc_state.b_active_height_measured &&
                           !g_b4_proc_state.b_active_height_tainted);

    g_b4_proc_state.status.hump_state = B4_HUMP_COMPLETE;
    g_b4_proc_state.status.b_hump_completed_event = true;
    g_b4_proc_state.status.hump_peak_height_um =
        g_b4_proc_state.active_peak_height_um;
    g_b4_proc_state.status.b_hump_height_valid = b_height_valid;

    if (g_b4_proc_state.status.current_hump_peak_mg >
        g_b4_proc_state.status.highest_hump_peak_mg)
    {
        g_b4_proc_state.status.highest_hump_peak_mg =
            g_b4_proc_state.status.current_hump_peak_mg;
    }

    /* An invalid height must never raise the run maximum. */
    if ((true == b_height_valid) &&
        (g_b4_proc_state.active_peak_height_um >
         g_b4_proc_state.status.highest_hump_height_um))
    {
        g_b4_proc_state.status.highest_hump_height_um =
            g_b4_proc_state.active_peak_height_um;
    }
}

static int32_t
b4_proc_clamp_cross (int64_t value)
{
    int32_t result = 0;

    if (value > (int64_t)INT32_MAX)
    {
        result = INT32_MAX;
    }
    else if (value < (int64_t)INT32_MIN)
    {
        result = INT32_MIN;
    }
    else
    {
        /* The range check above proves the conversion is exact. */
        result = (int32_t)value;
    }

    return (result);
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

/*** end of file ***/
