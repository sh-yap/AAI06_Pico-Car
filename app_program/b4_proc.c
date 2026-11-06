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
 * A gap in the sample stream or an invalid sample aborts a hump in progress
 * as invalid, and health stays DEGRADED until fresh level samples confirm
 * the car is back on flat ground.
 *
 * The start, completion, and hysteresis thresholds are multiples of a
 * running median of the incline amplitude, so they follow the rolling
 * noise of the car while it drives instead of a fixed height. The Vehicle
 * Controller tells this module through b4_proc_set_drive() whether the car
 * is driving. The median learns only on those samples, and only while the
 * hump state is NORMAL or CANDIDATE and there is no impact, so a stop or a
 * confirmed hump never trains it. It takes one step per millimetre of wheel
 * travel, not per sample, so the distance a ramp edge can influence it does
 * not depend on speed. The START sweep seeds the median. No hump may start
 * until the median has had 2^n steps, which is 2^n mm of driving since
 * START, where n is the configured step shift.
 *
 * The multiples remain provisional until flat driving traces set them.
 *
 * An impact is a knock that is extreme on all three accelerometer axes. Each
 * axis is high-pass filtered, and the score of a sample is the smallest, over
 * the three axes, of that axis's largest absolute filtered value in the last
 * 40 ms. The score runs the lean form of SPOT (Siffer et al., KDD 2017,
 * "Anomaly Detection in Streams with Extreme Value Theory") with the shape
 * parameter gamma taken as 0. A streaming 98% quantile t of the score is
 * learned, and the threshold is t plus the mean excess over t times
 * ln((k / n) / q). After a warm-up, a score above the threshold flags an
 * impact and is kept out of learning. The detector learns and judges only
 * while the Vehicle Controller says the car is driving and moving. What it
 * learns survives b4_proc_reset_run_stats() and b4_proc_clear_run_maximum()
 * until power-off. The 200 ms refractory still decides which impacts raise
 * an event. A flag while a hump is DESCENDING raises no event, because the
 * crossing's own exit drop would otherwise count.
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
/* The step count saturates here. Validation keeps the learning hold,
 * 2^shift steps, at most 4096, so the hold is always reachable. */
#define B4_PROC_MAX_LEARN_COUNT    65535U
/* One median step per this much wheel travel, in micrometres. */
#define B4_PROC_STEP_TRAVEL_UM      1000U
/* Largest encodable wheel speed, mm/s. It is numerically micrometres per
 * millisecond. */
#define B4_PROC_MAX_SPEED_MM_S      1023
/* Largest sweep mean accepted as a noise seed, milli-g. The seed scaled by
 * 256 is then at most 128000, far inside int32_t. */
#define B4_PROC_MAX_SEED_MG          500U

/* Impact learning. The warm-up is 3 s of driving at 100 Hz, shorter than the
 * calibration size used in the SPOT paper. */
static const uint32_t b4_proc_impact_warmup_samples = 300U;

/* The streaming 98% quantile t is kept in units of 1/50 mg. A step above t
 * moves it up by 49 units and any other step moves it down by 1 unit. It
 * rests where 49 * P(above) = 1 * (1 - P(above)), so P(above) = 1 / 50, which
 * is the 98% quantile the paper uses for t. About 2% of steps are above t. */
static const uint32_t b4_proc_impact_units_per_mg = 50U;
static const uint32_t b4_proc_impact_up_step_x50 = 49U;
static const uint32_t b4_proc_impact_down_step_x50 = 1U;

/* threshold = t + mean_excess * ln((k / n) / q), with k / n = 0.02 and
 * q = 1 / 12000 per sample, so ln(0.02 / q) = ln(240) = 5.48. One expected
 * false flag per 2 minutes at 100 Hz is 1 in 12000 samples, which is the
 * rate taken from the M1 gate. The multiplier is kept as 548 / 100. */
static const uint32_t b4_proc_impact_tail_mult_x100 = 548U;
static const uint32_t b4_proc_impact_tail_div = 100U;

/* The high-pass filter of each axis is one pole,
 * y = a * (y + x - x_previous), with a = exp(-2 * pi * fc / fs). The cutoff
 * fc = 4 Hz at fs = 100 Hz gives a = 0.777, which is 199 / 256. The cutoff is
 * an experiment value. The result held from 2 to 8 Hz in replay. */
static const int32_t b4_proc_impact_hp_coeff = 199;
static const int32_t b4_proc_impact_hp_scale = 256;

/* Indices of the accelerometer axes. */
#define B4_PROC_AXIS_X      0U
#define B4_PROC_AXIS_Y      1U
#define B4_PROC_AXIS_Z      2U
#define B4_PROC_AXIS_COUNT  3U

/* The current sample and the 4 before it, which is 40 ms at 100 Hz. All three
 * axes must have moved within this window. */
#define B4_PROC_IMPACT_WINDOW_SAMPLES   5U

/* The raw samples of each axis pass through a median of this many samples
 * before the impact high-pass filter. The size is an experiment value from an
 * offline replay. */
#define B4_PROC_IMPACT_MEDIAN_SAMPLES   5U

/* The raw hump-axis samples pass through a median of this many samples before
 * the hump detector sees them. The size is an experiment value. A median
 * ignores events shorter than half its window, which is 0.5 s at 100 Hz, and
 * the logged ramps last 0.94 s or more. It is also the largest window of the
 * median helper. */
#define B4_PROC_HUMP_MEDIAN_SAMPLES    50U

/** @brief Learned statistics of the three-axis impact score.
 */
typedef struct
{
    uint32_t t_x50;
    uint32_t excess_sum;
    uint32_t k;
    uint32_t threshold_mg;
} b4_proc_impact_t;

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
    int32_t           incline_baseline_mg;
    int32_t           noise_q8;
    int32_t           drive_speed_mm_s;
    uint32_t          active_peak_height_um;
    uint32_t          learn_count;
    uint32_t          travel_um;
    uint32_t          cal_sum_mg;
    uint32_t          cal_count;
    uint32_t          start_threshold_mg;
    uint32_t          complete_threshold_mg;
    uint32_t          last_impact_timestamp_ms;
    /* Learned impact score statistics. Only b4_proc_init() clears these, so
     * what is learned carries across runs until power-off. excess_sum is
     * multiplied as uint64_t, so the product cannot overflow. */
    b4_proc_impact_t  impact;
    /* High-pass filter state of each axis, scaled by 256, and the last
     * absolute filtered values in milli-g. The run reset clears them. */
    int32_t           impact_hp_x256[B4_PROC_AXIS_COUNT];
    uint16_t          impact_ring_mg[B4_PROC_AXIS_COUNT]
                                    [B4_PROC_IMPACT_WINDOW_SAMPLES];
    uint32_t          previous_timestamp_ms;
    uint32_t          mag_last_timestamp_ms;
    int64_t           turn_cross_accum;
    int16_t           mag_previous_x;
    int16_t           mag_previous_y;
    /* Raw samples ahead of the impact filter, and the median of the previous
     * sample. The fill count is shared by the three axes. Only the newest
     * count entries are read, so a restart clears the count, not the data. */
    int16_t           impact_raw_mg[B4_PROC_AXIS_COUNT]
                                   [B4_PROC_IMPACT_MEDIAN_SAMPLES];
    int16_t           impact_median_mg[B4_PROC_AXIS_COUNT];
    /* Raw hump-axis samples, read the same way. */
    int16_t           hump_raw_mg[B4_PROC_HUMP_MEDIAN_SAMPLES];
    uint16_t          impact_n;
    uint8_t           impact_raw_next;
    uint8_t           impact_raw_count;
    uint8_t           hump_raw_next;
    uint8_t           hump_raw_count;
    uint8_t           impact_ring_next;
    uint8_t           hump_confirm_count;
    uint8_t           hump_complete_count;
    uint8_t           level_count;
    uint8_t           baseline_step_count;
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
    bool              b_learn;
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

/** @brief Read one axis from a raw calibrated sample.
 *
 * @param p_sample The calibrated sample.
 * @param axis The selected axis number.
 * @return The raw acceleration for the selected axis in milli-g.
 */
static int32_t
b4_proc_sample_axis_mg (const b4_types_sample_t * p_sample, uint8_t axis);

/** @brief Return the median of a small array without changing it.
 *
 * The values are copied and sorted. For an even count the upper of the two
 * middle values is returned.
 *
 * @param p_values The values in milli-g, in any order.
 * @param count The number of values, from 1 to the largest median window.
 * @return The median in milli-g, or 0 when the count is 0.
 */
static int16_t
b4_proc_median_mg (const int16_t * p_values, uint8_t count);

/** @brief Add a raw hump-axis sample and return the median-based amplitude.
 *
 * @param p_sample The newest calibrated sample.
 * @return The absolute difference between the median of the raw hump-axis
 *         samples and the flat-ground baseline in milli-g, or 0 while the
 *         incline offset itself is zero.
 */
static uint32_t
b4_proc_hump_amplitude_mg (const b4_types_sample_t * p_sample);

/** @brief Restart the raw hump-axis history.
 */
static void
b4_proc_hump_ring_clear (void);

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
 * The height uses the moving mean, not the newest sample.
 */
static void
b4_proc_update_height (void);

/** @brief Return the noise estimate in milli-g.
 *
 * @return The rounded median estimate in milli-g, at least one.
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

/** @brief Apply one median step toward an amplitude.
 *
 * The step is 2^-n of the estimate, at least one Q8 unit, and the estimate
 * never goes below its scaled minimum. The step count saturates.
 *
 * @param amplitude_q8 The absolute incline offset in Q8 milli-g.
 */
static void
b4_proc_median_step (int32_t amplitude_q8);

/** @brief Update the median noise estimate for one sample.
 *
 * The estimate moves one step per millimetre of wheel travel toward the
 * amplitude, up when the amplitude
 * is above it and down when below, and only when the drive input says the
 * car is driving, the baseline is valid, the moving mean is full, the
 * hump state is NORMAL or CANDIDATE and there is no impact. The travel
 * comes from the drive speed and the time since the previous sample. The
 * same samples also feed the START sweep mean, whether or not the car
 * drives.
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

/** @brief Find the largest raw axis change since the previous sample.
 *
 * @param p_sample The newest calibrated sample.
 * @return The largest absolute axis change in milli-g.
 */
static uint32_t
b4_proc_sample_delta_mg (const b4_types_sample_t * p_sample);

/** @brief Clear the impact filter states and the window of recent values.
 *
 * The learned impact statistics are kept.
 */
static void
b4_proc_impact_clear_window (void);

/** @brief Filter the sample and find the three-axis impact score.
 *
 * The filters run whenever the previous sample is valid. The window holds 0
 * for a sample that is not judged.
 *
 * @param p_sample The newest calibrated sample.
 * @param b_judged True when the sample is judged and learned from.
 * @return The smallest, over the axes, of the largest windowed absolute
 *         filtered value in milli-g.
 */
static uint32_t
b4_proc_impact_score_mg (const b4_types_sample_t * p_sample, bool b_judged);

/** @brief Learn the score statistics from one unflagged score.
 *
 * @param score_mg The three-axis impact score in milli-g.
 */
static void
b4_proc_impact_learn (uint32_t score_mg);

/** @brief Detect an impact and apply the configured refractory interval.
 *
 * Learns and judges only when the Vehicle Controller says the car is driving
 * and the previous sample is valid. A score above the threshold while the
 * hump state is DESCENDING is still kept out of learning, but it raises no
 * event and does not start the refractory clock, because the crossing's own
 * exit drop would otherwise count.
 *
 * @param p_sample The newest calibrated sample.
 * @return True when this sample starts an impact event.
 */
static bool
b4_proc_detect_impact (const b4_types_sample_t * p_sample);

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
 * The median must also have had its learning hold since START, so the
 * roll of a launch cannot pass for a ramp.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_start_hump (uint32_t amplitude_mg);

/** @brief Confirm or cancel a candidate hump.
 *
 * A sample below the threshold cancels the candidate, so confirmation
 * needs consecutive samples above the threshold.
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
            /* NOTE: Without a START sweep the estimate begins at its
             * lowest allowed value. Every real START seeds it from the
             * sweep, and the learning hold keeps humps off until it has
             * moved. */
            g_b4_proc_state.noise_q8 =
                (int32_t)B4_PROC_MIN_THRESHOLD_MG * B4_PROC_NOISE_SCALE;
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
    uint32_t hump_amplitude_mg = 0U;
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
        g_b4_proc_state.status.drive_speed_mm_s =
            g_b4_proc_state.drive_speed_mm_s;
        g_b4_proc_state.status.fault = B4_FAULT_NONE;
        g_b4_proc_state.status.b_sample_stale = false;
        g_b4_proc_state.status.b_impact_event = false;
        g_b4_proc_state.status.b_hump_started_event = false;
        g_b4_proc_state.status.b_hump_completed_event = false;

        sample_delta_mg = b4_proc_sample_delta_mg(p_sample);
        g_b4_proc_state.status.b_impact_event =
            b4_proc_detect_impact(p_sample);
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
        b4_proc_push_incline(incline_mg);
        b_settled = b4_proc_incline_is_settled();
        b4_proc_update_baseline(incline_mg, b_settled);
        g_b4_proc_state.status.roll_baseline_mg =
            g_b4_proc_state.incline_baseline_mg;
        incline_delta_mg = b4_proc_incline_delta_mg();
        amplitude_mg = b4_proc_abs_mg(incline_delta_mg);
        g_b4_proc_state.status.incline_delta_mg = incline_delta_mg;
        /* Only the hump detector uses the median of the raw samples. The
         * noise, level, baseline and height keep the filtered signal. */
        hump_amplitude_mg = b4_proc_hump_amplitude_mg(p_sample);
        b4_proc_update_noise(amplitude_mg);

        if (false == g_b4_proc_state.status.b_impact_event)
        {
            b4_proc_advance_hump(hump_amplitude_mg, b_settled);
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
        b4_proc_impact_clear_window();
        b4_proc_reset_active_hump();

        /* A reset also discards the attitude reference and the settled
         * window, because a calibration between runs changes the sensor
         * bias and would otherwise leave a height offset in the baseline. */
        g_b4_proc_state.incline_baseline_mg = 0;
        g_b4_proc_state.incline_window_count = 0U;
        g_b4_proc_state.incline_window_next = 0U;
        b4_proc_hump_ring_clear();
        g_b4_proc_state.b_incline_baseline_valid = false;
        g_b4_proc_state.level_count = 0U;
        g_b4_proc_state.b_level_since_gap = false;
        g_b4_proc_state.learn_count = 0U;
        g_b4_proc_state.travel_um = 0U;
        g_b4_proc_state.noise_q8 =
            (int32_t)B4_PROC_MIN_THRESHOLD_MG * B4_PROC_NOISE_SCALE;
        b4_proc_refresh_thresholds();
        g_b4_proc_state.status.noise_mg = 0U;
        g_b4_proc_state.status.incline_delta_mg = 0;
        g_b4_proc_state.status.detectable_height_um = 0U;
        g_b4_proc_state.turn_cross_accum = 0;
        g_b4_proc_state.b_mag_previous_valid = false;
        g_b4_proc_state.status.roll_mg = 0;
        g_b4_proc_state.status.roll_baseline_mg = 0;
        g_b4_proc_state.status.learn_count = 0U;
        g_b4_proc_state.status.drive_speed_mm_s = 0;
        g_b4_proc_state.b_learn = false;
        g_b4_proc_state.drive_speed_mm_s = 0;
        g_b4_proc_state.status.turn_cross = 0;
        g_b4_proc_state.status.turn = B4_TURN_UNKNOWN;
    }

    return (result);
}

/** @brief Clear the run maximum and the latest result for a new run.
 *
 * The flat-ground baseline and level confirmation are kept, so a car that
 * is ready at the start line stays ready. The learning count restarts, so
 * no hump can start until the median has had its learning hold of driving
 * samples. The estimate itself is kept, seeded by the START sweep.
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
        g_b4_proc_state.learn_count = 0U;
        g_b4_proc_state.travel_um = 0U;
        g_b4_proc_state.status.learn_count = 0U;
        /* The impact filters, window and refractory clock restart, but the
         * learned impact statistics are kept. */
        b4_proc_impact_clear_window();
        g_b4_proc_state.b_have_last_impact = false;
        g_b4_proc_state.last_impact_timestamp_ms = 0U;
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
        b4_proc_hump_ring_clear();
        g_b4_proc_state.incline_baseline_mg = 0;
        g_b4_proc_state.b_incline_baseline_valid = false;
        g_b4_proc_state.level_count = 0U;
        g_b4_proc_state.b_level_since_gap = false;
        g_b4_proc_state.learn_count = 0U;
        g_b4_proc_state.travel_um = 0U;
        g_b4_proc_state.cal_sum_mg = 0U;
        g_b4_proc_state.cal_count = 0U;
        g_b4_proc_state.b_cal_measuring = false;
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
                    /* A zero seed is below the lowest allowed estimate
                     * and a flat trace is not a reason to fail: take the
                     * smallest one. */
                    floor_mg = B4_PROC_MIN_THRESHOLD_MG;
                }
                *p_floor_mg = floor_mg;

                if (floor_mg > B4_PROC_MAX_SEED_MG)
                {
                    result = B4_RESULT_INVALID_CALIBRATION;
                }
                else
                {
                    /* The sweep mean seeds the estimate and nothing
                     * else. It is not a floor, and the estimate moves
                     * away from it as the car drives. */
                    g_b4_proc_state.noise_q8 =
                        (int32_t)floor_mg * B4_PROC_NOISE_SCALE;
                    b4_proc_refresh_thresholds();
                    result = B4_RESULT_OK;
                }
            }
        }
    }

    return (result);
}

void
b4_proc_set_drive (bool b_learn, int32_t speed_mm_s)
{
    g_b4_proc_state.b_learn = b_learn;
    g_b4_proc_state.drive_speed_mm_s = speed_mm_s;
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
b4_proc_sample_axis_mg (const b4_types_sample_t * p_sample, uint8_t axis)
{
    int32_t axis_mg = 0;

    switch (axis)
    {
        case B4_CFG_AXIS_X:
            axis_mg = p_sample->accel_x_mg;
            break;
        case B4_CFG_AXIS_Y:
            axis_mg = p_sample->accel_y_mg;
            break;
        case B4_CFG_AXIS_Z:
            axis_mg = p_sample->accel_z_mg;
            break;
        default:
            axis_mg = 0;
            break;
    }

    return (axis_mg);
}

static int16_t
b4_proc_median_mg (const int16_t * p_values, uint8_t count)
{
    /* Every entry that is read has been written by the copy first. */
    int16_t sorted_mg[B4_PROC_HUMP_MEDIAN_SAMPLES];
    int16_t value_mg = 0;
    int16_t median_mg = 0;
    uint8_t used = count;
    uint8_t index = 0U;
    uint8_t slot = 0U;

    if (used > B4_PROC_HUMP_MEDIAN_SAMPLES)
    {
        used = B4_PROC_HUMP_MEDIAN_SAMPLES;
    }

    if (0U != used)
    {
        for (index = 0U; index < used; index++)
        {
            sorted_mg[index] = p_values[index];
        }

        /* Insertion sort: the arrays are small, and it needs no extra
         * memory. */
        for (index = 1U; index < used; index++)
        {
            value_mg = sorted_mg[index];
            slot = index;

            while ((slot > 0U) && (sorted_mg[slot - 1U] > value_mg))
            {
                sorted_mg[slot] = sorted_mg[slot - 1U];
                slot--;
            }

            sorted_mg[slot] = value_mg;
        }

        median_mg = sorted_mg[used / 2U];
    }

    return (median_mg);
}

static void
b4_proc_hump_ring_clear (void)
{
    /* Only the newest hump_raw_count entries are read, so the old data needs
     * no clearing. This follows the incline window, which also restarts on a
     * gap, an invalid sample, a reset and a level calibration. */
    g_b4_proc_state.hump_raw_next = 0U;
    g_b4_proc_state.hump_raw_count = 0U;
}

static uint32_t
b4_proc_hump_amplitude_mg (const b4_types_sample_t * p_sample)
{
    uint8_t mean_samples = g_b4_proc_state.config.height_mean_samples;
    uint32_t amplitude_mg = 0U;
    int32_t  raw_mg = b4_proc_sample_axis_mg(
        p_sample, g_b4_proc_state.config.hump_axis);

    /* The sample is validated to within +-4000 mg, so it fits int16_t. */
    g_b4_proc_state.hump_raw_mg[g_b4_proc_state.hump_raw_next] =
        (int16_t)raw_mg;
    g_b4_proc_state.hump_raw_next++;

    if (g_b4_proc_state.hump_raw_next >= B4_PROC_HUMP_MEDIAN_SAMPLES)
    {
        g_b4_proc_state.hump_raw_next = 0U;
    }

    if (g_b4_proc_state.hump_raw_count < B4_PROC_HUMP_MEDIAN_SAMPLES)
    {
        g_b4_proc_state.hump_raw_count++;
    }

    /* NOTE: Same condition as the incline offset, so the amplitude is zero
     * while no baseline exists or the incline window has not refilled. */
    if ((true == g_b4_proc_state.b_incline_baseline_valid) &&
        (g_b4_proc_state.incline_window_count >= mean_samples))
    {
        amplitude_mg = b4_proc_difference_mg(
            (int32_t)b4_proc_median_mg(g_b4_proc_state.hump_raw_mg,
                                       g_b4_proc_state.hump_raw_count),
            g_b4_proc_state.incline_baseline_mg);
    }

    return (amplitude_mg);
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
    b4_proc_hump_ring_clear();
    g_b4_proc_state.level_count = 0U;
    g_b4_proc_state.b_level_since_gap = false;
    g_b4_proc_state.learn_count = 0U;
    g_b4_proc_state.travel_um = 0U;
    g_b4_proc_state.status.learn_count = 0U;
}

static uint32_t
b4_proc_noise_mg (void)
{
    /* The estimate never goes below the scaled minimum, so it is positive
     * and the unsigned conversion is exact. Adding half a scale step rounds
     * to the nearest milli-g. */
    uint32_t noise_mg = (uint32_t)(g_b4_proc_state.noise_q8 +
                                   B4_PROC_NOISE_HALF_SCALE) /
                        (uint32_t)B4_PROC_NOISE_SCALE;

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
    g_b4_proc_state.status.learn_count = g_b4_proc_state.learn_count;
    /* The start threshold is at most 8000 times 20, or 160000 milli-g, so
     * the conversion cast is exact and the height product stays within
     * int64_t. */
    g_b4_proc_state.status.detectable_height_um =
        b4_proc_height_um((int32_t)start_mg);
}

static void
b4_proc_median_step (int32_t amplitude_q8)
{
    int32_t step_q8 = 0;
    int32_t floor_q8 = 0;

    /* NOTE: A median tracker. Each step moves the estimate by the same
     * fraction, 2^-n, whether the amplitude is just above it or far above
     * it, so a ramp edge counts as one ordinary noisy step and cannot drag
     * the threshold up. The step is at least one Q8 unit, so the estimate
     * still moves when it is small.
     *
     * The amplitude is the difference of two means within the 4000 milli-g
     * sample range, so it is at most 8000 and the scaled value is at most
     * 2.05e6. The estimate stays between its scaled minimum and that value
     * plus one step, so the sums below cannot overflow int32_t. The shift
     * is validated to 4 to 12, so it is a defined shift of a positive
     * value. */
    step_q8 = g_b4_proc_state.noise_q8 >>
              g_b4_proc_state.config.noise_step_shift;

    if (step_q8 < 1)
    {
        step_q8 = 1;
    }

    if (amplitude_q8 > g_b4_proc_state.noise_q8)
    {
        g_b4_proc_state.noise_q8 += step_q8;
    }
    else if (amplitude_q8 < g_b4_proc_state.noise_q8)
    {
        g_b4_proc_state.noise_q8 -= step_q8;
    }
    else
    {
        /* On the estimate: the median does not move. */
    }

    floor_q8 = (int32_t)B4_PROC_MIN_THRESHOLD_MG * B4_PROC_NOISE_SCALE;

    if (g_b4_proc_state.noise_q8 < floor_q8)
    {
        g_b4_proc_state.noise_q8 = floor_q8;
    }

    if (g_b4_proc_state.learn_count < B4_PROC_MAX_LEARN_COUNT)
    {
        g_b4_proc_state.learn_count++;
    }
}

static void
b4_proc_update_noise (uint32_t amplitude_mg)
{
    uint8_t mean_samples = g_b4_proc_state.config.height_mean_samples;

    /* NOTE: Learning continues in the CANDIDATE state. A candidate that is
     * cancelled was noise, and it lasts at most hump_confirm_samples.
     * Excluding it pulled the median low, which lowered the threshold and
     * made more false candidates, a feedback loop. Learning stops once a
     * hump is confirmed (ASCENDING and later), so a real ramp cannot drag
     * the threshold up. The sweep mean uses the same condition. */
    if ((true == g_b4_proc_state.b_incline_baseline_valid) &&
        (g_b4_proc_state.incline_window_count >= mean_samples) &&
        ((B4_HUMP_NORMAL == g_b4_proc_state.status.hump_state) ||
         (B4_HUMP_CANDIDATE == g_b4_proc_state.status.hump_state)) &&
        (false == g_b4_proc_state.status.b_impact_event))
    {
        int32_t  amplitude_q8 = 0;
        int32_t  speed_mm_s = g_b4_proc_state.drive_speed_mm_s;
        uint32_t elapsed_ms = 0U;
        uint32_t steps = 0U;

        /* The calibration mean takes the same eligible samples as the
         * median does, but whether or not the car is driving, because the
         * sweep pivots. It is not clipped: it must seed the estimate from
         * the sweep, not from the old estimate. */
        if ((true == g_b4_proc_state.b_cal_measuring) &&
            (g_b4_proc_state.cal_count < B4_PROC_CAL_MAX_SAMPLES))
        {
            g_b4_proc_state.cal_sum_mg += amplitude_mg;
            g_b4_proc_state.cal_count++;
        }

        if ((true == g_b4_proc_state.b_learn) &&
            (true == g_b4_proc_state.b_previous_sample_valid))
        {
            elapsed_ms = g_b4_proc_state.status.sample.timestamp_ms -
                         g_b4_proc_state.previous_timestamp_ms;

            /* NOTE: A zero interval adds nothing. An interval longer than
             * the stale limit, 100 ms, adds nothing either, because the
             * stream stalled and the speed no longer describes the gap.
             * The speed is limited to the encodable range, so a negative
             * or oversized value cannot add false travel. */
            if (speed_mm_s < 0)
            {
                speed_mm_s = 0;
            }

            if (speed_mm_s > B4_PROC_MAX_SPEED_MM_S)
            {
                speed_mm_s = B4_PROC_MAX_SPEED_MM_S;
            }

            if ((0U != elapsed_ms) &&
                (elapsed_ms <= g_b4_proc_state.config.stale_after_ms))
            {
                /* Millimetres per second times milliseconds is
                 * micrometres. The product is at most 1023 times the
                 * 100 ms stale limit, 102300, and the accumulator is below
                 * 1000 before it is added, so it stays far below 2^32. */
                g_b4_proc_state.travel_um +=
                    (uint32_t)speed_mm_s * elapsed_ms;
                steps = g_b4_proc_state.travel_um / B4_PROC_STEP_TRAVEL_UM;
                g_b4_proc_state.travel_um -=
                    steps * B4_PROC_STEP_TRAVEL_UM;
            }

            /* At most one sample of travel at the top speed, 102 steps at
             * 100 ms, so the loop is short. */
            amplitude_q8 = (int32_t)amplitude_mg * B4_PROC_NOISE_SCALE;

            for (uint32_t step = 0U; step < steps; step++)
            {
                b4_proc_median_step(amplitude_q8);
            }
        }
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

static void
b4_proc_impact_clear_window (void)
{
    uint32_t axis = 0U;
    uint32_t slot = 0U;

    for (axis = 0U; axis < B4_PROC_AXIS_COUNT; axis++)
    {
        g_b4_proc_state.impact_hp_x256[axis] = 0;

        for (slot = 0U; slot < B4_PROC_IMPACT_WINDOW_SAMPLES; slot++)
        {
            g_b4_proc_state.impact_ring_mg[axis][slot] = 0U;
        }
    }

    g_b4_proc_state.impact_ring_next = 0U;

    /* The next median starts a new history and is its own previous, so the
     * restart cannot look like a step. */
    g_b4_proc_state.impact_raw_next = 0U;
    g_b4_proc_state.impact_raw_count = 0U;

    for (axis = 0U; axis < B4_PROC_AXIS_COUNT; axis++)
    {
        g_b4_proc_state.impact_median_mg[axis] = 0;
    }
}

static uint32_t
b4_proc_impact_score_mg (const b4_types_sample_t * p_sample, bool b_judged)
{
    int32_t  accel_mg[B4_PROC_AXIS_COUNT] = {0, 0, 0};
    int32_t  median_mg = 0;
    uint32_t score_mg = 0xFFFFU;
    uint32_t window_max_mg = 0U;
    uint32_t value_mg = 0U;
    uint32_t axis = 0U;
    uint32_t slot = 0U;
    uint8_t  filled = 0U;
    bool     b_restart = false;

    accel_mg[B4_PROC_AXIS_X] = p_sample->accel_x_mg;
    accel_mg[B4_PROC_AXIS_Y] = p_sample->accel_y_mg;
    accel_mg[B4_PROC_AXIS_Z] = p_sample->accel_z_mg;

    /* A break in the stream starts the raw history again. Only the newest
     * impact_raw_count entries are read, so the old data needs no clearing. */
    if (false == g_b4_proc_state.b_previous_sample_valid)
    {
        g_b4_proc_state.impact_raw_next = 0U;
        g_b4_proc_state.impact_raw_count = 0U;
    }

    b_restart = (0U == g_b4_proc_state.impact_raw_count);
    filled = g_b4_proc_state.impact_raw_count;

    if (filled < B4_PROC_IMPACT_MEDIAN_SAMPLES)
    {
        filled++;
    }

    for (axis = 0U; axis < B4_PROC_AXIS_COUNT; axis++)
    {
        value_mg = 0U;

        /* The sample is validated to within +-4000 mg, so it fits int16_t. */
        g_b4_proc_state.impact_raw_mg[axis][g_b4_proc_state.impact_raw_next]
            = (int16_t)accel_mg[axis];
        median_mg = (int32_t)b4_proc_median_mg(
            g_b4_proc_state.impact_raw_mg[axis], filled);

        if (true == b_restart)
        {
            /* The median of validated samples is within +-4000 mg too, so it
             * fits int16_t. */
            g_b4_proc_state.impact_median_mg[axis] = (int16_t)median_mg;
        }

        if (true == g_b4_proc_state.b_previous_sample_valid)
        {
            /* Samples are validated to within +-4000 mg, and so is their
             * median, so the change of the median is within +-8000 mg, or
             * 2.05e6 scaled by 256. The filter state then stays within
             * 199 / 57 of that, about 7.2e6. The sum before the product is
             * at most 9.2e6, and 199 times it is 1.83e9, inside int32_t. */
            g_b4_proc_state.impact_hp_x256[axis] =
                (b4_proc_impact_hp_coeff *
                 (g_b4_proc_state.impact_hp_x256[axis] +
                  (b4_proc_impact_hp_scale *
                   (median_mg -
                    (int32_t)g_b4_proc_state.impact_median_mg[axis])))) /
                b4_proc_impact_hp_scale;

            if (true == b_judged)
            {
                value_mg = b4_proc_abs_mg(
                    g_b4_proc_state.impact_hp_x256[axis]) /
                    (uint32_t)b4_proc_impact_hp_scale;
            }
        }
        else
        {
            g_b4_proc_state.impact_hp_x256[axis] = 0;
        }

        /* Within +-4000 mg as above, so the narrowing is exact. */
        g_b4_proc_state.impact_median_mg[axis] = (int16_t)median_mg;

        if (value_mg > 0xFFFFU)
        {
            value_mg = 0xFFFFU;
        }

        /* The value is at most 65535 here, so it fits uint16_t. */
        g_b4_proc_state.impact_ring_mg[axis][g_b4_proc_state.impact_ring_next]
            = (uint16_t)value_mg;

        window_max_mg = 0U;

        for (slot = 0U; slot < B4_PROC_IMPACT_WINDOW_SAMPLES; slot++)
        {
            if (g_b4_proc_state.impact_ring_mg[axis][slot] > window_max_mg)
            {
                window_max_mg = g_b4_proc_state.impact_ring_mg[axis][slot];
            }
        }

        if (window_max_mg < score_mg)
        {
            score_mg = window_max_mg;
        }
    }

    g_b4_proc_state.impact_ring_next++;

    if (g_b4_proc_state.impact_ring_next >= B4_PROC_IMPACT_WINDOW_SAMPLES)
    {
        g_b4_proc_state.impact_ring_next = 0U;
    }

    g_b4_proc_state.impact_raw_count = filled;
    g_b4_proc_state.impact_raw_next++;

    if (g_b4_proc_state.impact_raw_next >= B4_PROC_IMPACT_MEDIAN_SAMPLES)
    {
        g_b4_proc_state.impact_raw_next = 0U;
    }

    return (score_mg);
}

static void
b4_proc_impact_learn (uint32_t score_mg)
{
    b4_proc_impact_t * p_impact = &g_b4_proc_state.impact;
    uint32_t t_mg = p_impact->t_x50 / b4_proc_impact_units_per_mg;

    if (score_mg > t_mg)
    {
        p_impact->t_x50 += b4_proc_impact_up_step_x50;
        p_impact->k++;
        p_impact->excess_sum += score_mg - t_mg;
    }
    else if (p_impact->t_x50 > 0U)
    {
        p_impact->t_x50 -= b4_proc_impact_down_step_x50;
    }
    else
    {
        /* The estimate is already at zero. */
    }

    if (p_impact->k > 0U)
    {
        /* The 64-bit product cannot overflow, and the quotient is the mean
         * excess times 5.48, which is no larger than excess_sum times 5.48
         * and so fits uint32_t for any realistic excess_sum. */
        p_impact->threshold_mg =
            (p_impact->t_x50 / b4_proc_impact_units_per_mg) +
            (uint32_t)(((uint64_t)p_impact->excess_sum *
                        b4_proc_impact_tail_mult_x100) /
                       ((uint64_t)b4_proc_impact_tail_div *
                        (uint64_t)p_impact->k));
    }
}

static bool
b4_proc_detect_impact (const b4_types_sample_t * p_sample)
{
    bool b_detected = false;
    bool b_flagged = false;
    bool b_refractory_elapsed = false;
    uint32_t score_mg = 0U;
    bool b_judged = false;

    b_judged = ((true == g_b4_proc_state.b_learn) &&
                (true == g_b4_proc_state.b_previous_sample_valid));
    score_mg = b4_proc_impact_score_mg(p_sample, b_judged);

    if (true == b_judged)
    {
        if ((g_b4_proc_state.impact_n >= b4_proc_impact_warmup_samples) &&
            (score_mg > g_b4_proc_state.impact.threshold_mg))
        {
            /* Not learned from. */
            b_flagged = true;
        }
        else
        {
            b4_proc_impact_learn(score_mg);
        }

        if (g_b4_proc_state.impact_n < b4_proc_impact_warmup_samples)
        {
            g_b4_proc_state.impact_n++;
        }
    }

    if (false == g_b4_proc_state.b_have_last_impact)
    {
        b_refractory_elapsed = true;
    }
    else if ((p_sample->timestamp_ms -
              g_b4_proc_state.last_impact_timestamp_ms) >=
             g_b4_proc_state.config.impact_refractory_ms)
    {
        b_refractory_elapsed = true;
    }
    else
    {
        b_refractory_elapsed = false;
    }

    if ((true == b_flagged) && (true == b_refractory_elapsed) &&
        (B4_HUMP_DESCENDING != g_b4_proc_state.status.hump_state))
    {
        b_detected = true;
        g_b4_proc_state.last_impact_timestamp_ms = p_sample->timestamp_ms;
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
 * The median must also have had its learning hold since START, so the
 * roll of a launch cannot pass for a ramp.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_start_hump (uint32_t amplitude_mg)
{
    /* NOTE: After a data gap the car may already be partway over a ramp,
     * so a hump may start only from confirmed level ground. Otherwise the
     * rest of the crossing would publish a partial height as valid. */
    /* The learning hold is one time constant of the median, 2^n learning
     * samples. The shift is validated to at most 12, so the shift of one
     * stays within uint32_t. */
    if ((true == g_b4_proc_state.b_level_since_gap) &&
        (g_b4_proc_state.learn_count >=
         ((uint32_t)1U << g_b4_proc_state.config.noise_step_shift)) &&
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
 * A sample below the threshold cancels the candidate, so confirmation
 * needs consecutive samples above the threshold.
 *
 * @param amplitude_mg The absolute incline offset in milli-g.
 */
static void
b4_proc_update_hump_candidate (uint32_t amplitude_mg)
{
    if (amplitude_mg >= g_b4_proc_state.start_threshold_mg)
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
