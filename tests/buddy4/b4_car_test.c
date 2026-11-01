/** @file b4_car_test.c
 *
 * @brief Buddy 4 ROBO-PICO car diagnostic harness.
 *
 * Orchestration only. Hardware access stays inside b4_imu, sample processing
 * stays inside b4_proc, and this module adds no driver, filter, calibration
 * method, or terrain algorithm of its own. It creates no RTOS objects and
 * runs in the kernel initial task, so it never returns.
 *
 * Console records use stable prefixes so a captured log can be parsed.
 *
 *   B4I  configuration, calibration, and stationary baseline
 *   B4R  periodic run summary counters
 *   B4S  periodic sample and status
 *   B4E  processing events, printed when they happen
 *   B4H  health and fault changes, printed when they happen
 *
 * Deviation from BARR-C 1.6.a, module wide: tm_printf() views read-only text
 * through unsigned char pointers. A format string cast only adds that same
 * view of the literal and cannot alter any value.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "b4_car_test.h"
#include "b4_car_test_log.h"
#include "b4_cfg.h"
#include "b4_imu.h"
#include "b4_proc.h"
#include "b4_types.h"
#include "usb_console_compat.h"

/** Console enumeration wait before the first record is printed. */
#define B4_CAR_TEST_CONSOLE_TIMEOUT_MS     20000U
/** Console enumeration poll interval. */
#define B4_CAR_TEST_CONSOLE_POLL_MS          100U
/** Settling time before the stationary baseline is measured. */
#define B4_CAR_TEST_BASELINE_SETTLE_MS      2000U
/** Number of samples in the stationary baseline. */
#define B4_CAR_TEST_BASELINE_SAMPLES         100U
/** Number of samples in each calibration window. */
#define B4_CAR_TEST_CAL_SAMPLES              100U
/** Countdown before the calibration pre window. */
#define B4_CAR_TEST_CAL_PRE_WAIT_MS        10000U
/** Countdown step used by the calibration prompt. */
#define B4_CAR_TEST_CAL_COUNTDOWN_STEP_MS   1000U
/** Settling time before the calibration post window. */
#define B4_CAR_TEST_CAL_POST_WAIT_MS        2000U
/** Sample period that targets the 100 Hz acquisition rate. */
#define B4_CAR_TEST_SAMPLE_PERIOD_MS          10U
/** Samples between B4S records, which is about 10 Hz. */
#define B4_CAR_TEST_REPORT_SAMPLES            10U
/** Samples between run summaries, which is about 30 seconds. */
#define B4_CAR_TEST_SUMMARY_SAMPLES         3000U
/** Interval between repeated startup failure records. */
#define B4_CAR_TEST_FAILURE_REPORT_MS       2000U

/** Build profile token, supplied by the dedicated test makefile. */
#ifndef B4_TEST_PROFILE
#define B4_TEST_PROFILE unknown_b4_car_test
#endif

#ifndef B4_TEST_CALIBRATION
#define B4_TEST_CALIBRATION 0
#endif

/** ROBO-PICO Grove port that the diagnostic mapping uses. */
#define B4_CAR_TEST_GROVE_PORT              "2"

#define B4_TEST_STRINGIFY_INNER(value) (#value)
#define B4_TEST_STRINGIFY(value) B4_TEST_STRINGIFY_INNER(value)

/** @brief Descriptive stationary statistics for one car test measurement. */
typedef struct
{
    int32_t sum_x_mg;
    int32_t sum_y_mg;
    int32_t sum_z_mg;
    int32_t min_x_mg;
    int32_t min_y_mg;
    int32_t min_z_mg;
    int32_t max_x_mg;
    int32_t max_y_mg;
    int32_t max_z_mg;
    int32_t count;
    bool    b_first;
} b4_car_test_stats_t;

/** @brief A three axis milli-g vector used by the calibration records. */
typedef struct
{
    int32_t x_mg;
    int32_t y_mg;
    int32_t z_mg;
} b4_car_test_vector_t;

/** @brief One stationary window that the calibration run collects. */
typedef struct
{
    uint16_t stored_count;
    uint16_t failed_count;
    b4_car_test_vector_t mean_mg;
    uint32_t max_deviation_mg;
} b4_car_test_window_t;

/** @brief Last reported health and fault pair for both sources. */
typedef struct
{
    b4_types_health_t proc_health;
    b4_types_fault_t  proc_fault;
    b4_types_health_t imu_health;
    b4_types_fault_t  imu_fault;
} b4_car_test_health_t;

/** @brief Car Test run counters reported by the periodic summary. */
typedef struct
{
    uint32_t total;
    uint32_t read_ok;
    uint32_t read_fail;
    uint32_t consecutive_fail;
    uint32_t max_consecutive_fail;
    uint32_t impact;
    uint32_t hump_start;
    uint32_t hump_complete;
    uint32_t stale;
    uint32_t highest_peak_mg;
    uint32_t since_report;
    uint32_t since_summary;
} b4_car_test_counter_t;

static b4_car_test_health_t g_car_test_health = {0};
static b4_car_test_counter_t g_car_test_counter = {0};
static b4_types_sample_t g_car_test_previous = {0};
static bool g_car_test_b_have_previous = false;

/* The car test firmware runs in the kernel initial task, whose stack is 1 KiB
 * and is shared with the kernel call path. The four aggregates below would
 * otherwise sit in the deepest frame, so they are file scope. Nothing else
 * writes them, because the car test firmware creates no other task. */
static b4_cfg_config_t g_car_test_config = {0};
static b4_types_status_t g_car_test_status = {0};
static b4_types_sample_t g_car_test_sample = {0};
static b4_car_test_stats_t g_car_test_stats = {0};

/* A calibration window has to keep every sample for
 * b4_cfg_calibrate_stationary, so that buffer is file scope for the same
 * 1 KiB stack reason as the aggregates above. */
static b4_types_sample_t g_car_test_cal_samples[B4_CAR_TEST_CAL_SAMPLES] = {0};
static b4_car_test_window_t g_car_test_window = {0};

/** @brief Wait, with a bound, until the USB console is enumerated. */
static void
car_test_console_wait (void);

/** @brief Return monotonic milliseconds since boot. */
static uint32_t
car_test_now_ms (void);

/** @brief Print the self-describing startup banner. */
static void
car_test_print_banner (const b4_cfg_config_t * p_config);

/** @brief Run the configuration and initialization sequence.
 *
 * @param p_config The configuration to validate, initialize, and fill in.
 * @param pp_operation The destination for the name of the failed step.
 * @return The result of the last step that ran.
 */
static b4_types_result_t
car_test_startup (b4_cfg_config_t * p_config, const char ** pp_operation);

/** @brief Report a startup failure forever so wiring can be diagnosed.
 *
 * @param p_operation The name of the step that failed.
 * @param result The result that step returned.
 */
static void
car_test_failure_loop (const char * p_operation, b4_types_result_t result);

/** @brief Measure and report the stationary sensor baseline. */
static void
car_test_run_baseline (void);

/** @brief Derive the expected stationary gravity vector from orientation.
 *
 * @param p_config The validated sensor and orientation configuration.
 * @param p_expected_mg The destination for the expected sensor-axis vector.
 * @return The result of deriving the expected vector.
 */
static b4_types_result_t
car_test_expected_gravity (const b4_cfg_config_t * p_config,
                           b4_car_test_vector_t * p_expected_mg);

/** @brief Run the calibration lifecycle and report what it measured. */
static void
car_test_run_calibration (void);

/** @brief Print a seconds countdown before a calibration window.
 *
 * @param wait_ms The total wait time.
 */
static void
car_test_countdown (uint32_t wait_ms);

/** @brief Collect one stationary window and summarise it.
 *
 * @param requested_samples The number of samples to attempt.
 */
static void
car_test_collect_window (uint16_t requested_samples);

/** @brief Fill the window means and the peak deviation from the buffer. */
static void
car_test_summarise_window (void);

/** @brief Return the largest axis deviation of one sample from a mean.
 *
 * @param p_sample The sample to measure.
 * @param p_mean_mg The window mean.
 * @return The largest absolute axis deviation in milli-g.
 */
static uint32_t
car_test_peak_deviation_mg (const b4_types_sample_t * p_sample,
                         const b4_car_test_vector_t * p_mean_mg);

/** @brief Print the calibration acceptance record.
 *
 * @param close_result The result of b4_imu_close.
 * @param calibrate_result The result of b4_cfg_calibrate_stationary.
 * @param init_result The result of the reinitialization.
 * @param pre_stored_count The number of samples the calibration used.
 * @param p_expected_mg The expected stationary vector.
 */
static void
car_test_report_calibration (b4_types_result_t close_result,
                          b4_types_result_t calibrate_result,
                          b4_types_result_t init_result,
                          b4_types_result_t reset_result,
                          uint16_t pre_stored_count,
                          const b4_car_test_vector_t * p_expected_mg);

/** @brief Add one sample to the stationary statistics.
 *
 * @param p_stats The statistics being accumulated.
 * @param p_sample The sample to add.
 */
static void
car_test_stats_add (b4_car_test_stats_t * p_stats,
                 const b4_types_sample_t * p_sample);

/** @brief Print the stationary baseline record.
 *
 * @param p_stats The accumulated statistics.
 * @param read_ok The number of successful reads.
 * @param read_fail The number of failed reads.
 */
static void
car_test_stats_report (const b4_car_test_stats_t * p_stats,
                    uint32_t read_ok,
                    uint32_t read_fail);

/** @brief Store and print the health state that the stream starts from.
 *
 * @param p_status The current processing status.
 */
static void
car_test_capture_health (const b4_types_status_t * p_status);

/** @brief Run the continuous sample, process, and report loop. */
static void
car_test_run_stream (void);

/** @brief Print B4H records for changed health or fault states.
 *
 * @param p_status The current processing status.
 * @param now_ms The current monotonic time in milliseconds.
 */
static void
car_test_report_transitions (const b4_types_status_t * p_status,
                          uint32_t now_ms);

/** @brief Print B4E records for events of the latest processed sample.
 *
 * @param p_status The current processing status.
 * @param b_processed True when the latest read produced a new sample.
 * @param delta_x_mg The car test diagnostic X change since the last sample.
 * @param delta_y_mg The car test diagnostic Y change since the last sample.
 * @param delta_z_mg The car test diagnostic Z change since the last sample.
 */
static void
car_test_report_events (const b4_types_status_t * p_status,
                     bool b_processed,
                     int32_t delta_x_mg,
                     int32_t delta_y_mg,
                     int32_t delta_z_mg);

/** @brief Print one periodic B4S sample and status record. */
static void
car_test_report_sample (const b4_types_status_t * p_status);

/** @brief Print the periodic run counters. */
static void
car_test_report_summary (void);

/** @brief Return the larger of two signed milli-g values. */
static int32_t
car_test_max_mg (int32_t left_mg, int32_t right_mg);

/** @brief Return the smaller of two signed milli-g values. */
static int32_t
car_test_min_mg (int32_t left_mg, int32_t right_mg);

/** @brief Return the larger of two unsigned counters. */
static uint32_t
car_test_max_count (uint32_t left_count, uint32_t right_count);

/** @brief Return the unsigned difference between two milli-g values. */
static int32_t
car_test_difference_mg (int32_t left_mg, int32_t right_mg);

/** @brief Estimate vector magnitude using integer arithmetic.
 *
 * @param x_mg The X acceleration in milli-g.
 * @param y_mg The Y acceleration in milli-g.
 * @param z_mg The Z acceleration in milli-g.
 * @return The estimated vector magnitude in milli-g.
 */
static uint32_t
car_test_gravity_magnitude_mg (int32_t x_mg,
                               int32_t y_mg,
                               int32_t z_mg);

/** @brief Run the Buddy 4 car test harness and return only on a fatal path.
 *
 * The sequence is configuration, initialization, a stationary baseline, the
 * calibration lifecycle, then the continuous stream. A startup failure stays
 * in the failure loop so the board never pretends that samples are valid.
 */
void
b4_car_test_run (void)
{
    b4_types_result_t result = B4_RESULT_OK;
    const char * p_operation = "b4_cfg_default";

    g_car_test_config = b4_cfg_default();

    car_test_console_wait();
    tm_printf((UB *)"B4I bringup step=b4_cfg_default result=OK\n");
    car_test_print_banner(&g_car_test_config);

    result = car_test_startup(&g_car_test_config, &p_operation);
    if (B4_RESULT_OK != result)
    {
        car_test_failure_loop(p_operation, result);
    }

    car_test_run_baseline();
#if B4_TEST_CALIBRATION
    car_test_run_calibration();
#else
    tm_printf((UB *)"B4I calibration mode=disabled; orientation review "
              "first\n");
#endif

    /* The stream copies the state it starts from, so later records only ever
     * report a real change. */
    (void)b4_proc_get_status(car_test_now_ms(), &g_car_test_status);
    car_test_capture_health(&g_car_test_status);

    car_test_run_stream();
}

/** @brief Wait, with a bound, until the USB console is enumerated. */
static void
car_test_console_wait (void)
{
    uint32_t waited_ms = 0U;

    while ((2U > tm_usb_state()) &&
           (B4_CAR_TEST_CONSOLE_TIMEOUT_MS > waited_ms))
    {
        tk_dly_tsk(B4_CAR_TEST_CONSOLE_POLL_MS);
        waited_ms += B4_CAR_TEST_CONSOLE_POLL_MS;
    }
}

/** @brief Return monotonic milliseconds since boot. */
static uint32_t
car_test_now_ms (void)
{
    SYSTIM system_time = {0};

    (void)tk_get_otm(&system_time);

    /* The lower word of the system time is the millisecond counter, and the
     * source and destination widths are both 32 bits, so the cast is
     * exact. */
    return ((uint32_t)system_time.lo);
}

/** @brief Print the self-describing startup banner.
 *
 * @param p_config The configuration that the harness is about to use.
 */
static void
car_test_print_banner (const b4_cfg_config_t * p_config)
{
    tm_printf((UB *)"\nBuddy 4 ROBO-PICO Car Test\n");
    tm_printf((UB *)"B4I firmware_profile=%s\n",
              B4_TEST_STRINGIFY(B4_TEST_PROFILE));
    tm_printf((UB *)"B4I mapping grove_port=" B4_CAR_TEST_GROVE_PORT
              " controller=I2C1"
              " i2c_unit=%u sda_gpio=%u scl_gpio=%u frequency_hz=%u\n",
              p_config->i2c_unit,
              p_config->sda_gpio,
              p_config->scl_gpio,
              p_config->i2c_frequency_hz);
    tm_printf((UB *)"B4I accel address=0x%02x data_rate_hz=%u range_g=%u\n",
              p_config->accel_i2c_address,
              p_config->accel_data_rate_hz,
              p_config->accel_range_g);
    tm_printf((UB *)"B4I bias_mg x=%d y=%d z=%d"
              " gravity_reference_mg=%u\n",
              p_config->bias_x_mg,
              p_config->bias_y_mg,
              p_config->bias_z_mg,
              p_config->gravity_reference_mg);
    tm_printf((UB *)"B4I orientation forward=%s%s lateral=%s%s"
              " vertical=%s%s\n",
              b4_car_test_log_axis_name(p_config->forward_axis),
              b4_car_test_log_sign_name(p_config->forward_sign),
              b4_car_test_log_axis_name(p_config->lateral_axis),
              b4_car_test_log_sign_name(p_config->lateral_sign),
              b4_car_test_log_axis_name(p_config->vertical_axis),
              b4_car_test_log_sign_name(p_config->vertical_sign));
    tm_printf((UB *)"B4I thresholds_mg motion=%u tilt=%u impact=%u"
              " hump_start_k_tenths=%u\n",
              p_config->motion_threshold_mg,
              p_config->tilt_threshold_mg,
              p_config->impact_delta_threshold_mg,
              p_config->hump_start_k_tenths);
    tm_printf((UB *)"B4I calibration noise_limit_mg=%u"
              " max_consecutive_read_failures=%u\n",
              p_config->calibration_noise_limit_mg,
              p_config->max_consecutive_read_failures);
}

/** @brief Run the configuration and initialization sequence.
 *
 * @param p_config The configuration to validate, initialize, and fill in.
 * @param pp_operation The destination for the name of the failed step.
 * @return The result of the last step that ran.
 */
static b4_types_result_t
car_test_startup (b4_cfg_config_t * p_config, const char ** pp_operation)
{
    b4_types_result_t result = B4_RESULT_OK;

    *pp_operation = "b4_cfg_validate";
    result = b4_cfg_validate(p_config);
    tm_printf((UB *)"B4I bringup step=b4_cfg_validate result=%s\n",
              b4_car_test_log_result_name(result));

    if (B4_RESULT_OK == result)
    {
        *pp_operation = "b4_proc_init";
        result = b4_proc_init(p_config);
        tm_printf((UB *)"B4I bringup step=b4_proc_init result=%s\n",
                  b4_car_test_log_result_name(result));
    }

    if (B4_RESULT_OK == result)
    {
        *pp_operation = "b4_imu_init";
        result = b4_imu_init(p_config);
        tm_printf((UB *)"B4I bringup step=b4_imu_init result=%s\n",
                  b4_car_test_log_result_name(result));
    }

    return (result);
}

/** @brief Report a startup failure forever so wiring can be diagnosed.
 *
 * @param p_operation The name of the step that failed.
 * @param result The result that step returned.
 */
static void
car_test_failure_loop (const char * p_operation, b4_types_result_t result)
{
    for (;;)
    {
        b4_types_health_t health = B4_HEALTH_NOT_INITIALIZED;
        b4_types_fault_t fault = B4_FAULT_NONE;
        uint8_t failure_count = 0U;

        (void)b4_imu_get_health(&health, &fault, &failure_count);

        tm_printf((UB *)"B4H status=startup_failure operation=%s"
                  " result=%s health=%s fault=%s failures=%u\n",
                  p_operation,
                  b4_car_test_log_result_name(result),
                  b4_car_test_log_health_name(health),
                  b4_car_test_log_fault_name(fault),
                  failure_count);

        tk_dly_tsk(B4_CAR_TEST_FAILURE_REPORT_MS);
    }
}

/** @brief Measure and report the stationary sensor baseline. */
static void
car_test_run_baseline (void)
{
    uint32_t read_ok = 0U;
    uint32_t read_fail = 0U;
    bool b_first_sample_reported = false;

    g_car_test_stats.b_first = true;

    tm_printf((UB *)"B4I baseline hold the sensor motionless, collecting"
              " %u samples\n",
              B4_CAR_TEST_BASELINE_SAMPLES);

    tk_dly_tsk(B4_CAR_TEST_BASELINE_SETTLE_MS);

    for (uint32_t index = 0U; index < B4_CAR_TEST_BASELINE_SAMPLES; index++)
    {
        if (B4_RESULT_OK == b4_imu_read(&g_car_test_sample))
        {
            car_test_stats_add(&g_car_test_stats, &g_car_test_sample);
            (void)b4_proc_process(&g_car_test_sample);
            read_ok++;

            if (false == b_first_sample_reported)
            {
                tm_printf((UB *)"B4I bringup first_xyz result=OK x=%d y=%d"
                          " z=%d\n",
                          g_car_test_sample.accel_x_mg,
                          g_car_test_sample.accel_y_mg,
                          g_car_test_sample.accel_z_mg);
                b_first_sample_reported = true;
            }
        }
        else
        {
            read_fail++;
        }

        tk_dly_tsk(B4_CAR_TEST_SAMPLE_PERIOD_MS);
    }

    if (false == b_first_sample_reported)
    {
        tm_printf((UB *)"B4I bringup first_xyz result=SENSOR_COMM_FAILURE\n");
    }

    car_test_stats_report(&g_car_test_stats, read_ok, read_fail);
}

/** @brief Derive the expected stationary gravity vector from orientation.
 *
 * The configuration was validated during startup. Its vertical axis is one
 * of the three sensor axes, its sign is +1 or -1, and gravity is bounded by
 * the supported sample range.
 *
 * @param p_config The validated sensor and orientation configuration.
 * @param p_expected_mg The destination for the expected sensor-axis vector.
 * @return The result of deriving the expected vector.
 */
static b4_types_result_t
car_test_expected_gravity (const b4_cfg_config_t * p_config,
                           b4_car_test_vector_t * p_expected_mg)
{
    b4_types_result_t result = B4_RESULT_OK;

    if ((NULL == p_config) || (NULL == p_expected_mg))
    {
        result = B4_RESULT_INVALID_ARGUMENT;
    }
    else
    {
        /* Validation bounds the unsigned reference to the signed sample
         * range, so the conversion preserves its exact value. */
        int32_t gravity_mg = (int32_t)p_config->gravity_reference_mg;

        p_expected_mg->x_mg = 0;
        p_expected_mg->y_mg = 0;
        p_expected_mg->z_mg = 0;

        /* Validation limits the sign to -1 or +1, so this conversion is
         * exact and the product remains within the validated sample range. */
        gravity_mg *= (int32_t)p_config->vertical_sign;

        switch (p_config->vertical_axis)
        {
            case B4_CFG_AXIS_X:
                p_expected_mg->x_mg = gravity_mg;
            break;
            case B4_CFG_AXIS_Y:
                p_expected_mg->y_mg = gravity_mg;
            break;
            case B4_CFG_AXIS_Z:
                p_expected_mg->z_mg = gravity_mg;
            break;
            default:
                result = B4_RESULT_HARDWARE_CONFIG_FAILURE;
            break;
        }
    }

    return (result);
}

/** @brief Run the calibration lifecycle and report what it measured.
 *
 * The sequence is the one the plan fixes: collect stationary uncorrected
 * samples, close the sensor, let b4_cfg_calibrate_stationary compute the
 * bias, initialize again with the updated configuration, reset processing
 * history, then collect a second stationary window. The harness never
 * computes a bias itself.
 */
static void
car_test_run_calibration (void)
{
    b4_types_result_t close_result = B4_RESULT_OK;
    b4_types_result_t calibrate_result = B4_RESULT_OK;
    b4_types_result_t init_result = B4_RESULT_OK;
    b4_types_result_t reset_result = B4_RESULT_NOT_INITIALIZED;
    b4_types_result_t expected_result = B4_RESULT_OK;
    b4_car_test_vector_t expected_mg = {0};
    b4_car_test_window_t pre_window = {0};

    expected_result = car_test_expected_gravity(
        &g_car_test_config,
        &expected_mg);
    if (B4_RESULT_OK != expected_result)
    {
        car_test_failure_loop("calibration_orientation", expected_result);
    }

    tm_printf((UB *)"B4I calibration start samples=%u noise_limit_mg=%u\n",
              B4_CAR_TEST_CAL_SAMPLES,
              g_car_test_config.calibration_noise_limit_mg);
    tm_printf((UB *)"B4I calibration hold the sensor flat and level,"
              " pre window starts in %u s\n",
              (B4_CAR_TEST_CAL_PRE_WAIT_MS /
               B4_CAR_TEST_CAL_COUNTDOWN_STEP_MS));

    car_test_countdown(B4_CAR_TEST_CAL_PRE_WAIT_MS);
    car_test_collect_window(B4_CAR_TEST_CAL_SAMPLES);
    pre_window = g_car_test_window;

    tm_printf((UB *)"B4I calibration pre_window read_ok=%u read_fail=%u"
              " mean_mg x=%d y=%d z=%d peak_deviation_mg=%u\n",
              pre_window.stored_count,
              pre_window.failed_count,
              pre_window.mean_mg.x_mg,
              pre_window.mean_mg.y_mg,
              pre_window.mean_mg.z_mg,
              pre_window.max_deviation_mg);

    close_result = b4_imu_close();
    tm_printf((UB *)"B4I calibration close result=%s\n",
              b4_car_test_log_result_name(close_result));

    calibrate_result = b4_cfg_calibrate_stationary(
        g_car_test_cal_samples,
        pre_window.stored_count,
        expected_mg.x_mg,
        expected_mg.y_mg,
        expected_mg.z_mg,
        &g_car_test_config);
    tm_printf((UB *)"B4I calibration stationary result=%s\n",
              b4_car_test_log_result_name(calibrate_result));
    tm_printf((UB *)"B4I calibration bias_mg x=%d y=%d z=%d\n",
              g_car_test_config.bias_x_mg,
              g_car_test_config.bias_y_mg,
              g_car_test_config.bias_z_mg);

    init_result = b4_imu_init(&g_car_test_config);
    tm_printf((UB *)"B4I calibration reinit result=%s\n",
              b4_car_test_log_result_name(init_result));
    if (B4_RESULT_OK != init_result)
    {
        car_test_failure_loop("b4_imu_init", init_result);
    }

    reset_result = b4_proc_reset_run_stats();
    tm_printf((UB *)"B4I calibration proc_reset result=%s\n",
              b4_car_test_log_result_name(reset_result));
    if (B4_RESULT_OK != reset_result)
    {
        car_test_failure_loop("b4_proc_reset_run_stats", reset_result);
    }

    tk_dly_tsk(B4_CAR_TEST_CAL_POST_WAIT_MS);
    car_test_collect_window(B4_CAR_TEST_CAL_SAMPLES);

    tm_printf((UB *)"B4I calibration post_window read_ok=%u read_fail=%u"
              " mean_mg x=%d y=%d z=%d peak_deviation_mg=%u\n",
              g_car_test_window.stored_count,
              g_car_test_window.failed_count,
              g_car_test_window.mean_mg.x_mg,
              g_car_test_window.mean_mg.y_mg,
              g_car_test_window.mean_mg.z_mg,
              g_car_test_window.max_deviation_mg);

    car_test_report_calibration(close_result,
                             calibrate_result,
                             init_result,
                             reset_result,
                             pre_window.stored_count,
                             &expected_mg);
}

/** @brief Print a seconds countdown before a calibration window.
 *
 * @param wait_ms The total wait time.
 */
static void
car_test_countdown (uint32_t wait_ms)
{
    uint32_t remaining_s = wait_ms / B4_CAR_TEST_CAL_COUNTDOWN_STEP_MS;

    while (0U < remaining_s)
    {
        tm_printf((UB *)"B4I calibration countdown_s=%u\n", remaining_s);
        tk_dly_tsk(B4_CAR_TEST_CAL_COUNTDOWN_STEP_MS);
        remaining_s--;
    }
}

/** @brief Collect one stationary window and summarise it.
 *
 * Every successful read is stored for b4_cfg_calibrate_stationary and fed to
 * b4_proc. Failed reads are counted but not stored, so the calibration only
 * ever sees valid samples.
 *
 * @param requested_samples The number of samples to attempt.
 */
static void
car_test_collect_window (uint16_t requested_samples)
{
    if (B4_CAR_TEST_CAL_SAMPLES < requested_samples)
    {
        requested_samples = B4_CAR_TEST_CAL_SAMPLES;
    }

    g_car_test_window.stored_count = 0U;
    g_car_test_window.failed_count = 0U;

    for (uint16_t index = 0U; index < requested_samples; index++)
    {
        if (B4_RESULT_OK == b4_imu_read(&g_car_test_sample))
        {
            g_car_test_cal_samples[g_car_test_window.stored_count] =
                g_car_test_sample;
            g_car_test_window.stored_count++;
            (void)b4_proc_process(&g_car_test_sample);
        }
        else
        {
            g_car_test_window.failed_count++;
        }

        tk_dly_tsk(B4_CAR_TEST_SAMPLE_PERIOD_MS);
    }

    car_test_summarise_window();
}

/** @brief Fill the window means and the peak deviation from the buffer. */
static void
car_test_summarise_window (void)
{
    int32_t sum_x_mg = 0;
    int32_t sum_y_mg = 0;
    int32_t sum_z_mg = 0;

    g_car_test_window.mean_mg.x_mg = 0;
    g_car_test_window.mean_mg.y_mg = 0;
    g_car_test_window.mean_mg.z_mg = 0;
    g_car_test_window.max_deviation_mg = 0U;

    for (uint16_t index = 0U; index < g_car_test_window.stored_count; index++)
    {
        sum_x_mg += g_car_test_cal_samples[index].accel_x_mg;
        sum_y_mg += g_car_test_cal_samples[index].accel_y_mg;
        sum_z_mg += g_car_test_cal_samples[index].accel_z_mg;
    }

    if (0U < g_car_test_window.stored_count)
    {
        g_car_test_window.mean_mg.x_mg =
            sum_x_mg / (int32_t)g_car_test_window.stored_count;
        g_car_test_window.mean_mg.y_mg =
            sum_y_mg / (int32_t)g_car_test_window.stored_count;
        g_car_test_window.mean_mg.z_mg =
            sum_z_mg / (int32_t)g_car_test_window.stored_count;

        for (uint16_t index = 0U; index < g_car_test_window.stored_count;
             index++)
        {
            uint32_t deviation_mg = car_test_peak_deviation_mg(
                &g_car_test_cal_samples[index], &g_car_test_window.mean_mg);

            if (deviation_mg > g_car_test_window.max_deviation_mg)
            {
                g_car_test_window.max_deviation_mg = deviation_mg;
            }
        }
    }
}

/** @brief Return the largest axis deviation of one sample from a mean.
 *
 * @param p_sample The sample to measure.
 * @param p_mean_mg The window mean.
 * @return The largest absolute axis deviation in milli-g.
 */
static uint32_t
car_test_peak_deviation_mg (const b4_types_sample_t * p_sample,
                         const b4_car_test_vector_t * p_mean_mg)
{
    /* car_test_difference_mg only ever returns a magnitude, so each conversion
     * to unsigned is exact. */
    uint32_t deviation_mg = (uint32_t)car_test_difference_mg(
        p_sample->accel_x_mg, p_mean_mg->x_mg);
    uint32_t axis_deviation_mg = (uint32_t)car_test_difference_mg(
        p_sample->accel_y_mg, p_mean_mg->y_mg);

    if (axis_deviation_mg > deviation_mg)
    {
        deviation_mg = axis_deviation_mg;
    }

    axis_deviation_mg = (uint32_t)car_test_difference_mg(
        p_sample->accel_z_mg, p_mean_mg->z_mg);
    if (axis_deviation_mg > deviation_mg)
    {
        deviation_mg = axis_deviation_mg;
    }

    return (deviation_mg);
}

/** @brief Print the calibration acceptance record.
 *
 * A corrected stationary mean lands on the expected vector, so each axis must
 * be within the calibration noise limit of it. That check fails when the bias
 * was never applied, which is the behaviour under test.
 *
 * @param close_result The result of b4_imu_close.
 * @param calibrate_result The result of b4_cfg_calibrate_stationary.
 * @param init_result The result of the reinitialization.
 * @param reset_result The result of resetting processing history.
 * @param pre_stored_count The number of samples the calibration used.
 * @param p_expected_mg The expected stationary vector.
 */
static void
car_test_report_calibration (b4_types_result_t close_result,
                          b4_types_result_t calibrate_result,
                          b4_types_result_t init_result,
                          b4_types_result_t reset_result,
                          uint16_t pre_stored_count,
                          const b4_car_test_vector_t * p_expected_mg)
{
    int32_t noise_limit_mg =
        (int32_t)g_car_test_config.calibration_noise_limit_mg;
    bool b_applied = false;
    bool b_noise_ok = false;
    bool b_pass = false;
    const UB * p_verdict = (UB *)"FAIL";

    tm_printf((UB *)"B4I calibration expected_mg x=%d y=%d z=%d\n",
              p_expected_mg->x_mg,
              p_expected_mg->y_mg,
              p_expected_mg->z_mg);

    b_applied =
        ((car_test_difference_mg(g_car_test_window.mean_mg.x_mg,
                              p_expected_mg->x_mg) <= noise_limit_mg) &&
         (car_test_difference_mg(g_car_test_window.mean_mg.y_mg,
                              p_expected_mg->y_mg) <= noise_limit_mg) &&
         (car_test_difference_mg(g_car_test_window.mean_mg.z_mg,
                              p_expected_mg->z_mg) <= noise_limit_mg));
    b_noise_ok = (g_car_test_window.max_deviation_mg <=
                  g_car_test_config.calibration_noise_limit_mg);
    b_pass = ((B4_RESULT_OK == close_result) &&
              (B4_RESULT_OK == calibrate_result) &&
              (B4_RESULT_OK == init_result) &&
              (B4_RESULT_OK == reset_result) &&
              (B4_CAR_TEST_CAL_SAMPLES == pre_stored_count) &&
              (B4_CAR_TEST_CAL_SAMPLES == g_car_test_window.stored_count) &&
              (true == b_applied) &&
              (true == b_noise_ok));

    if (true == b_pass)
    {
        p_verdict = (UB *)"PASS";
    }

    tm_printf((UB *)"B4I calibration verify applied=%u noise=%u"
              " verdict=%s\n",
              b_applied,
              b_noise_ok,
              p_verdict);
}

/** @brief Add one sample to the stationary statistics.
 *
 * @param p_stats The statistics being accumulated.
 * @param p_sample The sample to add.
 */
static void
car_test_stats_add (b4_car_test_stats_t * p_stats,
                 const b4_types_sample_t * p_sample)
{
    int32_t x_mg = p_sample->accel_x_mg;
    int32_t y_mg = p_sample->accel_y_mg;
    int32_t z_mg = p_sample->accel_z_mg;

    if (true == p_stats->b_first)
    {
        p_stats->min_x_mg = x_mg;
        p_stats->min_y_mg = y_mg;
        p_stats->min_z_mg = z_mg;
        p_stats->max_x_mg = x_mg;
        p_stats->max_y_mg = y_mg;
        p_stats->max_z_mg = z_mg;
        p_stats->b_first = false;
    }
    else
    {
        p_stats->min_x_mg = car_test_min_mg(p_stats->min_x_mg, x_mg);
        p_stats->min_y_mg = car_test_min_mg(p_stats->min_y_mg, y_mg);
        p_stats->min_z_mg = car_test_min_mg(p_stats->min_z_mg, z_mg);
        p_stats->max_x_mg = car_test_max_mg(p_stats->max_x_mg, x_mg);
        p_stats->max_y_mg = car_test_max_mg(p_stats->max_y_mg, y_mg);
        p_stats->max_z_mg = car_test_max_mg(p_stats->max_z_mg, z_mg);
    }

    p_stats->sum_x_mg += x_mg;
    p_stats->sum_y_mg += y_mg;
    p_stats->sum_z_mg += z_mg;
    p_stats->count++;
}

/** @brief Print the stationary baseline record.
 *
 * @param p_stats The accumulated statistics.
 * @param read_ok The number of successful reads.
 * @param read_fail The number of failed reads.
 */
static void
car_test_stats_report (const b4_car_test_stats_t * p_stats,
                    uint32_t read_ok,
                    uint32_t read_fail)
{
    b4_types_health_t health = B4_HEALTH_NOT_INITIALIZED;
    b4_types_fault_t fault = B4_FAULT_NONE;
    uint8_t failure_count = 0U;

    (void)b4_imu_get_health(&health, &fault, &failure_count);

    tm_printf((UB *)"B4I baseline samples=%u read_ok=%u read_fail=%u\n",
              (read_ok + read_fail),
              read_ok,
              read_fail);

    if (0 < p_stats->count)
    {
        int32_t mean_x_mg = p_stats->sum_x_mg / p_stats->count;
        int32_t mean_y_mg = p_stats->sum_y_mg / p_stats->count;
        int32_t mean_z_mg = p_stats->sum_z_mg / p_stats->count;

        tm_printf((UB *)"B4I baseline x_mg mean=%d min=%d max=%d range=%d\n",
                  mean_x_mg,
                  p_stats->min_x_mg,
                  p_stats->max_x_mg,
                  (p_stats->max_x_mg - p_stats->min_x_mg));
        tm_printf((UB *)"B4I baseline y_mg mean=%d min=%d max=%d range=%d\n",
                  mean_y_mg,
                  p_stats->min_y_mg,
                  p_stats->max_y_mg,
                  (p_stats->max_y_mg - p_stats->min_y_mg));
        tm_printf((UB *)"B4I baseline z_mg mean=%d min=%d max=%d range=%d\n",
                  mean_z_mg,
                  p_stats->min_z_mg,
                  p_stats->max_z_mg,
                  (p_stats->max_z_mg - p_stats->min_z_mg));
        tm_printf((UB *)"B4I baseline gravity_magnitude_mg=%u\n",
                  car_test_gravity_magnitude_mg(mean_x_mg,
                                                mean_y_mg,
                                                mean_z_mg));
    }

    tm_printf((UB *)"B4I baseline final imu_health=%s imu_fault=%s"
              " failures=%u\n",
              b4_car_test_log_health_name(health),
              b4_car_test_log_fault_name(fault),
              failure_count);
}

/** @brief Store and print the health state that the stream starts from.
 *
 * @param p_status The current processing status.
 */
static void
car_test_capture_health (const b4_types_status_t * p_status)
{
    b4_types_health_t imu_health = B4_HEALTH_NOT_INITIALIZED;
    b4_types_fault_t imu_fault = B4_FAULT_NONE;
    uint8_t failure_count = 0U;

    (void)b4_imu_get_health(&imu_health, &imu_fault, &failure_count);

    g_car_test_health.proc_health = p_status->health;
    g_car_test_health.proc_fault = p_status->fault;
    g_car_test_health.imu_health = imu_health;
    g_car_test_health.imu_fault = imu_fault;

    tm_printf((UB *)"B4I stream starting health=%s fault=%s imu_health=%s"
              " imu_fault=%s failures=%u\n",
              b4_car_test_log_health_name(g_car_test_health.proc_health),
              b4_car_test_log_fault_name(g_car_test_health.proc_fault),
              b4_car_test_log_health_name(g_car_test_health.imu_health),
              b4_car_test_log_fault_name(g_car_test_health.imu_fault),
              failure_count);
}

/** @brief Run the continuous sample, process, and report loop. */
static void
car_test_run_stream (void)
{
    for (;;)
    {
        int32_t delta_x_mg = 0;
        int32_t delta_y_mg = 0;
        int32_t delta_z_mg = 0;
        uint32_t now_ms = 0U;
        bool b_processed = false;
        b4_types_result_t read_result = b4_imu_read(&g_car_test_sample);

        g_car_test_counter.total++;

        if (B4_RESULT_OK == read_result)
        {
            if (true == g_car_test_b_have_previous)
            {
                delta_x_mg = car_test_difference_mg(
                    g_car_test_sample.accel_x_mg,
                    g_car_test_previous.accel_x_mg);
                delta_y_mg = car_test_difference_mg(
                    g_car_test_sample.accel_y_mg,
                    g_car_test_previous.accel_y_mg);
                delta_z_mg = car_test_difference_mg(
                    g_car_test_sample.accel_z_mg,
                    g_car_test_previous.accel_z_mg);
            }

            (void)b4_proc_process(&g_car_test_sample);

            g_car_test_previous = g_car_test_sample;
            g_car_test_b_have_previous = true;
            g_car_test_counter.read_ok++;
            g_car_test_counter.consecutive_fail = 0U;
            b_processed = true;
        }
        else
        {
            g_car_test_counter.read_fail++;
            g_car_test_counter.consecutive_fail++;
            g_car_test_counter.max_consecutive_fail =
                car_test_max_count(g_car_test_counter.max_consecutive_fail,
                                g_car_test_counter.consecutive_fail);
        }

        /* Read the clock after the sample so that the sample age can never
         * appear to be negative. */
        now_ms = car_test_now_ms();
        (void)b4_proc_get_status(now_ms, &g_car_test_status);

        if (true == g_car_test_status.b_sample_stale)
        {
            g_car_test_counter.stale++;
        }

        if (g_car_test_status.highest_hump_peak_mg >
            g_car_test_counter.highest_peak_mg)
        {
            g_car_test_counter.highest_peak_mg =
                g_car_test_status.highest_hump_peak_mg;
        }

        car_test_report_transitions(&g_car_test_status, now_ms);
        car_test_report_events(&g_car_test_status, b_processed, delta_x_mg,
                            delta_y_mg, delta_z_mg);

        g_car_test_counter.since_report++;
        g_car_test_counter.since_summary++;

        if (B4_CAR_TEST_REPORT_SAMPLES <= g_car_test_counter.since_report)
        {
            g_car_test_counter.since_report = 0U;
            car_test_report_sample(&g_car_test_status);
        }

        if (B4_CAR_TEST_SUMMARY_SAMPLES <= g_car_test_counter.since_summary)
        {
            g_car_test_counter.since_summary = 0U;
            car_test_report_summary();
        }

        tk_dly_tsk(B4_CAR_TEST_SAMPLE_PERIOD_MS);
    }
}

/** @brief Print B4H records for changed health or fault states.
 *
 * @param p_status The current processing status.
 * @param now_ms The current monotonic time in milliseconds.
 */
static void
car_test_report_transitions (const b4_types_status_t * p_status,
                          uint32_t now_ms)
{
    b4_types_health_t imu_health = B4_HEALTH_NOT_INITIALIZED;
    b4_types_fault_t imu_fault = B4_FAULT_NONE;
    uint8_t failure_count = 0U;

    (void)b4_imu_get_health(&imu_health, &imu_fault, &failure_count);

    if ((p_status->health != g_car_test_health.proc_health) ||
        (p_status->fault != g_car_test_health.proc_fault))
    {
        tm_printf((UB *)"B4H source=proc t=%u health=%s fault=%s\n",
                  now_ms,
                  b4_car_test_log_health_name(p_status->health),
                  b4_car_test_log_fault_name(p_status->fault));

        g_car_test_health.proc_health = p_status->health;
        g_car_test_health.proc_fault = p_status->fault;
    }

    if ((imu_health != g_car_test_health.imu_health) ||
        (imu_fault != g_car_test_health.imu_fault))
    {
        tm_printf((UB *)"B4H source=imu t=%u health=%s fault=%s"
                  " failures=%u\n",
                  now_ms,
                  b4_car_test_log_health_name(imu_health),
                  b4_car_test_log_fault_name(imu_fault),
                  failure_count);

        g_car_test_health.imu_health = imu_health;
        g_car_test_health.imu_fault = imu_fault;
    }
}

/** @brief Print B4E records for events of the latest processed sample.
 *
 * @param p_status The current processing status.
 * @param b_processed True when the latest read produced a new sample.
 * @param delta_x_mg The car test diagnostic X change since the last sample.
 * @param delta_y_mg The car test diagnostic Y change since the last sample.
 * @param delta_z_mg The car test diagnostic Z change since the last sample.
 */
static void
car_test_report_events (const b4_types_status_t * p_status,
                     bool b_processed,
                     int32_t delta_x_mg,
                     int32_t delta_y_mg,
                     int32_t delta_z_mg)
{
    /* Only a new sample can raise an event, and the event flags are cleared
     * by the next successful processing call. Reporting them while reads are
     * failing would repeat the last sample's events. */
    if (true == b_processed)
    {
        if (true == p_status->b_impact_event)
        {
            g_car_test_counter.impact++;
            tm_printf((UB *)"B4E t=%u event=impact motion=%s x=%d y=%d"
                      " z=%d dx=%d dy=%d dz=%d\n",
                      p_status->sample.timestamp_ms,
                      b4_car_test_log_motion_name(p_status->motion),
                      p_status->sample.accel_x_mg,
                      p_status->sample.accel_y_mg,
                      p_status->sample.accel_z_mg,
                      delta_x_mg,
                      delta_y_mg,
                      delta_z_mg);
        }

        if (true == p_status->b_hump_started_event)
        {
            g_car_test_counter.hump_start++;
            tm_printf((UB *)"B4E t=%u event=hump_start hump=%s"
                      " amplitude_mg=%u\n",
                      p_status->sample.timestamp_ms,
                      b4_car_test_log_hump_name(p_status->hump_state),
                      p_status->current_hump_peak_mg);
        }

        if (true == p_status->b_hump_completed_event)
        {
            g_car_test_counter.hump_complete++;
            tm_printf((UB *)"B4E t=%u event=hump_complete hump=%s"
                      " amplitude_mg=%u highest_mg=%u\n",
                      p_status->sample.timestamp_ms,
                      b4_car_test_log_hump_name(p_status->hump_state),
                      p_status->current_hump_peak_mg,
                      p_status->highest_hump_peak_mg);
        }
    }
}

/** @brief Print one periodic B4S sample and status record.
 *
 * @param p_status The current processing status.
 */
static void
car_test_report_sample (const b4_types_status_t * p_status)
{
    b4_types_health_t imu_health = B4_HEALTH_NOT_INITIALIZED;
    b4_types_fault_t imu_fault = B4_FAULT_NONE;
    uint8_t failure_count = 0U;

    (void)b4_imu_get_health(&imu_health, &imu_fault, &failure_count);

    tm_printf((UB *)"B4S timestamp_ms=%u x=%d y=%d z=%d"
              " filtered_x=%d filtered_y=%d filtered_z=%d"
              " vertical_deviation=%d tilt=%s motion=%s hump_state=%s"
              " current_hump_peak=%u highest_hump_peak=%u"
              " health=%s fault=%s imu_health=%s imu_fault=%s"
              " consecutive_failures=%u stale=%u\n",
              p_status->sample.timestamp_ms,
              p_status->sample.accel_x_mg,
              p_status->sample.accel_y_mg,
              p_status->sample.accel_z_mg,
              p_status->filtered_x_mg,
              p_status->filtered_y_mg,
              p_status->filtered_z_mg,
              p_status->vertical_deviation_mg,
              b4_car_test_log_tilt_name(p_status->tilt),
              b4_car_test_log_motion_name(p_status->motion),
              b4_car_test_log_hump_name(p_status->hump_state),
              p_status->current_hump_peak_mg,
              p_status->highest_hump_peak_mg,
              b4_car_test_log_health_name(p_status->health),
              b4_car_test_log_fault_name(p_status->fault),
              b4_car_test_log_health_name(imu_health),
              b4_car_test_log_fault_name(imu_fault),
              failure_count,
              p_status->b_sample_stale);
}

/** @brief Print the periodic run counters. */
static void
car_test_report_summary (void)
{
    tm_printf((UB *)"B4R total_reads=%u successful_reads=%u"
              " failed_reads=%u consecutive_failures=%u"
              " max_consecutive_failures=%u stale_samples=%u\n",
              g_car_test_counter.total,
              g_car_test_counter.read_ok,
              g_car_test_counter.read_fail,
              g_car_test_counter.consecutive_fail,
              g_car_test_counter.max_consecutive_fail,
              g_car_test_counter.stale);
    tm_printf((UB *)"B4R impact_events=%u hump_start_events=%u"
              " hump_complete_events=%u highest_hump_peak_mg=%u"
              " console_dropped_bytes=%u\n",
              g_car_test_counter.impact,
              g_car_test_counter.hump_start,
              g_car_test_counter.hump_complete,
              g_car_test_counter.highest_peak_mg,
              tm_usb_dropped_bytes());
}

/** @brief Return the larger of two signed milli-g values.
 *
 * @param left_mg The first value.
 * @param right_mg The second value.
 * @return The larger value.
 */
static int32_t
car_test_max_mg (int32_t left_mg, int32_t right_mg)
{
    int32_t result_mg = left_mg;

    if (right_mg > left_mg)
    {
        result_mg = right_mg;
    }

    return (result_mg);
}

/** @brief Return the smaller of two signed milli-g values.
 *
 * @param left_mg The first value.
 * @param right_mg The second value.
 * @return The smaller value.
 */
static int32_t
car_test_min_mg (int32_t left_mg, int32_t right_mg)
{
    int32_t result_mg = left_mg;

    if (right_mg < left_mg)
    {
        result_mg = right_mg;
    }

    return (result_mg);
}

/** @brief Return the larger of two unsigned counters.
 *
 * @param left_count The first counter.
 * @param right_count The second counter.
 * @return The larger counter.
 */
static uint32_t
car_test_max_count (uint32_t left_count, uint32_t right_count)
{
    uint32_t result_count = left_count;

    if (right_count > left_count)
    {
        result_count = right_count;
    }

    return (result_count);
}

/** @brief Return the unsigned difference between two milli-g values.
 *
 * @param left_mg The first value.
 * @param right_mg The second value.
 * @return The magnitude of the difference.
 */
static int32_t
car_test_difference_mg (int32_t left_mg, int32_t right_mg)
{
    int32_t difference_mg = left_mg - right_mg;

    if (0 > difference_mg)
    {
        difference_mg = -difference_mg;
    }

    return (difference_mg);
}

/** @brief Estimate vector magnitude using an integer square root.
 *
 * The accelerometer range is limited to plus or minus two g by the validated
 * car-test configuration, so the intermediate sum remains within uint32_t.
 *
 * @param x_mg The X acceleration in milli-g.
 * @param y_mg The Y acceleration in milli-g.
 * @param z_mg The Z acceleration in milli-g.
 * @return The estimated vector magnitude in milli-g.
 */
static uint32_t
car_test_gravity_magnitude_mg (int32_t x_mg,
                               int32_t y_mg,
                               int32_t z_mg)
{
    int32_t abs_x_mg = car_test_difference_mg(x_mg, 0);
    int32_t abs_y_mg = car_test_difference_mg(y_mg, 0);
    int32_t abs_z_mg = car_test_difference_mg(z_mg, 0);
    uint32_t square_sum = (uint32_t)((abs_x_mg * abs_x_mg) +
                                     (abs_y_mg * abs_y_mg) +
                                     (abs_z_mg * abs_z_mg));
    uint32_t root_mg = square_sum;

    if (0U < square_sum)
    {
        uint32_t next_root_mg = (root_mg + 1U) / 2U;

        while (next_root_mg < root_mg)
        {
            root_mg = next_root_mg;
            next_root_mg = (root_mg + (square_sum / root_mg)) / 2U;
        }
    }

    return (root_mg);
}

/*** end of file ***/
