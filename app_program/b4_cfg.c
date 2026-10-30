/** @file b4_cfg.c
 *
 * @brief Buddy 4 configuration defaults and stationary calibration.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "b4_cfg.h"

#define B4_CFG_MAX_GPIO                    29U
#define B4_CFG_I2C_UNIT                     1U
/* The reported ROBO-PICO IMU wiring uses Grove port 2, I2C1 SDA=GP2 and
   SCL=GP3. */
#define B4_CFG_I2C_SDA_GPIO                 2U
#define B4_CFG_I2C_SCL_GPIO                 3U
#define B4_CFG_I2C_FREQUENCY_HZ        400000U
#define B4_CFG_ACCEL_ADDRESS_LOW         0x18U
#define B4_CFG_ACCEL_ADDRESS_HIGH        0x19U
#define B4_CFG_ACCEL_DATA_RATE_HZ          100U
#define B4_CFG_ACCEL_RANGE_G                 2U
#define B4_CFG_MAG_ADDRESS                0x1EU
#define B4_CFG_MAX_SAMPLE_MG              2000
#define B4_CFG_MAX_THRESHOLD_MG           6000U
#define B4_CFG_MIN_CAL_SAMPLES                4U
#define B4_CFG_MAX_CAL_SAMPLES              128U
#define B4_CFG_MIN_FILTER_DIVISOR             1U
#define B4_CFG_MAX_FILTER_DIVISOR            16U
#define B4_CFG_MAX_AXIS                       2U
#define B4_CFG_MIN_HUMP_LEVER_UM          50000U
#define B4_CFG_MAX_HUMP_LEVER_UM         300000U
#define B4_CFG_MIN_CASTER_FORWARD_UM      20000U
#define B4_CFG_MAX_CASTER_FORWARD_UM     300000U
#define B4_CFG_MAX_SETTLE_BAND_MG           500U
#define B4_CFG_MIN_SETTLE_SAMPLES             2U
#define B4_CFG_MAX_BASELINE_DIVISOR         100U
#define B4_CFG_MAX_TURN_THRESHOLD     100000000U
#define B4_CFG_MIN_TURN_WINDOW                2U
#define B4_CFG_MAX_TURN_WINDOW              100U
#define B4_CFG_MAX_NOISE_SAMPLES          10000U
/* The warm-up count saturates at 255 in b4_proc.c, so a larger limit
 * could never be reached. */
#define B4_CFG_MAX_WARMUP_SAMPLES           255U
#define B4_CFG_MAX_K_TENTHS                 200U

/** @brief Return an absolute milli-g value without signed overflow.
 *
 * @param value_mg The signed acceleration value in milli-g.
 * @return The absolute acceleration value as unsigned milli-g.
 */
static uint32_t
b4_cfg_abs_mg (int32_t value_mg);

/** @brief Check whether an axis selects one of the three sensor axes.
 *
 * @param axis The configured axis number.
 * @return True when the axis number is valid.
 */
static bool
b4_cfg_axis_is_valid (uint8_t axis);

/** @brief Check whether an axis sign has a supported direction.
 *
 * @param sign The configured sign value.
 * @return True when the sign is positive one or negative one.
 */
static bool
b4_cfg_sign_is_valid (int8_t sign);

/** @brief Check whether all sample axes fit the sensor range.
 *
 * @param p_sample The sample to check.
 * @return True when every sample axis is within plus or minus two g.
 */
static bool
b4_cfg_sample_is_in_range (const b4_types_sample_t * p_sample);

/** @brief Check whether the selected pins match the RP2040 driver mapping.
 *
 * @param p_config The configuration containing an I2C unit and pin pair.
 * @return True when the unit and pins are supported by the existing driver.
 */
static bool
b4_cfg_bus_pins_are_valid (const b4_cfg_config_t * p_config);

/** @brief Calculate stable sample means for stationary calibration.
 *
 * @param p_samples The stationary samples expressed in milli-g.
 * @param sample_count The number of samples in the sequence.
 * @param noise_limit_mg The maximum permitted axis deviation.
 * @param p_mean_x_mg The destination for the mean X acceleration.
 * @param p_mean_y_mg The destination for the mean Y acceleration.
 * @param p_mean_z_mg The destination for the mean Z acceleration.
 * @return The calibration result.
 */
static b4_types_result_t
b4_cfg_stationary_mean (const b4_types_sample_t * p_samples,
                        uint16_t sample_count,
                        uint32_t noise_limit_mg,
                        int32_t * p_mean_x_mg,
                        int32_t * p_mean_y_mg,
                        int32_t * p_mean_z_mg);

/** @brief Return provisional values for the current Pico I2C driver.
 *
 * The IMU uses ROBO-PICO Grove port 2, I2C1 SDA=GP2 and SCL=GP3.
 *
 * @return A configuration using the ROBO-PICO Grove port 2 and
 *         provisional thresholds.
 */
b4_cfg_config_t
b4_cfg_default (void)
{
    b4_cfg_config_t config = {0};

    config.i2c_unit = B4_CFG_I2C_UNIT;
    config.sda_gpio = B4_CFG_I2C_SDA_GPIO;
    config.scl_gpio = B4_CFG_I2C_SCL_GPIO;
    config.accel_i2c_address = B4_CFG_ACCEL_ADDRESS_HIGH;
    config.accel_data_rate_hz = B4_CFG_ACCEL_DATA_RATE_HZ;
    config.accel_range_g = B4_CFG_ACCEL_RANGE_G;
    config.mag_i2c_address = B4_CFG_MAG_ADDRESS;
    config.mag_data_rate_hz = B4_CFG_MAG_RATE_30_HZ;
    config.i2c_frequency_hz = B4_CFG_I2C_FREQUENCY_HZ;
    config.forward_axis = B4_CFG_AXIS_Y;
    config.lateral_axis = B4_CFG_AXIS_X;
    config.vertical_axis = B4_CFG_AXIS_Z;
    /* NOTE: The ramp passes under the right drive wheel while the left
     * wheel and caster stay on the floor, so the axle line tilts sideways
     * by height / track and the lateral axis carries the height. Speed
     * changes do not disturb it, unlike the forward axis. */
    config.hump_axis = B4_CFG_AXIS_X;
    /* NOTE: Bench hand tests on 2026-10-29 read negative for nose up and
     * negative for right side up, so both axes read backwards. This is
     * consistent with the IMU facing backwards on the chassis. */
    config.forward_sign = -1;
    config.lateral_sign = -1;
    config.vertical_sign = 1;
    config.turn_sign = 1;
    config.gravity_reference_mg = 1000U;
    config.stale_after_ms = 100U;
    config.mag_stale_after_ms = 500U;
    config.motion_threshold_mg = 120U;
    config.tilt_threshold_mg = 250U;
    config.impact_delta_threshold_mg = 700U;
    config.impact_refractory_ms = 200U;
    /* NOTE: This is only the starting floor, until the first START
     * calibration. Every START measures the moving noise in its pivot
     * sweep and replaces it (b4_proc_calibrate_apply), so it follows the
     * floor and chassis of the day. The value is the settled estimate on
     * flat floor at 100 mm/s, 11 to 16 mg in
     * evidence/buddy1/20261030_025325_wifi_check.log and
     * 20261030_030046_wifi_check.log, taking the lower. It is never
     * derived from the ramp. The estimate starts at the floor and never
     * goes below it. */
    config.noise_floor_mg = 12U;
    /* The estimate rises over 10 s at 100 Hz, so a ramp edge or a snag
     * cannot pull the threshold up quickly, and falls over 2 s, so a
     * smoother surface is followed within seconds. */
    config.noise_rise_samples = 1000U;
    config.noise_fall_samples = 200U;
    config.noise_warmup_samples = 100U;
    /* WARNING: Provisional. Step 2 of the noise threshold plan sets both
     * factors from flat runs with no ramp, never from the ramp's height. */
    config.hump_start_k_tenths = 40U;
    config.hump_complete_k_tenths = 20U;
    config.calibration_noise_limit_mg = 35U;
    /* Left to right tyre contact distance, measured by the user as 115 mm
     * on 2026-10-29. */
    config.hump_lever_um = 115000U;
    /* NOTE: These are the effective contact geometry, calibrated from the
     * two static ramp lifts on 2026-10-29. After the sign flip the right
     * lift gave a roll of about +95 mg with a pitch of about -49 mg, and the
     * left lift gave about -91 mg with about -40 mg. The tape-measured
     * caster distance is 80 mm and the gap is unexplained. The values are
     * not fitted to snag data. */
    config.caster_forward_um = 120000U;
    config.caster_lateral_um = 4600;
    /* WARNING: Both consistency bands are provisional. They are set from
     * the residual peaks of the coming driving runs. */
    config.tilt_residual_band_mg = 30U;
    config.tilt_magnitude_band_mg = 30U;
    /* WARNING: A 30-sample window on the gentle ramp slope spreads about
     * 14 mg at 50 mm/s, so a wider band let the baseline climb the ramp
     * before the hump was confirmed and halved the height in a host
     * simulation. Keep the band and gate below that slope spread. */
    config.height_settle_band_mg = 10U;
    config.height_baseline_gate_mg = 12U;
    config.turn_cross_threshold = 20000U;
    config.filter_divisor = 4U;
    config.height_settle_samples = 30U;
    /* NOTE: The ramp crest is a point, so the wheel is never still at full
     * height. 24 samples is 240 ms at 100 Hz, which averages out the
     * roughly 100 mm/s vibration at the cost of a small drop in the peak. */
    config.height_mean_samples = 24U;
    /* NOTE: One 1 mg baseline step per 64 settled samples is about 1.6 mg/s
     * at 100 Hz, too slow to follow a climb but enough for floor drift. */
    config.height_baseline_divisor = 64U;
    config.turn_window_samples = 20U;
    /* WARNING: Ten samples is 0.1 s at the planned 100 Hz rate. Retune this
     * count from the measured rate and the flat-ground roll trace. */
    config.hump_confirm_samples = 10U;
    config.hump_complete_samples = 5U;
    config.max_consecutive_read_failures = 3U;

    return (config);
}

/** @brief Validate the configuration before sensor use.
 *
 * @param p_config The configuration to validate.
 * @return The validation result.
 */
b4_types_result_t
b4_cfg_validate (const b4_cfg_config_t * p_config)
{
    b4_types_result_t result = B4_RESULT_OK;
    bool b_axes_valid = false;
    bool b_addresses_valid = false;
    bool b_bus_pins_valid = false;

    if (NULL == p_config)
    {
        result = B4_RESULT_INVALID_ARGUMENT;
    }
    else
    {
        b_axes_valid =
            (b4_cfg_axis_is_valid(p_config->forward_axis)) &&
            (b4_cfg_axis_is_valid(p_config->lateral_axis)) &&
            (b4_cfg_axis_is_valid(p_config->vertical_axis)) &&
            (b4_cfg_axis_is_valid(p_config->hump_axis)) &&
            (p_config->hump_axis != p_config->vertical_axis) &&
            (p_config->forward_axis != p_config->lateral_axis) &&
            (p_config->forward_axis != p_config->vertical_axis) &&
            (p_config->lateral_axis != p_config->vertical_axis) &&
            (b4_cfg_sign_is_valid(p_config->forward_sign)) &&
            (b4_cfg_sign_is_valid(p_config->lateral_sign)) &&
            (b4_cfg_sign_is_valid(p_config->vertical_sign)) &&
            (b4_cfg_sign_is_valid(p_config->turn_sign));

        b_addresses_valid =
            (B4_CFG_ACCEL_ADDRESS_LOW == p_config->accel_i2c_address) ||
            (B4_CFG_ACCEL_ADDRESS_HIGH == p_config->accel_i2c_address);
        b_bus_pins_valid = b4_cfg_bus_pins_are_valid(p_config);

        if ((!b_axes_valid) || (!b_addresses_valid) ||
            (false == b_bus_pins_valid) ||
            (p_config->i2c_unit != B4_CFG_I2C_UNIT) ||
            (p_config->sda_gpio > B4_CFG_MAX_GPIO) ||
            (p_config->scl_gpio > B4_CFG_MAX_GPIO) ||
            (p_config->i2c_frequency_hz != B4_CFG_I2C_FREQUENCY_HZ) ||
            (p_config->accel_data_rate_hz !=
             B4_CFG_ACCEL_DATA_RATE_HZ) ||
            (p_config->accel_range_g != B4_CFG_ACCEL_RANGE_G) ||
            (p_config->mag_i2c_address != B4_CFG_MAG_ADDRESS) ||
            ((p_config->mag_data_rate_hz != B4_CFG_MAG_RATE_30_HZ) &&
             (p_config->mag_data_rate_hz != B4_CFG_MAG_RATE_75_HZ)) ||
            (0U == p_config->gravity_reference_mg) ||
            (p_config->gravity_reference_mg >
             (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
            (b4_cfg_abs_mg(p_config->bias_x_mg) >
             (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
            (b4_cfg_abs_mg(p_config->bias_y_mg) >
             (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
            (b4_cfg_abs_mg(p_config->bias_z_mg) >
             (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
            (0U == p_config->stale_after_ms) ||
            (0U == p_config->mag_stale_after_ms) ||
            (0U == p_config->motion_threshold_mg) ||
            (p_config->motion_threshold_mg >
             B4_CFG_MAX_THRESHOLD_MG) ||
            (0U == p_config->tilt_threshold_mg) ||
            (p_config->tilt_threshold_mg >
             B4_CFG_MAX_THRESHOLD_MG) ||
            (0U == p_config->impact_delta_threshold_mg) ||
            (p_config->impact_delta_threshold_mg >
             B4_CFG_MAX_THRESHOLD_MG) ||
            (0U == p_config->impact_refractory_ms) ||
            (0U == p_config->noise_floor_mg) ||
            (p_config->noise_floor_mg > B4_CFG_MAX_SETTLE_BAND_MG) ||
            (0U == p_config->noise_rise_samples) ||
            (p_config->noise_rise_samples > B4_CFG_MAX_NOISE_SAMPLES) ||
            (0U == p_config->noise_fall_samples) ||
            (p_config->noise_fall_samples > B4_CFG_MAX_NOISE_SAMPLES) ||
            (0U == p_config->noise_warmup_samples) ||
            (p_config->noise_warmup_samples > B4_CFG_MAX_WARMUP_SAMPLES) ||
            (0U == p_config->hump_complete_k_tenths) ||
            (p_config->hump_start_k_tenths <=
             p_config->hump_complete_k_tenths) ||
            (p_config->hump_start_k_tenths > B4_CFG_MAX_K_TENTHS) ||
            (0U == p_config->calibration_noise_limit_mg) ||
            (p_config->calibration_noise_limit_mg >
             (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
            (p_config->hump_lever_um < B4_CFG_MIN_HUMP_LEVER_UM) ||
            (p_config->hump_lever_um > B4_CFG_MAX_HUMP_LEVER_UM) ||
            (p_config->caster_forward_um < B4_CFG_MIN_CASTER_FORWARD_UM) ||
            (p_config->caster_forward_um > B4_CFG_MAX_CASTER_FORWARD_UM) ||
            ((int64_t)p_config->caster_lateral_um >=
             (int64_t)(p_config->hump_lever_um / 2U)) ||
            ((int64_t)p_config->caster_lateral_um <=
             -(int64_t)(p_config->hump_lever_um / 2U)) ||
            (0U == p_config->tilt_residual_band_mg) ||
            (p_config->tilt_residual_band_mg > B4_CFG_MAX_SETTLE_BAND_MG) ||
            (0U == p_config->tilt_magnitude_band_mg) ||
            (p_config->tilt_magnitude_band_mg >
             B4_CFG_MAX_SETTLE_BAND_MG) ||
            (0U == p_config->height_settle_band_mg) ||
            (p_config->height_settle_band_mg > B4_CFG_MAX_SETTLE_BAND_MG) ||
            (p_config->height_baseline_gate_mg <
             p_config->height_settle_band_mg) ||
            (p_config->height_baseline_gate_mg >
             B4_CFG_MAX_SETTLE_BAND_MG) ||
            (0U == p_config->turn_cross_threshold) ||
            (p_config->turn_cross_threshold >
             B4_CFG_MAX_TURN_THRESHOLD) ||
            (p_config->filter_divisor < B4_CFG_MIN_FILTER_DIVISOR) ||
            (p_config->filter_divisor > B4_CFG_MAX_FILTER_DIVISOR) ||
            (p_config->height_settle_samples < B4_CFG_MIN_SETTLE_SAMPLES) ||
            (p_config->height_settle_samples > B4_CFG_MAX_SETTLE_SAMPLES) ||
            (p_config->height_mean_samples < B4_CFG_MIN_SETTLE_SAMPLES) ||
            (p_config->height_mean_samples > B4_CFG_MAX_SETTLE_SAMPLES) ||
            (0U == p_config->height_baseline_divisor) ||
            (p_config->height_baseline_divisor >
             B4_CFG_MAX_BASELINE_DIVISOR) ||
            (p_config->turn_window_samples < B4_CFG_MIN_TURN_WINDOW) ||
            (p_config->turn_window_samples > B4_CFG_MAX_TURN_WINDOW) ||
            (p_config->hump_confirm_samples < 2U) ||
            (0U == p_config->hump_complete_samples) ||
            (0U == p_config->max_consecutive_read_failures))
        {
            result = B4_RESULT_HARDWARE_CONFIG_FAILURE;
        }
    }

    return (result);
}

/** @brief Calibrate per-axis bias values from stable stationary samples.
 *
 * The stored bias is the stationary residual in sensor coordinates, mean
 * minus expected, and the adapter applies it as measured minus bias. The
 * samples must be uncorrected readings because the adapter subtracts the
 * stored bias from every sample it returns.
 *
 * @param p_samples The stationary samples expressed in milli-g.
 * @param sample_count The number of samples in the sequence.
 * @param expected_x_mg The expected stationary X acceleration.
 * @param expected_y_mg The expected stationary Y acceleration.
 * @param expected_z_mg The expected stationary Z acceleration.
 * @param p_config The configuration whose bias values will be updated.
 * @return The calibration result.
 */
b4_types_result_t
b4_cfg_calibrate_stationary (const b4_types_sample_t * p_samples,
                             uint16_t sample_count,
                             int32_t expected_x_mg,
                             int32_t expected_y_mg,
                             int32_t expected_z_mg,
                             b4_cfg_config_t * p_config)
{
    b4_types_result_t result = B4_RESULT_OK;
    int32_t mean_x_mg = 0;
    int32_t mean_y_mg = 0;
    int32_t mean_z_mg = 0;
    int32_t bias_x_mg = 0;
    int32_t bias_y_mg = 0;
    int32_t bias_z_mg = 0;

    if ((NULL == p_samples) || (NULL == p_config))
    {
        result = B4_RESULT_INVALID_ARGUMENT;
    }
    else if ((sample_count < B4_CFG_MIN_CAL_SAMPLES) ||
             (sample_count > B4_CFG_MAX_CAL_SAMPLES) ||
             (B4_RESULT_OK != b4_cfg_validate(p_config)) ||
             (b4_cfg_abs_mg(expected_x_mg) >
              (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
             (b4_cfg_abs_mg(expected_y_mg) >
              (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
             (b4_cfg_abs_mg(expected_z_mg) >
              (uint32_t)B4_CFG_MAX_SAMPLE_MG))
    {
        result = B4_RESULT_INVALID_CALIBRATION;
    }
    else
    {
        result = b4_cfg_stationary_mean(
            p_samples,
            sample_count,
            p_config->calibration_noise_limit_mg,
            &mean_x_mg,
            &mean_y_mg,
            &mean_z_mg);

        /* A positive residual means the sensor reads high, so the stored
         * bias is the measured mean minus the expected value. */
        bias_x_mg = mean_x_mg - expected_x_mg;
        bias_y_mg = mean_y_mg - expected_y_mg;
        bias_z_mg = mean_z_mg - expected_z_mg;

        if ((B4_RESULT_OK == result) &&
            ((b4_cfg_abs_mg(bias_x_mg) >
              (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
             (b4_cfg_abs_mg(bias_y_mg) >
              (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
             (b4_cfg_abs_mg(bias_z_mg) >
              (uint32_t)B4_CFG_MAX_SAMPLE_MG)))
        {
            result = B4_RESULT_INVALID_CALIBRATION;
        }
        else if (B4_RESULT_OK == result)
        {
            p_config->bias_x_mg = bias_x_mg;
            p_config->bias_y_mg = bias_y_mg;
            p_config->bias_z_mg = bias_z_mg;
        }
    }

    return (result);
}

/** @brief Return an absolute milli-g value without signed overflow.
 *
 * @param value_mg The signed acceleration value in milli-g.
 * @return The absolute acceleration value as unsigned milli-g.
 */
static uint32_t
b4_cfg_abs_mg (int32_t value_mg)
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

/** @brief Check whether an axis selects one of the three sensor axes.
 *
 * @param axis The configured axis number.
 * @return True when the axis number is valid.
 */
static bool
b4_cfg_axis_is_valid (uint8_t axis)
{
    bool b_valid = false;

    if (axis <= B4_CFG_MAX_AXIS)
    {
        b_valid = true;
    }

    return (b_valid);
}

/** @brief Check whether an axis sign has a supported direction.
 *
 * @param sign The configured sign value.
 * @return True when the sign is positive one or negative one.
 */
static bool
b4_cfg_sign_is_valid (int8_t sign)
{
    bool b_valid = false;

    if ((1 == sign) || (-1 == sign))
    {
        b_valid = true;
    }

    return (b_valid);
}

/** @brief Check whether all sample axes fit the sensor range.
 *
 * @param p_sample The sample to check.
 * @return True when every sample axis is within plus or minus two g.
 */
static bool
b4_cfg_sample_is_in_range (const b4_types_sample_t * p_sample)
{
    bool b_in_range = false;

    if ((p_sample->accel_x_mg <= B4_CFG_MAX_SAMPLE_MG) &&
        (p_sample->accel_x_mg >= -B4_CFG_MAX_SAMPLE_MG) &&
        (p_sample->accel_y_mg <= B4_CFG_MAX_SAMPLE_MG) &&
        (p_sample->accel_y_mg >= -B4_CFG_MAX_SAMPLE_MG) &&
        (p_sample->accel_z_mg <= B4_CFG_MAX_SAMPLE_MG) &&
        (p_sample->accel_z_mg >= -B4_CFG_MAX_SAMPLE_MG))
    {
        b_in_range = true;
    }

    return (b_in_range);
}

/** @brief Check whether the selected pins match the ROBO-PICO mapping.
 *
 * @param p_config The configuration containing an I2C unit and pin pair.
 * @return True only for I2C1 on the selected Grove port pins.
 */
static bool
b4_cfg_bus_pins_are_valid (const b4_cfg_config_t * p_config)
{
    bool b_valid = false;

    if ((B4_CFG_I2C_UNIT == p_config->i2c_unit) &&
        (B4_CFG_I2C_SDA_GPIO == p_config->sda_gpio) &&
        (B4_CFG_I2C_SCL_GPIO == p_config->scl_gpio))
    {
        b_valid = true;
    }

    return (b_valid);
}

/** @brief Calculate stable sample means for stationary calibration.
 *
 * @param p_samples The stationary samples expressed in milli-g.
 * @param sample_count The number of samples in the sequence.
 * @param noise_limit_mg The maximum permitted axis deviation.
 * @param p_mean_x_mg The destination for the mean X acceleration.
 * @param p_mean_y_mg The destination for the mean Y acceleration.
 * @param p_mean_z_mg The destination for the mean Z acceleration.
 * @return The calibration result.
 */
static b4_types_result_t
b4_cfg_stationary_mean (const b4_types_sample_t * p_samples,
                        uint16_t sample_count,
                        uint32_t noise_limit_mg,
                        int32_t * p_mean_x_mg,
                        int32_t * p_mean_y_mg,
                        int32_t * p_mean_z_mg)
{
    b4_types_result_t result = B4_RESULT_OK;
    int32_t sum_x_mg = 0;
    int32_t sum_y_mg = 0;
    int32_t sum_z_mg = 0;
    uint32_t max_deviation_mg = 0U;
    uint16_t sample_index;

    for (sample_index = 0U; sample_index < sample_count; sample_index++)
    {
        if ((false == p_samples[sample_index].b_valid) ||
            (false == b4_cfg_sample_is_in_range(&p_samples[sample_index])))
        {
            result = B4_RESULT_INVALID_CALIBRATION;
        }
        else
        {
            sum_x_mg += p_samples[sample_index].accel_x_mg;
            sum_y_mg += p_samples[sample_index].accel_y_mg;
            sum_z_mg += p_samples[sample_index].accel_z_mg;
        }
    }

    if (B4_RESULT_OK == result)
    {
        *p_mean_x_mg = sum_x_mg / (int32_t)sample_count;
        *p_mean_y_mg = sum_y_mg / (int32_t)sample_count;
        *p_mean_z_mg = sum_z_mg / (int32_t)sample_count;

        for (sample_index = 0U; sample_index < sample_count; sample_index++)
        {
            uint32_t deviation_mg = 0U;
            uint32_t axis_deviation_mg = 0U;

            axis_deviation_mg = b4_cfg_abs_mg(
                p_samples[sample_index].accel_x_mg - *p_mean_x_mg);
            if (axis_deviation_mg > deviation_mg)
            {
                deviation_mg = axis_deviation_mg;
            }

            axis_deviation_mg = b4_cfg_abs_mg(
                p_samples[sample_index].accel_y_mg - *p_mean_y_mg);
            if (axis_deviation_mg > deviation_mg)
            {
                deviation_mg = axis_deviation_mg;
            }

            axis_deviation_mg = b4_cfg_abs_mg(
                p_samples[sample_index].accel_z_mg - *p_mean_z_mg);
            if (axis_deviation_mg > deviation_mg)
            {
                deviation_mg = axis_deviation_mg;
            }

            if (deviation_mg > max_deviation_mg)
            {
                max_deviation_mg = deviation_mg;
            }
        }

        if (max_deviation_mg > noise_limit_mg)
        {
            result = B4_RESULT_INVALID_CALIBRATION;
        }
    }

    return (result);
}
