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
#define B4_CFG_I2C_SDA_GPIO                 2U
#define B4_CFG_I2C_SCL_GPIO                 3U
#define B4_CFG_I2C_FREQUENCY_HZ        400000U
#define B4_CFG_ACCEL_ADDRESS_LOW         0x18U
#define B4_CFG_ACCEL_ADDRESS_HIGH        0x19U
#define B4_CFG_ACCEL_DATA_RATE_HZ          100U
#define B4_CFG_ACCEL_RANGE_G                 2U
#define B4_CFG_MAX_SAMPLE_MG              2000
#define B4_CFG_MAX_THRESHOLD_MG           6000U
#define B4_CFG_MIN_CAL_SAMPLES                4U
#define B4_CFG_MAX_CAL_SAMPLES              128U
#define B4_CFG_MIN_FILTER_DIVISOR             1U
#define B4_CFG_MAX_FILTER_DIVISOR            16U
#define B4_CFG_MAX_AXIS                       2U

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
 * @return A configuration using ROBO-PICO Grove Port 2 and provisional
 *         thresholds.
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
    config.i2c_frequency_hz = B4_CFG_I2C_FREQUENCY_HZ;
    config.forward_axis = B4_CFG_AXIS_Y;
    config.lateral_axis = B4_CFG_AXIS_X;
    config.vertical_axis = B4_CFG_AXIS_Z;
    config.forward_sign = 1;
    config.lateral_sign = 1;
    config.vertical_sign = 1;
    config.gravity_reference_mg = 1000U;
    config.stale_after_ms = 100U;
    config.motion_threshold_mg = 120U;
    config.tilt_threshold_mg = 250U;
    config.impact_delta_threshold_mg = 700U;
    config.impact_refractory_ms = 200U;
    config.hump_start_threshold_mg = 100U;
    config.hump_hysteresis_mg = 40U;
    config.hump_complete_threshold_mg = 60U;
    config.calibration_noise_limit_mg = 35U;
    config.filter_divisor = 4U;
    config.hump_confirm_samples = 2U;
    config.hump_complete_samples = 3U;
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
            (p_config->forward_axis != p_config->lateral_axis) &&
            (p_config->forward_axis != p_config->vertical_axis) &&
            (p_config->lateral_axis != p_config->vertical_axis) &&
            (b4_cfg_sign_is_valid(p_config->forward_sign)) &&
            (b4_cfg_sign_is_valid(p_config->lateral_sign)) &&
            (b4_cfg_sign_is_valid(p_config->vertical_sign));

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
            (p_config->hump_start_threshold_mg <=
             p_config->hump_complete_threshold_mg) ||
            (p_config->hump_start_threshold_mg >
             B4_CFG_MAX_THRESHOLD_MG) ||
            (0U == p_config->hump_hysteresis_mg) ||
            (p_config->hump_hysteresis_mg >
             B4_CFG_MAX_THRESHOLD_MG) ||
            (p_config->hump_complete_threshold_mg >
             B4_CFG_MAX_THRESHOLD_MG) ||
            (0U == p_config->calibration_noise_limit_mg) ||
            (p_config->calibration_noise_limit_mg >
             (uint32_t)B4_CFG_MAX_SAMPLE_MG) ||
            (p_config->filter_divisor < B4_CFG_MIN_FILTER_DIVISOR) ||
            (p_config->filter_divisor > B4_CFG_MAX_FILTER_DIVISOR) ||
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
 * @return True only for I2C1 on Grove Port 2, GP2 and GP3.
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
