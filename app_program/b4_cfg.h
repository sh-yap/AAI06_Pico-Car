/** @file b4_cfg.h
 *
 * @brief Buddy 4 hardware, calibration, and processing configuration.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_CFG_H
#define B4_CFG_H

#include <stdint.h>

#include "b4_types.h"

#define B4_CFG_AXIS_X 0U
#define B4_CFG_AXIS_Y 1U
#define B4_CFG_AXIS_Z 2U

/** @brief ROBO-PICO hardware, calibration, and processing configuration.
 *
 * The ROBO-PICO IMU uses Grove port 2, which maps I2C1 to SDA=GP2 and
 * SCL=GP3. I2C runs at 400 kHz. Accelerometer and algorithm rates use the
 * units in their names.
 *
 * I2C units are zero-based. SDA and SCL fields contain GPIO numbers. The
 * address uses the seven-bit accelerometer form. Rates are in hertz and the
 * accelerometer range uses g.
 *
 * Axis fields use B4_CFG_AXIS_X, B4_CFG_AXIS_Y, or B4_CFG_AXIS_Z. Each sign
 * is +1 or -1; positive maps to forward, right, or up for the chosen axis.
 *
 * Bias fields are in milli-g. A bias is the stationary residual of one axis
 * in sensor coordinates, measured minus expected, so a positive bias means
 * the sensor reads high. Applying a bias means subtracting it from the
 * measured value, calibrated = measured - bias. The X, Y, and Z bias fields
 * always address the sensor axes, not the configured forward, lateral, and
 * vertical axes. b4_cfg_calibrate_stationary is the only supported writer.
 *
 * Processing thresholds are in milli-g. Sample age and event intervals are
 * in milliseconds. Sample counts are consecutive samples. A filter divisor
 * of one passes each input directly.
 */
typedef struct
{
    uint8_t i2c_unit;
    uint8_t sda_gpio;
    uint8_t scl_gpio;
    uint8_t accel_i2c_address;
    uint8_t accel_data_rate_hz;
    uint8_t accel_range_g;
    uint32_t i2c_frequency_hz;
    uint8_t forward_axis;
    uint8_t lateral_axis;
    uint8_t vertical_axis;
    int8_t forward_sign;
    int8_t lateral_sign;
    int8_t vertical_sign;
    int32_t bias_x_mg;
    int32_t bias_y_mg;
    int32_t bias_z_mg;
    uint32_t gravity_reference_mg;
    uint32_t stale_after_ms;
    uint32_t motion_threshold_mg;
    uint32_t tilt_threshold_mg;
    uint32_t impact_delta_threshold_mg;
    uint32_t impact_refractory_ms;
    uint32_t hump_start_threshold_mg;
    uint32_t hump_hysteresis_mg;
    uint32_t hump_complete_threshold_mg;
    uint32_t calibration_noise_limit_mg;
    uint8_t filter_divisor;
    uint8_t hump_confirm_samples;
    uint8_t hump_complete_samples;
    uint8_t max_consecutive_read_failures;
} b4_cfg_config_t;

/** Return the ROBO-PICO car-test defaults. */
b4_cfg_config_t
b4_cfg_default (void);

/** Validate hardware, orientation, calibration, and algorithm settings. */
b4_types_result_t
b4_cfg_validate (const b4_cfg_config_t * p_config);

/** Calibrate per-axis bias values from stationary samples in milli-g.
 *
 * Each stored bias is the mean stationary sample minus the expected value
 * for that axis, so a sensor that reads high stores a positive bias. The
 * samples must be uncorrected readings, collected before any bias is
 * applied, for example while the configuration is at its b4_cfg_default()
 * values. The adapter applies the stored bias to every sample it returns,
 * so recalibrating after a bias is stored requires clearing the bias fields
 * first. */
b4_types_result_t
b4_cfg_calibrate_stationary (const b4_types_sample_t * p_samples,
                             uint16_t sample_count,
                             int32_t expected_x_mg,
                             int32_t expected_y_mg,
                             int32_t expected_z_mg,
                             b4_cfg_config_t * p_config);

#endif /* B4_CFG_H */
