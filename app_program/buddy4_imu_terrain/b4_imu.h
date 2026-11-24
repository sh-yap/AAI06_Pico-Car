/** @file b4_imu.h
 *
 * @brief Buddy 4 LSM303DLHC accelerometer and magnetometer interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_IMU_H
#define B4_IMU_H

#include <stdint.h>

#include "b4_cfg.h"
#include "b4_types.h"

/** @brief Initialize the accelerometer through the project I2C device.
 *
 * Calling this while the adapter is already initialized returns success
 * without reopening or reconfiguring the device, so the first successful
 * call keeps its configuration. Newly calibrated bias values take effect
 * after b4_imu_close followed by another initialization with the updated
 * configuration.
 *
 * The magnetometer is configured in the same call. A magnetometer that
 * cannot be configured leaves the accelerometer usable and the
 * magnetometer unavailable, which can be checked with b4_imu_get_mag_health.
 *
 * @param p_config The validated hardware and calibration configuration.
 * @return The initialization result.
 */
b4_types_result_t
b4_imu_init (const b4_cfg_config_t * p_config);

/** @brief Read and calibrate one three-axis accelerometer sample.
 *
 * Each returned axis has the stored bias for that sensor axis subtracted.
 *
 * @param p_sample The destination for the calibrated sample.
 * @return The sample result.
 */
b4_types_result_t
b4_imu_read (b4_types_sample_t * p_sample);

/** @brief Copy sensor health, fault, and consecutive read failure count.
 *
 * @param p_health The destination for the current sensor health.
 * @param p_fault The destination for the current sensor fault.
 * @param p_failure_count The destination for the consecutive failure count.
 * @return The status result.
 */
b4_types_result_t
b4_imu_get_health (b4_types_health_t * p_health,
                   b4_types_fault_t * p_fault,
                   uint8_t * p_failure_count);

/** @brief Read one magnetometer sample when the sensor has new data.
 *
 * A sample whose data-ready bit is clear, or whose decoded counts are out of
 * range, is returned with its validity flag false. The result distinguishes
 * that harmless condition from a communication failure, so the caller can
 * keep reporting unavailable heading instead of a fabricated direction.
 *
 * @param p_mag_sample The destination for the magnetometer sample.
 * @return The sample result.
 */
b4_types_result_t
b4_imu_read_magnetometer (b4_types_mag_sample_t * p_mag_sample);

/** @brief Copy magnetometer health, fault, and consecutive failure count.
 *
 * @param p_health The destination for the current magnetometer health.
 * @param p_fault The destination for the current magnetometer fault.
 * @param p_failure_count The destination for the consecutive failure count.
 * @return The status result.
 */
b4_types_result_t
b4_imu_get_mag_health (b4_types_health_t * p_health,
                       b4_types_fault_t * p_fault,
                       uint8_t * p_failure_count);

/** @brief Power down the accelerometer and close its I2C device handle.
 *
 * A later initialization applies the configuration it is given, so closing
 * is the step that lets newly calibrated bias values take effect.
 *
 * @return The close result.
 */
b4_types_result_t
b4_imu_close (void);

#endif /* B4_IMU_H */
