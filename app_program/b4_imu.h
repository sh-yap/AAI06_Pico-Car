/** @file b4_imu.h
 *
 * @brief Buddy 4 LSM303DLHC accelerometer interface.
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
 * @param p_config The validated hardware and calibration configuration.
 * @return The initialization result.
 */
b4_types_result_t
b4_imu_init (const b4_cfg_config_t * p_config);

/** @brief Read and calibrate one three-axis accelerometer sample.
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

/** @brief Power down the accelerometer and close its I2C device handle.
 *
 * @return The close result.
 */
b4_types_result_t
b4_imu_close (void);

#endif /* B4_IMU_H */
