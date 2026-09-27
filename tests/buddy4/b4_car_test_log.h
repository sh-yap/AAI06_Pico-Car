/** @file b4_car_test_log.h
 *
 * @brief Buddy 4 diagnostic name and record-format support.
 *
 * @par
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_CAR_TEST_LOG_H
#define B4_CAR_TEST_LOG_H

#include <stdint.h>

#include <tk/tkernel.h>

#include "b4_cfg.h"
#include "b4_types.h"

/** Return the console name of a Buddy 4 result. */
const UB *
b4_car_test_log_result_name (b4_types_result_t result);

/** Return the console name of a Buddy 4 fault. */
const UB *
b4_car_test_log_fault_name (b4_types_fault_t fault);

/** Return the console name of a Buddy 4 health level. */
const UB *
b4_car_test_log_health_name (b4_types_health_t health);

/** Return the console name of a Buddy 4 tilt direction. */
const UB *
b4_car_test_log_tilt_name (b4_types_tilt_t tilt);

/** Return the console name of a Buddy 4 motion classification. */
const UB *
b4_car_test_log_motion_name (b4_types_motion_t motion);

/** Return the console name of a Buddy 4 hump state. */
const UB *
b4_car_test_log_hump_name (b4_types_hump_state_t hump_state);

/** Return the console name of a configured sensor axis. */
const UB *
b4_car_test_log_axis_name (uint8_t axis);

/** Return the console sign of a configured axis direction. */
const UB *
b4_car_test_log_sign_name (int8_t sign);

#endif /* B4_CAR_TEST_LOG_H */

/*** end of file ***/
