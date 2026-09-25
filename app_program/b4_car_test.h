/** @file b4_car_test.h
 *
 * @brief Buddy 4 ROBO-PICO car diagnostic harness entry point.
 *
 * The harness is built only when the B4_CAR_TEST build flag is set. It owns no
 * hardware and no algorithms. It drives b4_imu and b4_proc and reports what
 * they return, so a vehicle test run can validate the sensor and processing
 * path without adding a second driver or a second processing path.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_CAR_TEST_H
#define B4_CAR_TEST_H

/* Default to the normal application for builds that do not set the flag. */
#ifndef B4_CAR_TEST
#define B4_CAR_TEST 0
#endif

/** @brief Run the Buddy 4 car test harness.
 *
 * WARNING: this function does not return. It is called directly from the
 * kernel initial task, because the car test firmware creates no RTOS objects
 * and a return from usermain() would shut the system down.
 */
void
b4_car_test_run (void);

#endif /* B4_CAR_TEST_H */
