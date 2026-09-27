/** @file b4_car_test_main.h
 *
 * @brief Buddy 4 diagnostic firmware entry point.
 *
 * @par
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_CAR_TEST_MAIN_H
#define B4_CAR_TEST_MAIN_H

#include <tk/tkernel.h>

/** @brief Enter the dedicated Buddy 4 diagnostic firmware.
 *
 * @return Zero after the diagnostic harness returns.
 */
EXPORT INT
usermain (void);

#endif /* B4_CAR_TEST_MAIN_H */

/*** end of file ***/
