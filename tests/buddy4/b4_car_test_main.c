/** @file b4_car_test_main.c
 *
 * @brief Buddy 4 ROBO-PICO diagnostic firmware entry point.
 *
 * This module owns only the dedicated image entry point. Sensor access,
 * calibration, processing, and diagnostic records remain in their modules.
 *
 * @par
 * SPDX-License-Identifier: Apache-2.0
 */
#include "b4_car_test_main.h"

#include "b4_car_test.h"

/** @brief Enter the dedicated Buddy 4 diagnostic firmware.
 *
 * @return Zero after the diagnostic harness returns.
 */
EXPORT INT
usermain (void)
{
    b4_car_test_run();

    return (0);
}

/*** end of file ***/
