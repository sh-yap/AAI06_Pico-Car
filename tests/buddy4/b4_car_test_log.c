/** @file b4_car_test_log.c
 *
 * @brief Buddy 4 diagnostic enum names for machine-readable records.
 *
 * @par
 * SPDX-License-Identifier: Apache-2.0
 */
#include "b4_car_test_log.h"

static const UB g_result_names[][24] = {
    "OK",
    "INVALID_ARGUMENT",
    "NOT_INITIALIZED",
    "HARDWARE_CONFIG_FAILURE",
    "SENSOR_COMM_FAILURE",
    "INVALID_CALIBRATION",
    "INVALID_SAMPLE",
    "STALE_SAMPLE",
    "TIMEOUT",
    "REPEATED_READ_FAILURE",
    "UNKNOWN_RESULT",
};

static const UB g_fault_names[][20] = {
    "NONE",
    "HARDWARE_CONFIG",
    "SENSOR_COMM",
    "INVALID_CALIBRATION",
    "INVALID_SAMPLE",
    "STALE_SAMPLE",
    "READ_TIMEOUT",
    "REPEATED_READ",
    "UNKNOWN_FAULT",
};

static const UB g_health_names[][16] = {
    "NOT_INITIALIZED",
    "READY",
    "DEGRADED",
    "FAULT",
    "UNKNOWN_HEALTH",
};

static const UB g_tilt_names[][16] = {
    "UNKNOWN",
    "LEVEL",
    "FORWARD",
    "BACKWARD",
    "LEFT",
    "RIGHT",
    "COMPOUND",
    "UNKNOWN_TILT",
};

static const UB g_motion_names[][16] = {
    "QUIET",
    "ACCELERATING",
    "CLIMBING_HUMP",
    "DESCENDING_HUMP",
    "IMPACT",
    "UNKNOWN_MOTION",
};

static const UB g_hump_names[][13] = {
    "NORMAL",
    "CANDIDATE",
    "ASCENDING",
    "PEAK",
    "DESCENDING",
    "COMPLETE",
    "UNKNOWN_HUMP",
};

static const UB g_axis_names[][2] = {"X", "Y", "Z", "?"};
static const UB g_sign_names[][2] = {"+", "-", "?"};

const UB *
b4_car_test_log_result_name (b4_types_result_t result)
{
    const UB * p_name = g_result_names[B4_RESULT_REPEATED_READ_FAILURE + 1];

    if (B4_RESULT_REPEATED_READ_FAILURE >= result)
    {
        p_name = g_result_names[result];
    }

    return (p_name);
}

const UB *
b4_car_test_log_fault_name (b4_types_fault_t fault)
{
    const UB * p_name = g_fault_names[B4_FAULT_REPEATED_READ + 1];

    if (B4_FAULT_REPEATED_READ >= fault)
    {
        p_name = g_fault_names[fault];
    }

    return (p_name);
}

const UB *
b4_car_test_log_health_name (b4_types_health_t health)
{
    const UB * p_name = g_health_names[B4_HEALTH_FAULT + 1];

    if (B4_HEALTH_FAULT >= health)
    {
        p_name = g_health_names[health];
    }

    return (p_name);
}

const UB *
b4_car_test_log_tilt_name (b4_types_tilt_t tilt)
{
    const UB * p_name = g_tilt_names[B4_TILT_COMPOUND + 1];

    if (B4_TILT_COMPOUND >= tilt)
    {
        p_name = g_tilt_names[tilt];
    }

    return (p_name);
}

const UB *
b4_car_test_log_motion_name (b4_types_motion_t motion)
{
    const UB * p_name = g_motion_names[B4_MOTION_IMPACT + 1];

    if (B4_MOTION_IMPACT >= motion)
    {
        p_name = g_motion_names[motion];
    }

    return (p_name);
}

const UB *
b4_car_test_log_hump_name (b4_types_hump_state_t hump_state)
{
    const UB * p_name = g_hump_names[B4_HUMP_COMPLETE + 1];

    if (B4_HUMP_COMPLETE >= hump_state)
    {
        p_name = g_hump_names[hump_state];
    }

    return (p_name);
}

const UB *
b4_car_test_log_axis_name (uint8_t axis)
{
    const UB * p_name = g_axis_names[B4_CFG_AXIS_Z + 1];

    if (B4_CFG_AXIS_Z >= axis)
    {
        p_name = g_axis_names[axis];
    }

    return (p_name);
}

const UB *
b4_car_test_log_sign_name (int8_t sign)
{
    const UB * p_name = g_sign_names[2];

    if (1 == sign)
    {
        p_name = g_sign_names[0];
    }
    else if (-1 == sign)
    {
        p_name = g_sign_names[1];
    }
    else
    {
        p_name = g_sign_names[2];
    }

    return (p_name);
}

/*** end of file ***/
