/** @file b4_types.h
 *
 * @brief Shared Buddy 4 result, sensor, and status types.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef B4_TYPES_H
#define B4_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Results returned by Buddy 4 functions.
 */
typedef enum
{
    B4_RESULT_OK = 0,
    B4_RESULT_INVALID_ARGUMENT,
    B4_RESULT_NOT_INITIALIZED,
    B4_RESULT_HARDWARE_CONFIG_FAILURE,
    B4_RESULT_SENSOR_COMM_FAILURE,
    B4_RESULT_INVALID_CALIBRATION,
    B4_RESULT_INVALID_SAMPLE,
    B4_RESULT_STALE_SAMPLE,
    B4_RESULT_TIMEOUT,
    B4_RESULT_REPEATED_READ_FAILURE
} b4_types_result_t;

/** @brief Fault details for the latest sensor or processing condition.
 */
typedef enum
{
    B4_FAULT_NONE = 0,
    B4_FAULT_HARDWARE_CONFIG,
    B4_FAULT_SENSOR_COMM,
    B4_FAULT_INVALID_CALIBRATION,
    B4_FAULT_INVALID_SAMPLE,
    B4_FAULT_STALE_SAMPLE,
    B4_FAULT_READ_TIMEOUT,
    B4_FAULT_REPEATED_READ,
    B4_FAULT_MAGNETOMETER
} b4_types_fault_t;

/** @brief Sensor and processing health levels.
 */
typedef enum
{
    B4_HEALTH_NOT_INITIALIZED = 0,
    B4_HEALTH_READY,
    B4_HEALTH_DEGRADED,
    B4_HEALTH_FAULT
} b4_types_health_t;

/** @brief Tilt direction from the configured forward and lateral axes.
 */
typedef enum
{
    B4_TILT_UNKNOWN = 0,
    B4_TILT_LEVEL,
    B4_TILT_FORWARD,
    B4_TILT_BACKWARD,
    B4_TILT_LEFT,
    B4_TILT_RIGHT,
    B4_TILT_COMPOUND
} b4_types_tilt_t;

/** @brief Motion classifications supported by the accelerometer pipeline.
 *
 * B4_MOTION_QUIET means the raw sample change is below its threshold. It does
 * not prove that the vehicle is stationary.
 */
typedef enum
{
    B4_MOTION_QUIET = 0,
    B4_MOTION_ACCELERATING,
    B4_MOTION_CLIMBING_HUMP,
    B4_MOTION_DESCENDING_HUMP,
    B4_MOTION_IMPACT
} b4_types_motion_t;

/** @brief States in the vertical acceleration hump detector.
 */
typedef enum
{
    B4_HUMP_NORMAL = 0,
    B4_HUMP_CANDIDATE,
    B4_HUMP_ASCENDING,
    B4_HUMP_PEAK,
    B4_HUMP_DESCENDING,
    B4_HUMP_COMPLETE
} b4_types_hump_state_t;

/** @brief One calibrated accelerometer sample with acceleration in milli-g.
 *
 * The timestamp is monotonic milliseconds since boot. The validity flag is
 * false when the acquisition layer cannot use the sample.
 */
typedef struct
{
    uint32_t timestamp_ms;
    int32_t accel_x_mg;
    int32_t accel_y_mg;
    int32_t accel_z_mg;
    bool b_valid;
} b4_types_sample_t;

/** @brief One magnetometer sample in sensor counts.
 *
 * The timestamp is monotonic milliseconds since boot. The validity flag is
 * false when the acquisition layer cannot use the sample, so a failed read
 * never looks like a real magnetic measurement.
 */
typedef struct
{
    uint32_t timestamp_ms;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    bool b_valid;
} b4_types_mag_sample_t;

/** @brief Turning classification from the sensed magnetic heading change.
 *
 * B4_TURN_UNKNOWN is reported when the magnetometer is missing, invalid, or
 * stale, so an unavailable direction is never presented as a real one.
 */
typedef enum
{
    B4_TURN_UNKNOWN = 0,
    B4_TURN_STRAIGHT,
    B4_TURN_LEFT,
    B4_TURN_RIGHT
} b4_types_turn_t;

/** @brief Public status snapshot for the latest processed sample.
 *
 * Axis samples retain sensor coordinates after offset calibration. Filtered
 * axes and vertical deviation are in milli-g. NOTE: roll_mg and
 * roll_baseline_mg keep their names for the existing diagnostic formats but
 * carry the filtered incline of the configured hump axis, which is pitch
 * when the caster climbs the ramp. Hump peaks are absolute incline
 * amplitudes in milli-g. A hump height estimate is a support-height
 * difference in micrometres, valid only after a settled window was measured
 * during that hump. Event flags describe the latest processed sample and are
 * not latched. Tilt and turn use configured signs.
 */
typedef struct
{
    b4_types_health_t health;
    b4_types_fault_t fault;
    b4_types_tilt_t tilt;
    b4_types_motion_t motion;
    b4_types_hump_state_t hump_state;
    b4_types_turn_t turn;
    b4_types_sample_t sample;
    b4_types_mag_sample_t mag_sample;
    int32_t filtered_x_mg;
    int32_t filtered_y_mg;
    int32_t filtered_z_mg;
    int32_t vertical_deviation_mg;
    int32_t roll_mg;
    int32_t roll_baseline_mg;
    int32_t turn_cross;
    uint32_t current_hump_peak_mg;
    uint32_t highest_hump_peak_mg;
    uint32_t hump_peak_height_um;
    uint32_t highest_hump_height_um;
    bool b_hump_height_valid;
    bool b_mag_valid;
    bool b_sample_stale;
    bool b_impact_event;
    bool b_hump_started_event;
    bool b_hump_completed_event;
} b4_types_status_t;

#endif /* B4_TYPES_H */
