/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * comms.h
 * Communication API: telemetry, events and commands over WiFi/MQTT.
 * Owner: Buddy 1.
 *
 * Rules for callers
 *   - Every function returns immediately; none of them waits for the
 *     network.  With WiFi down (or a build without WiFi) data is simply
 *     kept or dropped, and the car is unaffected.
 *   - Each comms_report_*() function must be called from ONE task only
 *     (the owner of that data).  Events and commands may be used from any
 *     task.
 *   - Units as each owner's own data (see the structs).  No floating point.
 *   - Call comms_init() once from usermain() before starting any task.
 *
 * Payload format: packed binary by default (CR1), or JSON when built with
 * COMMS_FORMAT=json; fields and bit widths in docs/BINARY_FORMAT.md.  The
 * laptop decodes both with tools/robot_decode.ps1.
 */
#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ *
 *  Shared vocabulary (names follow the project roadmap)
 * ------------------------------------------------------------------ */

typedef enum
{
    COMMS_STATE_INIT = 0,
    COMMS_STATE_READY,
    COMMS_STATE_LINE_FOLLOW,
    COMMS_STATE_EXECUTE_NAV,
    COMMS_STATE_OBSTACLE_STOP,
    COMMS_STATE_OBSTACLE_SCAN,
    COMMS_STATE_BYPASS,
    COMMS_STATE_LINE_RECOVERY,
    COMMS_STATE_FINISHED,
    COMMS_STATE_SAFE_STOP,
    COMMS_STATE_COUNT
} comms_state_t;

typedef enum
{
    COMMS_NAV_NONE = 0,
    COMMS_NAV_LEFT,
    COMMS_NAV_RIGHT,
    COMMS_NAV_STRAIGHT,
    COMMS_NAV_UTURN,
    COMMS_NAV_COUNT
} comms_nav_t;

/* Same numbering as Buddy 3's line_state_t (docs/DATA_DEFINITIONS.md). */
typedef enum
{
    COMMS_LINE_LEFT = 0,
    COMMS_LINE_CENTRE,
    COMMS_LINE_RIGHT,
    COMMS_LINE_LOST,
    COMMS_LINE_JUNCTION,
    COMMS_LINE_COUNT
} comms_line_pos_t;

/* Same numbering as Buddy 4's b4_types_motion_t.  QUIET means the readings
 * barely changed; it does NOT prove the car is stationary. */
typedef enum
{
    COMMS_MOTION_QUIET = 0,
    COMMS_MOTION_ACCELERATING,
    COMMS_MOTION_CLIMBING_HUMP,
    COMMS_MOTION_DESCENDING_HUMP,
    COMMS_MOTION_IMPACT,
    COMMS_MOTION_COUNT
} comms_motion_class_t;

typedef enum
{
    COMMS_AVOID_STOP = 0,
    COMMS_AVOID_CONTINUE,
    COMMS_AVOID_TURN_LEFT,
    COMMS_AVOID_TURN_RIGHT,
    COMMS_AVOID_REVERSE_RETRY,
    COMMS_AVOID_COUNT
} comms_avoid_t;

typedef enum
{
    COMMS_CMD_NONE = 0,
    COMMS_CMD_START,        /* begin the mission                        */
    COMMS_CMD_STOP          /* stop safely (controller decides how)     */
} comms_cmd_t;

/* ------------------------------------------------------------------ *
 *  Telemetry snapshots: latest value wins, sent at most 5 times/s
 * ------------------------------------------------------------------ */

/* Buddy 2 - motion control. */
typedef struct
{
    int32_t speed_left_mm_s;
    int32_t speed_right_mm_s;
    int32_t encoder_left;       /* raw counts since boot                */
    int32_t encoder_right;
    int32_t distance_mm;        /* travelled since boot                 */
} comms_motion_t;

/* Buddy 3 - line sensing. */
typedef struct
{
    comms_line_pos_t position;
    int32_t          error;     /* -127 (left) .. +127 (right)          */
    uint8_t          sensors;   /* bit0 centre (GP27), bit1 right
                                   (GP26); 1 = black.  2 line sensors
                                   (Buddy 3); GP28 is barcode only.    */
} comms_line_t;

/* Buddy 4 - IMU / terrain (Buddy 4's own units). */
typedef struct
{
    comms_motion_class_t motion;
    int32_t              tilt_mg;    /* pitch, -1000 .. +1000 milli-g   */
    int32_t              hump_dmm;   /* current hump height, 0.1 mm
                                        steps, 0 .. 800 (0 .. 80.0 mm)  */
} comms_terrain_t;

/* Buddy 5 - obstacle profile (sent as an event). */
typedef struct
{
    int32_t       distance_mm;  /* closest point                        */
    int32_t       angle_deg;    /* bearing of closest point, 90 = ahead */
    int32_t       width_mm;
    int32_t       clear_left_mm;
    int32_t       clear_right_mm;
    comms_avoid_t action;       /* recommended avoidance                */
} comms_obstacle_t;

/* Counters for diagnostics and the resource report. */
typedef struct
{
    bool     b_connected;
    uint32_t events_sent;
    uint32_t events_dropped;    /* queue full: event discarded          */
    uint32_t telemetry_sent;
    uint32_t commands_received;
} comms_stats_t;

/* ------------------------------------------------------------------ *
 *  API
 * ------------------------------------------------------------------ */

bool comms_init(void);
bool comms_is_connected(void);
void comms_get_stats(comms_stats_t * p_stats);

/* Vehicle controller ONLY: these replace the whole state / fault word, so a
 * second caller would overwrite the controller's value. */
void comms_set_state(comms_state_t state);
void comms_set_fault(uint32_t fault_flags);     /* 0 = no fault; bits 0-15
                                                   are sent (DATA_DEFS §6) */
bool comms_begin_run(void);     /* resets the run summary               */
bool comms_end_run(void);       /* publishes the run summary            */
bool comms_get_command(comms_cmd_t * p_cmd);    /* true if one waiting  */

/* Latest-value telemetry (one owner task each). */
void comms_report_motion(comms_motion_t const * p_motion);
void comms_report_line(comms_line_t const * p_line);
void comms_report_terrain(comms_terrain_t const * p_terrain);

/* Events: queued and sent reliably.  false = queue full, event dropped. */
bool comms_event_barcode(char code, comms_nav_t command);
bool comms_event_hump(int32_t peak_dmm);    /* 0.1 mm steps, 0 .. 800   */
bool comms_event_obstacle(comms_obstacle_t const * p_obstacle);
bool comms_event_motion(comms_motion_class_t motion);

#endif /* COMMS_H */
