#ifndef BUDDY2_MOTION_TYPES_H
#define BUDDY2_MOTION_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_COMMAND_SET_VELOCITY = 0,
    MOTION_COMMAND_MOVE_DISTANCE,
    MOTION_COMMAND_TURN_ANGLE
} motion_command_type_t;

typedef enum {
    MOTION_STATE_UNINITIALISED = 0,
    MOTION_STATE_IDLE,
    MOTION_STATE_RUNNING,
    MOTION_STATE_COMPLETE,
    MOTION_STATE_STOPPED,
    MOTION_STATE_FAULT
} motion_state_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_TIMEOUT,
    MOTION_FAULT_STALL,
    MOTION_FAULT_ENCODER,
    MOTION_FAULT_INVALID_COMMAND,
    MOTION_FAULT_HARDWARE_CONFIG
} motion_fault_t;

typedef enum {
    MOTION_RESULT_OK = 0,
    MOTION_RESULT_INVALID,
    MOTION_RESULT_UNINITIALISED,
    MOTION_RESULT_BUSY,
    MOTION_RESULT_QUEUE_FULL,
    MOTION_RESULT_HARDWARE,
    MOTION_RESULT_TIMEOUT
} motion_result_t;

typedef struct {
    motion_command_type_t type;
    uint32_t sequence_id;
    uint32_t timeout_ms;
    union {
        struct { int32_t linear_mm_s; int32_t angular_mdeg_s; } velocity;
        struct { int32_t distance_mm; int32_t speed_mm_s; } distance;
        struct { int32_t angle_mdeg; int32_t wheel_speed_mm_s; } turn;
    } value;
} motion_command_t;

typedef struct {
    uint32_t active_sequence_id;
    motion_state_t state;
    motion_fault_t fault;
    int32_t left_encoder_counts;
    int32_t right_encoder_counts;
    int32_t left_speed_mm_s;
    int32_t right_speed_mm_s;
    int32_t travelled_mm;
    int32_t target_mm;
    int32_t target_speed_mm_s;
    int32_t target_angle_mdeg;
    uint16_t left_pwm_permille;
    uint16_t right_pwm_permille;
    uint32_t left_invalid_encoder_transitions;
    uint32_t right_invalid_encoder_transitions;
    uint32_t invalid_encoder_transitions;
    uint32_t encoder_irq_count;
    uint64_t timestamp_us;
    /* Monotonic timestamp at the most recent state transition. */
    uint64_t state_timestamp_us;
    bool valid;
} motion_status_t;

#endif
