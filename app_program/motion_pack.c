/** @file motion_pack.c
 *
 * @brief Packed Motion command and status records (CR1, see
 *        docs/DATA_REPRESENTATION.md section 1).
 *
 * Only the stored or queued copies are packed.  motion_command_t and
 * motion_status_t stay full-width value types for callers and calculations.
 */

#include <stddef.h>

#include "motion_types.h"

#define MOTION_COMMAND_TYPE_MAX 3U
#define MOTION_STATE_MAX 6U
#define MOTION_FAULT_MAX 6U
#define MOTION_ENCODER_FAULT_MAX 4U
#define MOTION_TIMEOUT_UNITS_MAX 0x3FFFU
#define MOTION_TIMEOUT_MS_MAX (MOTION_TIMEOUT_UNITS_MAX * 10U)

static uint32_t field_mask(unsigned bits)
{
    return bits >= 32U ? 0xFFFFFFFFU : ((1U << bits) - 1U);
}

/* Clamp a measured value to its signed field and encode two's complement. */
static uint32_t field_signed(int32_t value, int32_t minimum, int32_t maximum,
                             unsigned bits)
{
    if (value < minimum) {
        value = minimum;
    }
    if (value > maximum) {
        value = maximum;
    }
    return (uint32_t)value & field_mask(bits);
}

static uint32_t field_unsigned(uint32_t value, uint32_t maximum, unsigned bits)
{
    if (value > maximum) {
        value = maximum;
    }
    return value & field_mask(bits);
}

static uint32_t field_wrap(uint32_t value, unsigned bits)
{
    return value & field_mask(bits);
}

static int32_t field_signed_get(uint32_t word, unsigned shift, unsigned bits)
{
    uint32_t mask = field_mask(bits);
    uint32_t value = (word >> shift) & mask;
    uint32_t sign = 1U << (bits - 1U);

    if ((value & sign) != 0U) {
        return -(int32_t)((~value & mask) + 1U);
    }
    return (int32_t)value;
}

static uint32_t timeout_units(uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return 0U;
    }
    if (timeout_ms > MOTION_TIMEOUT_MS_MAX) {
        return MOTION_TIMEOUT_UNITS_MAX;
    }
    /* Round up: a stored timeout must never be shorter than requested. */
    return (timeout_ms + 9U) / 10U;
}

bool motion_pack_command(const motion_command_t *command,
                         motion_command_packed_t *packed)
{
    uint32_t type;
    uint32_t payload;

    if (command == NULL || packed == NULL) {
        return false;
    }
    type = (uint32_t)command->type;
    if (type >= MOTION_COMMAND_TYPE_MAX) {
        return false;
    }
    packed->word[0] = type
                      | (field_wrap(command->sequence_id, 16U) << 2)
                      | (timeout_units(command->timeout_ms) << 18);
    if (command->type == MOTION_COMMAND_SET_VELOCITY) {
        payload = field_signed(command->value.velocity.linear_mm_s,
                               -512, 511, 10U)
                  | (field_signed(command->value.velocity.angular_mdeg_s,
                                  -524288, 524287, 20U) << 10);
    } else if (command->type == MOTION_COMMAND_MOVE_DISTANCE) {
        payload = field_signed(command->value.distance.distance_mm,
                               -4096, 4095, 13U)
                  | (field_signed(command->value.distance.speed_mm_s,
                                  -512, 511, 10U) << 13);
    } else {
        payload = field_signed(command->value.turn.angle_mdeg,
                               -524288, 524287, 20U)
                  | (field_signed(command->value.turn.wheel_speed_mm_s,
                                  -512, 511, 10U) << 20);
    }
    packed->word[1] = payload;
    return true;
}

bool motion_unpack_command(const motion_command_packed_t *packed,
                           motion_command_t *command)
{
    uint32_t type;

    if (packed == NULL || command == NULL) {
        return false;
    }
    type = packed->word[0] & 3U;
    if (type >= MOTION_COMMAND_TYPE_MAX) {
        return false;
    }
    command->type = (motion_command_type_t)type;
    command->sequence_id = (packed->word[0] >> 2) & 0xFFFFU;
    command->timeout_ms = ((packed->word[0] >> 18) & MOTION_TIMEOUT_UNITS_MAX)
                          * 10U;
    if (command->type == MOTION_COMMAND_SET_VELOCITY) {
        command->value.velocity.linear_mm_s =
            field_signed_get(packed->word[1], 0U, 10U);
        command->value.velocity.angular_mdeg_s =
            field_signed_get(packed->word[1], 10U, 20U);
    } else if (command->type == MOTION_COMMAND_MOVE_DISTANCE) {
        command->value.distance.distance_mm =
            field_signed_get(packed->word[1], 0U, 13U);
        command->value.distance.speed_mm_s =
            field_signed_get(packed->word[1], 13U, 10U);
    } else {
        command->value.turn.angle_mdeg =
            field_signed_get(packed->word[1], 0U, 20U);
        command->value.turn.wheel_speed_mm_s =
            field_signed_get(packed->word[1], 20U, 10U);
    }
    return true;
}

bool motion_pack_status(const motion_status_t *status,
                        motion_status_packed_t *packed)
{
    uint32_t valid;

    if (status == NULL || packed == NULL) {
        return false;
    }
    valid = status->valid ? 1U : 0U;
    if ((uint32_t)status->state >= MOTION_STATE_MAX
        || (uint32_t)status->fault >= MOTION_FAULT_MAX
        || (uint32_t)status->encoder_fault >= MOTION_ENCODER_FAULT_MAX) {
        valid = 0U;
    }
    packed->live[0] = field_wrap(status->active_sequence_id, 16U)
                      | (field_wrap((uint32_t)status->state, 3U) << 16)
                      | (field_wrap((uint32_t)status->fault, 3U) << 19)
                      | (field_wrap((uint32_t)status->encoder_fault, 2U) << 22)
                      | (field_wrap(status->encoder_fault_wheel_mask, 2U) << 24)
                      | (valid << 26);
    /* Measured wheel speed can exceed the 300 mm/s command limit while a
       sample is being validated, so it gets 12 signed bits (the physical
       bound is 1000 mm/s).  Targets and slew stay at 10. */
    packed->live[1] = field_signed(status->left_speed_mm_s, -2048, 2047, 12U)
                      | (field_signed(status->right_speed_mm_s,
                                      -2048, 2047, 12U) << 12);
    packed->live[2] = field_signed(status->left_target_mm_s, -512, 511, 10U)
                      | (field_signed(status->right_target_mm_s,
                                      -512, 511, 10U) << 10)
                      | (field_signed(status->left_slewed_target_mm_s,
                                      -512, 511, 10U) << 20);
    packed->live[3] = field_signed(status->right_slewed_target_mm_s,
                                   -512, 511, 10U)
                      | (field_signed(status->left_drive_permille,
                                      -1024, 1023, 11U) << 10)
                      | (field_signed(status->right_drive_permille,
                                      -1024, 1023, 11U) << 21);
    packed->live[4] = field_unsigned(status->left_pwm_permille, 1023U, 10U)
                      | (field_unsigned(status->right_pwm_permille,
                                       1023U, 10U) << 10)
                      | (field_signed(status->target_speed_mm_s,
                                      -512, 511, 10U) << 20);
    packed->live[5] = field_signed(status->travelled_mm,
                                   -8388608, 8388607, 24U);
    packed->live[6] = field_signed(status->target_mm,
                                   -8388608, 8388607, 24U);
    packed->live[7] = field_signed(status->target_angle_mdeg,
                                   -524288, 524287, 20U);
    /* Boot-relative counters are never reset, so they get their own full
       word: 24 s would clamp at +-599 m of travel. */
    packed->live[8] = (uint32_t)status->left_encoder_counts;
    packed->live[9] = (uint32_t)status->right_encoder_counts;

    packed->diag[0] = field_unsigned(status->encoder_fault_elapsed_ms,
                                     65535U, 16U)
                      | (field_signed(status->encoder_fault_left_delta,
                                      -32768, 32767, 16U) << 16);
    packed->diag[1] = field_signed(status->encoder_fault_right_delta,
                                   -32768, 32767, 16U);
    /* Acquisition can be delayed by preemption; the regression replay uses
       100 ms, so this is kept as a full 32-bit microsecond value. */
    packed->diag[2] = status->encoder_snapshot_duration_us;
    packed->diag[3] = field_signed(status->encoder_fault_previous_left,
                                   -8388608, 8388607, 24U);
    packed->diag[4] = field_signed(status->encoder_fault_previous_right,
                                   -8388608, 8388607, 24U);
    packed->diag[5] = field_wrap(status->encoder_fault_previous_irq, 24U);
    packed->diag[6] = (uint32_t)status->encoder_fault_previous_time_us;
    packed->diag[7] = (uint32_t)status->encoder_sample_time_us;
    packed->diag[8] = (uint32_t)status->timestamp_us;
    packed->diag[9] = (uint32_t)(status->timestamp_us >> 32);
    packed->diag[10] = (uint32_t)status->state_timestamp_us;
    packed->diag[11] = (uint32_t)(status->state_timestamp_us >> 32);
    packed->diag[12] = field_unsigned(status->encoder_fault_previous_left_pwm,
                                      1023U, 10U)
                       | (field_unsigned(status->encoder_fault_previous_right_pwm,
                                         1023U, 10U) << 10);
    packed->diag[13] = field_wrap(status->left_invalid_encoder_transitions, 16U)
                       | (field_wrap(status->right_invalid_encoder_transitions,
                                     16U) << 16);
    packed->diag[14] = field_wrap(status->invalid_encoder_transitions, 16U);
    packed->diag[15] = field_wrap(status->encoder_irq_count, 24U);
    return valid != 0U;
}

bool motion_unpack_status(const motion_status_packed_t *packed,
                          motion_status_t *status)
{
    uint32_t state;
    uint32_t fault;
    uint32_t encoder_fault;
    bool enums_valid;
    bool valid;

    if (packed == NULL || status == NULL) {
        return false;
    }
    state = (packed->live[0] >> 16) & 7U;
    fault = (packed->live[0] >> 19) & 7U;
    encoder_fault = (packed->live[0] >> 22) & 3U;
    enums_valid = state < MOTION_STATE_MAX
                  && fault < MOTION_FAULT_MAX
                  && encoder_fault < MOTION_ENCODER_FAULT_MAX;
    valid = ((packed->live[0] >> 26) & 1U) != 0U && enums_valid;

    status->active_sequence_id = packed->live[0] & 0xFFFFU;
    status->state = enums_valid ? (motion_state_t)state
                                : MOTION_STATE_UNINITIALISED;
    status->fault = enums_valid ? (motion_fault_t)fault : MOTION_FAULT_NONE;
    status->encoder_fault = enums_valid
                            ? (motion_encoder_fault_t)encoder_fault
                            : MOTION_ENCODER_FAULT_NONE;
    status->encoder_fault_wheel_mask =
        (uint8_t)((packed->live[0] >> 24) & 3U);
    status->valid = valid;
    status->left_speed_mm_s =
        field_signed_get(packed->live[1], 0U, 12U);
    status->right_speed_mm_s =
        field_signed_get(packed->live[1], 12U, 12U);
    status->left_target_mm_s =
        field_signed_get(packed->live[2], 0U, 10U);
    status->right_target_mm_s =
        field_signed_get(packed->live[2], 10U, 10U);
    status->left_slewed_target_mm_s =
        field_signed_get(packed->live[2], 20U, 10U);
    status->right_slewed_target_mm_s =
        field_signed_get(packed->live[3], 0U, 10U);
    status->left_drive_permille =
        field_signed_get(packed->live[3], 10U, 11U);
    status->right_drive_permille =
        field_signed_get(packed->live[3], 21U, 11U);
    status->left_pwm_permille = (uint16_t)(packed->live[4] & 0x3FFU);
    status->right_pwm_permille = (uint16_t)((packed->live[4] >> 10) & 0x3FFU);
    status->target_speed_mm_s =
        field_signed_get(packed->live[4], 20U, 10U);
    status->travelled_mm = field_signed_get(packed->live[5], 0U, 24U);
    status->target_mm = field_signed_get(packed->live[6], 0U, 24U);
    status->target_angle_mdeg = field_signed_get(packed->live[7], 0U, 20U);
    status->left_encoder_counts = (int32_t)packed->live[8];
    status->right_encoder_counts = (int32_t)packed->live[9];

    status->encoder_fault_elapsed_ms = packed->diag[0] & 0xFFFFU;
    status->encoder_fault_left_delta =
        field_signed_get(packed->diag[0], 16U, 16U);
    status->encoder_fault_right_delta =
        field_signed_get(packed->diag[1], 0U, 16U);
    status->encoder_snapshot_duration_us = packed->diag[2];
    status->encoder_fault_previous_left =
        field_signed_get(packed->diag[3], 0U, 24U);
    status->encoder_fault_previous_right =
        field_signed_get(packed->diag[4], 0U, 24U);
    status->encoder_fault_previous_irq = packed->diag[5] & 0xFFFFFFU;
    status->encoder_fault_previous_time_us = packed->diag[6];
    status->encoder_sample_time_us = packed->diag[7];
    status->timestamp_us = ((uint64_t)packed->diag[9] << 32) | packed->diag[8];
    status->state_timestamp_us =
        ((uint64_t)packed->diag[11] << 32) | packed->diag[10];
    status->encoder_fault_previous_left_pwm =
        (uint16_t)(packed->diag[12] & 0x3FFU);
    status->encoder_fault_previous_right_pwm =
        (uint16_t)((packed->diag[12] >> 10) & 0x3FFU);
    status->left_invalid_encoder_transitions = packed->diag[13] & 0xFFFFU;
    status->right_invalid_encoder_transitions =
        (packed->diag[13] >> 16) & 0xFFFFU;
    status->invalid_encoder_transitions = packed->diag[14] & 0xFFFFU;
    status->encoder_irq_count = packed->diag[15] & 0xFFFFFFU;
    return valid;
}
