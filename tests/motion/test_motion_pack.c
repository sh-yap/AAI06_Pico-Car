/* Round-trip, clamp, wrap and reject checks for the packed Motion records. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../app_program/buddy2_motion/motion_types.h"

static void check(bool ok, const char *name)
{
    if (!ok) {
        fprintf(stderr, "FAIL: %s\n", name);
        exit(1);
    }
    printf("PASS: %s\n", name);
}

static void command_timeout_check(uint32_t requested, uint32_t stored)
{
    motion_command_t command = {0};
    motion_command_packed_t packed;
    motion_command_t result;

    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.sequence_id = 1U;
    command.timeout_ms = requested;
    check(motion_pack_command(&command, &packed), "timeout command packs");
    check(motion_unpack_command(&packed, &result), "timeout command unpacks");
    check(result.timeout_ms == stored, "timeout rounds up in 10 ms units");
}

static void command_checks(void)
{
    motion_command_t command = {0};
    motion_command_packed_t packed;
    motion_command_t result;

    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.sequence_id = 0U;
    command.timeout_ms = 0U;
    check(motion_pack_command(&command, &packed), "zero command packs");
    check(packed.word[0] == 0U && packed.word[1] == 0U,
          "zero command has zero words");
    check(motion_unpack_command(&packed, &result), "zero command unpacks");
    check(result.type == MOTION_COMMAND_SET_VELOCITY
          && result.sequence_id == 0U && result.timeout_ms == 0U
          && result.value.velocity.linear_mm_s == 0
          && result.value.velocity.angular_mdeg_s == 0,
          "zero command round-trips");

    command.sequence_id = 65535U;
    command.timeout_ms = 163830U;
    command.value.velocity.linear_mm_s = 511;
    command.value.velocity.angular_mdeg_s = 524287;
    check(motion_pack_command(&command, &packed), "maximum command packs");
    check(motion_unpack_command(&packed, &result), "maximum command unpacks");
    check(result.sequence_id == 65535U && result.timeout_ms == 163830U
          && result.value.velocity.linear_mm_s == 511
          && result.value.velocity.angular_mdeg_s == 524287,
          "maximum command round-trips");

    command.sequence_id = 0x10000U;
    command.timeout_ms = 0U;
    command.value.velocity.linear_mm_s = -512;
    command.value.velocity.angular_mdeg_s = -524288;
    check(motion_pack_command(&command, &packed), "minimum command packs");
    check(motion_unpack_command(&packed, &result), "minimum command unpacks");
    check(result.sequence_id == 0U && result.value.velocity.linear_mm_s == -512
          && result.value.velocity.angular_mdeg_s == -524288,
          "minimum command round-trips and sequence wraps");

    command.value.velocity.linear_mm_s = 1000;
    command.value.velocity.angular_mdeg_s = -999999;
    check(motion_pack_command(&command, &packed), "out-of-range velocity packs");
    check(motion_unpack_command(&packed, &result), "out-of-range velocity unpacks");
    check(result.value.velocity.linear_mm_s == 511
          && result.value.velocity.angular_mdeg_s == -524288,
          "velocity values clamp to their fields");

    command.type = MOTION_COMMAND_MOVE_DISTANCE;
    command.value.distance.distance_mm = 3000;
    command.value.distance.speed_mm_s = -300;
    (void)motion_pack_command(&command, &packed);
    (void)motion_unpack_command(&packed, &result);
    check(result.type == MOTION_COMMAND_MOVE_DISTANCE
          && result.value.distance.distance_mm == 3000
          && result.value.distance.speed_mm_s == -300,
          "long bypass distance round-trips");
    command.value.distance.distance_mm = 5000;
    command.value.distance.speed_mm_s = -1000;
    (void)motion_pack_command(&command, &packed);
    (void)motion_unpack_command(&packed, &result);
    check(result.value.distance.distance_mm == 4095
          && result.value.distance.speed_mm_s == -512,
          "distance values clamp to their fields");

    command.type = MOTION_COMMAND_TURN_ANGLE;
    command.value.turn.angle_mdeg = 999999;
    command.value.turn.wheel_speed_mm_s = -1000;
    (void)motion_pack_command(&command, &packed);
    (void)motion_unpack_command(&packed, &result);
    check(result.type == MOTION_COMMAND_TURN_ANGLE
          && result.value.turn.angle_mdeg == 524287
          && result.value.turn.wheel_speed_mm_s == -512,
          "turn values clamp to their fields");

    command.type = (motion_command_type_t)3;
    check(!motion_pack_command(&command, &packed),
          "unknown command type is rejected on pack");
    packed.word[0] = 3U;
    packed.word[1] = 0U;
    check(!motion_unpack_command(&packed, &result),
          "unknown command type is rejected on unpack");

    command_timeout_check(0U, 0U);
    command_timeout_check(1U, 10U);
    command_timeout_check(10U, 10U);
    command_timeout_check(11U, 20U);
    command_timeout_check(20001U, 20010U);
    command_timeout_check(163830U, 163830U);
    command_timeout_check(163831U, 163830U);
}

/* The VC producer wraps sequence ids at 16 bits and skips 0; Motion echoes
   the packed value.  Run 65537 ids through a command round trip. */
static void sequence_wrap_checks(void)
{
    motion_command_t command = {0};
    motion_command_packed_t packed;
    motion_command_t result;
    uint32_t seq = 0U;
    uint32_t i;
    bool ok = true;
    bool wrapped_to_one = false;

    command.type = MOTION_COMMAND_SET_VELOCITY;
    for (i = 0U; i < 65537U; i++) {
        seq = (seq + 1U) & 0xFFFFU;
        if (seq == 0U) {
            seq = 1U;
        }
        if ((i == 65535U) && (seq == 1U)) {
            wrapped_to_one = true;
        }
        command.sequence_id = seq;
        if (!motion_pack_command(&command, &packed)
            || !motion_unpack_command(&packed, &result)
            || result.sequence_id != seq
            || result.sequence_id == 0U) {
            ok = false;
            break;
        }
    }
    check(ok, "65537 sequence ids round-trip and none is zero");
    check(wrapped_to_one, "the id after 65535 wraps to 1");

    command.sequence_id = 65535U;
    (void)motion_pack_command(&command, &packed);
    (void)motion_unpack_command(&packed, &result);
    check(result.sequence_id == 65535U, "id 65535 round-trips");

    command.sequence_id = 1U;
    (void)motion_pack_command(&command, &packed);
    (void)motion_unpack_command(&packed, &result);
    check(result.sequence_id == 1U, "id 1 round-trips after the wrap");
}

/* Boot-relative encoder counters are never reset, so they are stored full
   width; a 24 s field clamps at +-599 m of travel. */
static void status_encoder_range_checks(void)
{
    static int32_t const values[] = {
        INT32_C(8388608) + INT32_C(100),
        -(INT32_C(8388608) + INT32_C(100)),
        INT32_MAX,
        INT32_MIN
    };
    motion_status_t status = {0};
    motion_status_packed_t packed;
    motion_status_t result;
    uint32_t i;

    status.state = MOTION_STATE_IDLE;
    status.valid = true;

    for (i = 0U; i < (uint32_t)(sizeof values / sizeof values[0]); i++) {
        status.left_encoder_counts = values[i];
        status.right_encoder_counts = values[i];
        check(motion_pack_status(&status, &packed),
              "encoder range status packs");
        check(motion_unpack_status(&packed, &result),
              "encoder range status unpacks");
        check(result.left_encoder_counts == values[i]
              && result.right_encoder_counts == values[i],
              "encoder counts round-trip full width");
    }
}

static void status_checks(void)
{
    motion_status_t status = {0};
    motion_status_packed_t packed;
    motion_status_t result;

    check(!motion_pack_status(&status, &packed), "zero status packs invalid");
    check(!motion_unpack_status(&packed, &result), "zero status unpacks invalid");
    check(result.active_sequence_id == 0U
          && result.state == MOTION_STATE_UNINITIALISED
          && result.fault == MOTION_FAULT_NONE
          && result.encoder_fault == MOTION_ENCODER_FAULT_NONE,
          "zero status has zero fields");

    status.active_sequence_id = 65535U;
    status.state = MOTION_STATE_FAULT;
    status.fault = MOTION_FAULT_HARDWARE_CONFIG;
    status.encoder_fault = MOTION_ENCODER_FAULT_INVALID_TRANSITIONS;
    status.encoder_fault_wheel_mask = 3U;
    status.valid = true;
    status.left_speed_mm_s = 2047;
    status.right_speed_mm_s = -2048;
    status.left_target_mm_s = 300;
    status.right_target_mm_s = -300;
    status.left_slewed_target_mm_s = 100;
    status.right_slewed_target_mm_s = -100;
    status.left_drive_permille = 1023;
    status.right_drive_permille = -1024;
    status.left_pwm_permille = 1023U;
    status.right_pwm_permille = 1000U;
    status.target_speed_mm_s = -300;
    status.travelled_mm = 8388607;
    status.target_mm = -8388608;
    status.target_angle_mdeg = 524287;
    status.left_encoder_counts = -8388608;
    status.right_encoder_counts = 8388607;
    status.encoder_fault_elapsed_ms = 65535U;
    status.encoder_fault_left_delta = 32767;
    status.encoder_fault_right_delta = -32768;
    status.encoder_snapshot_duration_us = 65535U;
    status.encoder_fault_previous_left = -8388608;
    status.encoder_fault_previous_right = 8388607;
    status.encoder_fault_previous_irq = 0xFFFFFFU;
    status.encoder_fault_previous_time_us = 0x123456789ABCDEF0ULL;
    status.encoder_sample_time_us = 0xFEDCBA9876543210ULL;
    status.timestamp_us = 0x123456789ABCDEF0ULL;
    status.state_timestamp_us = 0xFEDCBA9876543210ULL;
    status.encoder_fault_previous_left_pwm = 1023U;
    status.encoder_fault_previous_right_pwm = 1000U;
    status.left_invalid_encoder_transitions = 0xFFFFU;
    status.right_invalid_encoder_transitions = 0x8000U;
    status.invalid_encoder_transitions = 0x1234U;
    status.encoder_irq_count = 0xFFFFFFU;
    check(motion_pack_status(&status, &packed), "maximum status packs valid");
    check(motion_unpack_status(&packed, &result), "maximum status unpacks valid");
    check(result.valid && result.active_sequence_id == 65535U
          && result.state == MOTION_STATE_FAULT
          && result.fault == MOTION_FAULT_HARDWARE_CONFIG
          && result.encoder_fault == MOTION_ENCODER_FAULT_INVALID_TRANSITIONS
          && result.encoder_fault_wheel_mask == 3U,
          "status flags round-trip");
    check(result.left_speed_mm_s == 2047 && result.right_speed_mm_s == -2048
          && result.left_target_mm_s == 300
          && result.right_target_mm_s == -300
          && result.left_slewed_target_mm_s == 100
          && result.right_slewed_target_mm_s == -100,
          "wheel speeds and targets round-trip");
    check(result.left_drive_permille == 1023
          && result.right_drive_permille == -1024
          && result.left_pwm_permille == 1023U
          && result.right_pwm_permille == 1000U
          && result.target_speed_mm_s == -300,
          "drive, PWM and target speed round-trip");
    check(result.travelled_mm == 8388607
          && result.target_mm == -8388608
          && result.target_angle_mdeg == 524287
          && result.left_encoder_counts == -8388608
          && result.right_encoder_counts == 8388607,
          "travel, targets and encoder counts round-trip");
    check(result.encoder_fault_elapsed_ms == 65535U
          && result.encoder_fault_left_delta == 32767
          && result.encoder_fault_right_delta == -32768
          && result.encoder_snapshot_duration_us == 65535U,
          "fault interval and deltas round-trip");
    check(result.encoder_fault_previous_left == -8388608
          && result.encoder_fault_previous_right == 8388607
          && result.encoder_fault_previous_irq == 0xFFFFFFU,
          "previous encoder evidence round-trips");
    check(result.encoder_fault_previous_time_us == 0x9ABCDEF0ULL
          && result.encoder_sample_time_us == 0x76543210ULL,
          "narrow timestamps keep their low 32 bits");
    check(result.timestamp_us == 0x123456789ABCDEF0ULL
          && result.state_timestamp_us == 0xFEDCBA9876543210ULL,
          "status timestamps keep all 64 bits");
    check(result.encoder_fault_previous_left_pwm == 1023U
          && result.encoder_fault_previous_right_pwm == 1000U
          && result.left_invalid_encoder_transitions == 0xFFFFU
          && result.right_invalid_encoder_transitions == 0x8000U
          && result.invalid_encoder_transitions == 0x1234U
          && result.encoder_irq_count == 0xFFFFFFU,
          "PWM and diagnostic counters round-trip");

    status.left_speed_mm_s = 10000;
    status.right_speed_mm_s = -10000;
    status.left_drive_permille = 5000;
    status.right_drive_permille = -5000;
    status.left_pwm_permille = 5000U;
    status.travelled_mm = 1000000000;
    status.left_encoder_counts = -1000000000;
    status.encoder_fault_elapsed_ms = 100000U;
    status.encoder_fault_left_delta = 100000;
    status.left_invalid_encoder_transitions = 100000U;
    (void)motion_pack_status(&status, &packed);
    (void)motion_unpack_status(&packed, &result);
    check(result.left_speed_mm_s == 2047 && result.right_speed_mm_s == -2048
          && result.left_drive_permille == 1023
          && result.right_drive_permille == -1024
          && result.left_pwm_permille == 1023U,
          "status measured values clamp to their fields");
    check(result.travelled_mm == 8388607
          && result.left_encoder_counts == -1000000000
          && result.encoder_fault_elapsed_ms == 65535U
          && result.encoder_fault_left_delta == 32767
          && result.left_invalid_encoder_transitions == 34464U,
          "status travel clamps, encoder counts keep full width, "
          "diagnostic counters wrap");

    status.state = (motion_state_t)7;
    status.valid = true;
    check(!motion_pack_status(&status, &packed),
          "unknown status enum is rejected on pack");
    (void)motion_unpack_status(&packed, &result);
    check(!result.valid && result.state == MOTION_STATE_UNINITIALISED,
          "unknown status enum is rejected on unpack");
}

int main(void)
{
    command_checks();
    sequence_wrap_checks();
    status_checks();
    status_encoder_range_checks();
    return 0;
}
