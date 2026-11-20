/* Round-trip, clamp and reject checks for the packed Buddy 4 status record
 * and the int16_t incline-window store (docs/DATA_REPRESENTATION.md 4.1). */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "b4_types.h"
#include "b4_pack.h"

static int g_checks = 0;
static int g_failed = 0;

static void
check (bool ok, const char * name)
{
    g_checks++;
    if (!ok)
    {
        g_failed++;
    }
    printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
}

static void
status_zero (b4_types_status_t * s)
{
    memset(s, 0, sizeof(*s));
}

/* Compare every field, so a missing member in pack/unpack is caught. */
static bool
status_same (const b4_types_status_t * a, const b4_types_status_t * b)
{
    return (a->health == b->health) &&
           (a->fault == b->fault) &&
           (a->tilt == b->tilt) &&
           (a->motion == b->motion) &&
           (a->hump_state == b->hump_state) &&
           (a->turn == b->turn) &&
           (a->sample.timestamp_ms == b->sample.timestamp_ms) &&
           (a->sample.accel_x_mg == b->sample.accel_x_mg) &&
           (a->sample.accel_y_mg == b->sample.accel_y_mg) &&
           (a->sample.accel_z_mg == b->sample.accel_z_mg) &&
           (a->sample.b_valid == b->sample.b_valid) &&
           (a->mag_sample.timestamp_ms == b->mag_sample.timestamp_ms) &&
           (a->mag_sample.mag_x == b->mag_sample.mag_x) &&
           (a->mag_sample.mag_y == b->mag_sample.mag_y) &&
           (a->mag_sample.mag_z == b->mag_sample.mag_z) &&
           (a->mag_sample.b_valid == b->mag_sample.b_valid) &&
           (a->filtered_x_mg == b->filtered_x_mg) &&
           (a->filtered_y_mg == b->filtered_y_mg) &&
           (a->filtered_z_mg == b->filtered_z_mg) &&
           (a->vertical_deviation_mg == b->vertical_deviation_mg) &&
           (a->roll_mg == b->roll_mg) &&
           (a->roll_baseline_mg == b->roll_baseline_mg) &&
           (a->turn_cross == b->turn_cross) &&
           (a->incline_delta_mg == b->incline_delta_mg) &&
           (a->drive_speed_mm_s == b->drive_speed_mm_s) &&
           (a->noise_mg == b->noise_mg) &&
           (a->learn_count == b->learn_count) &&
           (a->detectable_height_um == b->detectable_height_um) &&
           (a->current_hump_peak_mg == b->current_hump_peak_mg) &&
           (a->highest_hump_peak_mg == b->highest_hump_peak_mg) &&
           (a->hump_peak_height_um == b->hump_peak_height_um) &&
           (a->highest_hump_height_um == b->highest_hump_height_um) &&
           (a->active_hump_height_um == b->active_hump_height_um) &&
           (a->b_hump_height_valid == b->b_hump_height_valid) &&
           (a->b_mag_valid == b->b_mag_valid) &&
           (a->b_sample_stale == b->b_sample_stale) &&
           (a->b_sharp_change_event == b->b_sharp_change_event) &&
           (a->b_lift_cue == b->b_lift_cue) &&
           (a->b_hump_started_event == b->b_hump_started_event) &&
           (a->b_hump_completed_event == b->b_hump_completed_event) &&
           (a->b_jolt_window_active == b->b_jolt_window_active);
}

static void
status_all_flags (b4_types_status_t * s, bool b_set)
{
    s->b_hump_height_valid = b_set;
    s->b_mag_valid = b_set;
    s->b_sample_stale = b_set;
    s->b_sharp_change_event = b_set;
    s->b_lift_cue = b_set;
    s->b_hump_started_event = b_set;
    s->b_hump_completed_event = b_set;
    s->b_jolt_window_active = b_set;
}

static void
checks_record_size (void)
{
    check(B4_STATUS_PACKED_WORDS == 17U, "status record is 17 words");
    check(sizeof(b4_status_packed_t) == 68U, "status record is 68 bytes");
}

static void
checks_zero (void)
{
    b4_types_status_t status;
    b4_types_status_t result;
    b4_status_packed_t packed;
    unsigned index;
    unsigned zero_words = 0U;

    status_zero(&status);
    (void)memset(&packed, 0xFF, sizeof(packed));
    b4_pack_status(&status, &packed);
    for (index = 0U; index < B4_STATUS_PACKED_WORDS; index++)
    {
        if (0U == packed.word[index])
        {
            zero_words++;
        }
    }
    check(zero_words == B4_STATUS_PACKED_WORDS, "zero status packs zero");
    check(b4_unpack_status(&packed, &result), "zero status unpacks");
    check(status_same(&status, &result), "zero status round-trips");
}

static void
status_fill_max (b4_types_status_t * s)
{
    status_zero(s);
    s->health = B4_HEALTH_FAULT;
    s->fault = B4_FAULT_MAGNETOMETER;
    s->tilt = B4_TILT_COMPOUND;
    s->motion = B4_MOTION_LIFTED;
    s->hump_state = B4_HUMP_COMPLETE;
    s->turn = B4_TURN_RIGHT;
    s->sample.timestamp_ms = 0xFFFFFFFFU;
    s->sample.accel_x_mg = 4095;
    s->sample.accel_y_mg = 4095;
    s->sample.accel_z_mg = 4095;
    s->sample.b_valid = true;
    s->mag_sample.timestamp_ms = 0xFFFFFFFFU;
    s->mag_sample.mag_x = 4095;
    s->mag_sample.mag_y = 4095;
    s->mag_sample.mag_z = 4095;
    s->mag_sample.b_valid = true;
    s->filtered_x_mg = 4095;
    s->filtered_y_mg = 4095;
    s->filtered_z_mg = 4095;
    s->vertical_deviation_mg = 8191;
    s->roll_mg = 4095;
    s->roll_baseline_mg = 4095;
    s->turn_cross = 16777215;
    s->incline_delta_mg = 8191;
    s->drive_speed_mm_s = 2047;
    s->noise_mg = 511U;
    s->learn_count = 0xFFFFFFU;
    s->detectable_height_um = 0xFFFFFU;
    s->current_hump_peak_mg = 8191U;
    s->highest_hump_peak_mg = 8191U;
    s->hump_peak_height_um = 131071U;
    s->highest_hump_height_um = 0xFFFFFFFFU;
    s->active_hump_height_um = 131071U;
    status_all_flags(s, true);
}

static void
checks_max (void)
{
    b4_types_status_t status;
    b4_types_status_t result;
    b4_status_packed_t packed;

    status_fill_max(&status);
    b4_pack_status(&status, &packed);
    check(b4_unpack_status(&packed, &result), "maximum status unpacks");
    check(status_same(&status, &result), "maximum status round-trips");
}

static void
checks_min (void)
{
    b4_types_status_t status;
    b4_types_status_t result;
    b4_status_packed_t packed;

    status_zero(&status);
    status.sample.accel_x_mg = -4096;
    status.sample.accel_y_mg = -4096;
    status.sample.accel_z_mg = -4096;
    status.mag_sample.mag_x = -4096;
    status.mag_sample.mag_y = -4096;
    status.mag_sample.mag_z = -4096;
    status.filtered_x_mg = -4096;
    status.filtered_y_mg = -4096;
    status.filtered_z_mg = -4096;
    status.vertical_deviation_mg = -8192;
    status.roll_mg = -4096;
    status.roll_baseline_mg = -4096;
    status.incline_delta_mg = -8192;
    status.turn_cross = -16777216;
    b4_pack_status(&status, &packed);
    check(b4_unpack_status(&packed, &result), "minimum status unpacks");
    check(status_same(&status, &result), "minimum status round-trips");
}

static void
checks_clamp (void)
{
    b4_types_status_t status;
    b4_types_status_t result;
    b4_types_status_t expect;
    b4_status_packed_t packed;

    status_zero(&status);
    status.sample.accel_x_mg = 5000;
    status.sample.accel_y_mg = -5000;
    status.sample.accel_z_mg = 100000;
    status.mag_sample.mag_x = 20000;
    status.mag_sample.mag_y = -20000;
    status.mag_sample.mag_z = 32767;
    status.filtered_x_mg = INT32_MAX;
    status.filtered_y_mg = INT32_MIN;
    status.filtered_z_mg = 5000;
    status.vertical_deviation_mg = INT32_MIN;
    status.roll_mg = INT32_MAX;
    status.roll_baseline_mg = 20000;
    status.incline_delta_mg = 100000;
    status.turn_cross = INT32_MAX;
    status.drive_speed_mm_s = 5000;
    status.noise_mg = 5000U;
    status.learn_count = 0xFFFFFFFFU;
    status.detectable_height_um = 0xFFFFFFFFU;
    status.current_hump_peak_mg = 20000U;
    status.highest_hump_peak_mg = 0xFFFFFFFFU;
    status.hump_peak_height_um = 1000000U;
    status.highest_hump_height_um = 0xFFFFFFFFU;
    status.active_hump_height_um = 131072U;

    status_zero(&expect);
    expect.sample.accel_x_mg = 4095;
    expect.sample.accel_y_mg = -4096;
    expect.sample.accel_z_mg = 4095;
    expect.mag_sample.mag_x = 4095;
    expect.mag_sample.mag_y = -4096;
    expect.mag_sample.mag_z = 4095;
    expect.filtered_x_mg = 4095;
    expect.filtered_y_mg = -4096;
    expect.filtered_z_mg = 4095;
    expect.vertical_deviation_mg = -8192;
    expect.roll_mg = 4095;
    expect.roll_baseline_mg = 4095;
    expect.incline_delta_mg = 8191;
    expect.turn_cross = 16777215;
    expect.drive_speed_mm_s = 2047;
    expect.noise_mg = 511U;
    expect.learn_count = 0xFFFFFFU;
    expect.detectable_height_um = 0xFFFFFU;
    expect.current_hump_peak_mg = 8191U;
    expect.highest_hump_peak_mg = 8191U;
    expect.hump_peak_height_um = 131071U;
    expect.highest_hump_height_um = 0xFFFFFFFFU;
    expect.active_hump_height_um = 131071U;

    b4_pack_status(&status, &packed);
    check(b4_unpack_status(&packed, &result), "clamped status unpacks");
    check(status_same(&expect, &result), "out-of-range values clamp");

    status_zero(&status);
    status.drive_speed_mm_s = -5;
    b4_pack_status(&status, &packed);
    check(b4_unpack_status(&packed, &result), "negative speed unpacks");
    check(0 == result.drive_speed_mm_s, "negative speed clamps to zero");
}

static void
checks_flags (void)
{
    static const unsigned shifts[8] = {17U, 18U, 19U, 20U, 21U, 22U, 23U,
                                       24U};
    b4_types_status_t status;
    b4_types_status_t result;
    b4_status_packed_t packed;
    unsigned index;
    bool b_ok = true;

    for (index = 0U; index < 8U; index++)
    {
        status_zero(&status);
        switch (index)
        {
            case 0U:
                status.b_hump_height_valid = true;
                break;
            case 1U:
                status.b_mag_valid = true;
                break;
            case 2U:
                status.b_sample_stale = true;
                break;
            case 3U:
                status.b_sharp_change_event = true;
                break;
            case 4U:
                status.b_lift_cue = true;
                break;
            case 5U:
                status.b_hump_started_event = true;
                break;
            case 6U:
                status.b_hump_completed_event = true;
                break;
            default:
                status.b_jolt_window_active = true;
                break;
        }
        b4_pack_status(&status, &packed);
        if (packed.word[0] != (1U << shifts[index]))
        {
            b_ok = false;
        }
        (void)b4_unpack_status(&packed, &result);
        if (!status_same(&status, &result))
        {
            b_ok = false;
        }
    }
    check(b_ok, "each status flag has its own bit and round-trips");
}

static void
checks_reserved (void)
{
    b4_types_status_t status;
    b4_status_packed_t packed;
    bool b_ok = true;

    status_fill_max(&status);
    b4_pack_status(&status, &packed);
    b_ok = b_ok && (0U == (packed.word[0] & 0xFE000000U));
    b_ok = b_ok && (0U == (packed.word[2] & 0xF8000000U));
    b_ok = b_ok && (0U == (packed.word[3] & 0xFFFFE000U));
    b_ok = b_ok && (0U == (packed.word[5] & 0xFC000000U));
    b_ok = b_ok && (0U == (packed.word[6] & 0xFFFFC000U));
    b_ok = b_ok && (0U == (packed.word[7] & 0xFC000000U));
    b_ok = b_ok && (0U == (packed.word[8] & 0xFFFFE000U));
    b_ok = b_ok && (0U == (packed.word[9] & 0xF8000000U));
    b_ok = b_ok && (0U == (packed.word[10] & 0xF8000000U));
    b_ok = b_ok && (0U == (packed.word[11] & 0xFE000000U));
    b_ok = b_ok && (0U == (packed.word[14] & 0xF0000000U));
    check(b_ok, "reserved bits stay zero");
}

static void
checks_reachable (void)
{
    b4_types_status_t status;
    b4_types_status_t result;
    b4_status_packed_t packed;

    /* The validity check accepts +-4000 mg samples, so every 13 s field
     * must carry the whole reachable band. */
    status_zero(&status);
    status.sample.accel_x_mg = 4000;
    status.sample.accel_y_mg = -4000;
    status.sample.accel_z_mg = 4000;
    status.filtered_x_mg = 4000;
    status.filtered_y_mg = -4000;
    status.filtered_z_mg = 4000;
    status.roll_mg = 4000;
    status.roll_baseline_mg = -4000;
    /* The deviation subtracts a 1..2000 mg gravity reference. */
    status.vertical_deviation_mg = 3999;
    b4_pack_status(&status, &packed);
    check(b4_unpack_status(&packed, &result),
          "reachable +-4000 mg values unpack");
    check(status_same(&status, &result),
          "reachable +-4000 mg values round-trip");

    status_zero(&status);
    status.vertical_deviation_mg = -6000;
    b4_pack_status(&status, &packed);
    (void)b4_unpack_status(&packed, &result);
    check(-6000 == result.vertical_deviation_mg,
          "reachable -6000 mg deviation round-trips");

    /* An amplitude is the difference of two +-4000 mg means. */
    status_zero(&status);
    status.incline_delta_mg = 8000;
    status.current_hump_peak_mg = 8000U;
    status.highest_hump_peak_mg = 8000U;
    b4_pack_status(&status, &packed);
    check(b4_unpack_status(&packed, &result),
          "reachable +8000 mg values unpack");
    check(status_same(&status, &result),
          "reachable +8000 mg values round-trip");

    status_zero(&status);
    status.incline_delta_mg = -8000;
    b4_pack_status(&status, &packed);
    (void)b4_unpack_status(&packed, &result);
    check(-8000 == result.incline_delta_mg,
          "reachable -8000 mg delta round-trips");

    /* A config-legal lever and offset reach 300000 * 8000 / 1 um. */
    status_zero(&status);
    status.highest_hump_height_um = 2400000000U;
    status.hump_peak_height_um = 131071U;
    status.active_hump_height_um = 131071U;
    b4_pack_status(&status, &packed);
    check(b4_unpack_status(&packed, &result), "reachable heights unpack");
    check(2400000000U == result.highest_hump_height_um,
          "run-maximum height keeps its full range");
    check((131071U == result.hump_peak_height_um) &&
          (131071U == result.active_hump_height_um),
          "17-bit heights round-trip");
}

static void
checks_unknown_enum (void)
{
    b4_types_status_t result;
    b4_status_packed_t packed;

    (void)memset(&packed, 0, sizeof(packed));
    /* fault 15, tilt 7, motion 6, hump_state 7 are not codes of their
     * enums; health and turn use every code their width can hold. */
    packed.word[0] = (15U << 2) | (7U << 6) | (6U << 9) | (7U << 12);
    check(!b4_unpack_status(&packed, &result),
          "unknown enum codes are rejected");
    check((B4_FAULT_NONE == result.fault) && (B4_TILT_UNKNOWN == result.tilt)
          && (B4_MOTION_QUIET == result.motion)
          && (B4_HUMP_NORMAL == result.hump_state)
          && (B4_HEALTH_NOT_INITIALIZED == result.health)
          && (B4_TURN_UNKNOWN == result.turn),
          "unknown enum codes read as no data");

    packed.word[0] = (15U << 2);
    (void)b4_unpack_status(&packed, &result);
    check(B4_FAULT_NONE == result.fault, "one unknown code rejects the rest");
}

static void
checks_mg_i16 (void)
{
    check(INT16_MAX == b4_pack_mg_i16(INT32_MAX), "window clamps high");
    check(INT16_MIN == b4_pack_mg_i16(INT32_MIN), "window clamps low");
    check(4000 == b4_pack_mg_i16(4000), "window keeps +4000 mg");
    check(-4000 == b4_pack_mg_i16(-4000), "window keeps -4000 mg");
    check(INT16_MAX == b4_pack_mg_i16((int32_t)INT16_MAX + 1),
          "window clamps just above the top");
    check(INT16_MIN == b4_pack_mg_i16((int32_t)INT16_MIN - 1),
          "window clamps just below the bottom");
    check(0 == b4_pack_mg_i16(0), "window keeps zero");
}

int
main (void)
{
    checks_record_size();
    checks_zero();
    checks_max();
    checks_min();
    checks_clamp();
    checks_reachable();
    checks_flags();
    checks_reserved();
    checks_unknown_enum();
    checks_mg_i16();

    printf("%d checks, %d failed\n", g_checks, g_failed);
    return ((0 == g_failed) ? 0 : 1);
}
