/** @file test_buddy3_status.c
 *
 * @brief Host unit tests for app_program/buddy3_status.c (Buddy 3).
 *
 * Run with:  make -C tests/buddy3 test
 *
 * Covers risk R-B3-01: every valid value survives pack/unpack,
 * boundary values keep their sign, and reserved codes are rejected.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "buddy3_status.h"

static uint32_t failures = 0U;
static uint32_t checks = 0U;


static void
check(char const * p_name, bool ok)
{
    checks++;

    if (!ok)
    {
        failures++;
        (void)printf("FAIL %s\n", p_name);
    }
}


static bool
round_trip(buddy3_status_t const * p_in)
{
    buddy3_status_t out = {0};

    if (!buddy3_status_unpack(buddy3_status_pack(p_in), &out))
    {
        return false;
    }

    return (out.line_state == p_in->line_state) &&
           (out.line_error == p_in->line_error) &&
           (out.line_sensors == p_in->line_sensors) &&
           (out.barcode == p_in->barcode) &&
           (out.nav_command == p_in->nav_command) &&
           (out.cal_state == p_in->cal_state);
}


/* Every combination of enum values, sensors and error boundaries. */
static void
test_all_valid_values(void)
{
    static int8_t const errors[] = { -127, -1, 0, 1, 64, 127 };
    buddy3_status_t s;
    uint32_t ls, e, sn, bc, nav, cal;
    bool all_ok = true;

    for (ls = 0U; ls <= (uint32_t)LINE_JUNCTION; ls++)
    for (e = 0U; e < (sizeof errors / sizeof errors[0]); e++)
    for (sn = 0U; sn <= 3U; sn++)
    for (bc = 0U; bc <= (uint32_t)BARCODE_INVALID; bc++)
    for (nav = 0U; nav <= (uint32_t)NAV_UTURN; nav++)
    for (cal = 0U; cal <= (uint32_t)LINE_CAL_FAIL_GAP; cal++)
    {
        s.line_state   = (line_state_t)ls;
        s.line_error   = errors[e];
        s.line_sensors = (uint8_t)sn;
        s.barcode      = (barcode_t)bc;
        s.nav_command  = (nav_command_t)nav;
        s.cal_state    = (line_cal_state_t)cal;

        if (!round_trip(&s))
        {
            all_ok = false;
            (void)printf("  round trip failed: ls=%u err=%d sn=%u bc=%u"
                         " nav=%u cal=%u\n",
                         (unsigned)ls, (int)errors[e], (unsigned)sn,
                         (unsigned)bc, (unsigned)nav, (unsigned)cal);
        }
    }

    check("all valid values round-trip", all_ok);
}


/* Packed word uses only bits 0-21. */
static void
test_width(void)
{
    buddy3_status_t s = { LINE_JUNCTION, -127, 3U, BARCODE_INVALID, NAV_UTURN,
                          LINE_CAL_FAIL_GAP };

    check("no bits above bit 21",
          (buddy3_status_pack(&s) & B3_RESERVED_MASK) == 0U);
}


/* Reserved codes and reserved bits must be rejected. */
static void
test_reserved_rejected(void)
{
    buddy3_status_t out = {0};
    uint32_t code;

    for (code = (uint32_t)LINE_JUNCTION + 1U; code <= B3_LINE_STATE_MASK; code++)
    {
        check("reserved line state rejected",
              !buddy3_status_unpack(code << B3_LINE_STATE_SHIFT, &out));
    }

    for (code = (uint32_t)BARCODE_INVALID + 1U; code <= B3_BARCODE_MASK; code++)
    {
        check("reserved barcode rejected",
              !buddy3_status_unpack(code << B3_BARCODE_SHIFT, &out));
    }

    for (code = (uint32_t)NAV_UTURN + 1U; code <= B3_NAV_MASK; code++)
    {
        check("reserved nav command rejected",
              !buddy3_status_unpack(code << B3_NAV_SHIFT, &out));
    }

    check("line error -128 rejected",
          !buddy3_status_unpack(0x80UL << B3_LINE_ERROR_SHIFT, &out));

    for (code = (uint32_t)LINE_CAL_FAIL_GAP + 1U; code <= B3_CAL_MASK; code++)
    {
        check("reserved calibration state rejected",
              !buddy3_status_unpack(code << B3_CAL_SHIFT, &out));
    }

    check("reserved bit 22 rejected",
          !buddy3_status_unpack(1UL << 22, &out));

    check("reserved bit 31 rejected",
          !buddy3_status_unpack(1UL << 31, &out));
}


/* A rejected word must not overwrite the caller's last good status. */
static void
test_reject_keeps_output(void)
{
    buddy3_status_t out = { LINE_RIGHT, 42, 2U, BARCODE_F, NAV_RIGHT,
                            LINE_CAL_OK };

    (void)buddy3_status_unpack(7UL << B3_NAV_SHIFT, &out);

    check("rejected unpack leaves output unchanged",
          (out.line_state == LINE_RIGHT) && (out.line_error == 42) &&
          (out.line_sensors == 2U) && (out.barcode == BARCODE_F) &&
          (out.nav_command == NAV_RIGHT) && (out.cal_state == LINE_CAL_OK));
}


int
main(void)
{
    test_all_valid_values();
    test_width();
    test_reserved_rejected();
    test_reject_keeps_output();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return (failures == 0U) ? 0 : 1;
}
