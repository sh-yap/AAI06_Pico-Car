/** @file test_barcode.c
 *
 * @brief Host tests for app_program/barcode.c (Buddy 3).
 *
 * Run with:  make -C tests/buddy3 test
 *
 * Simulates the barcode sensor being sampled every 10 ms while the
 * car drives over a Code 39 barcode with the course geometry
 * (narrow 3 mm, wide 9 mm, *X* = 141 mm). Encoder resolution uses
 * the Buddy 2 bench calibration (~2480 counts/rev, 65 mm wheel).
 *
 * Correctness checks must pass. The speed sweep only reports how
 * often 100 Hz sampling decodes correctly at each speed.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "barcode.h"

#define NARROW_UM          3000     /* 3 mm, course spec photo */
#define WIDE_UM            9000     /* 9 mm  */
#define QUIET_UM           30000    /* white before and after */
#define SAMPLE_MS          10U      /* Buddy 3 task period */
#define COUNTS_PER_M       12150    /* 2480 / (pi * 65 mm) per mm, x1000 */
#define PHASES             50U      /* start offsets within one sample */

#define MAX_ELEMENTS       29U

static char last_char = 0;    /* raw character of the last drive() */

/* Task wake-up jitter, ms either side (0 for exact sampling). */
static int32_t jitter_ms = 0;
static uint32_t lcg = 1U;

static uint32_t failures = 0U;
static uint32_t checks = 0U;

/* Code 39 patterns, first element = bit 8, 1 = wide. */
static uint16_t const PAT_STAR = 0x094U;
static uint16_t const PAT_A    = 0x109U;
static uint16_t const PAT_B    = 0x049U;
static uint16_t const PAT_C    = 0x148U;
static uint16_t const PAT_D    = 0x019U;
static uint16_t const PAT_E    = 0x118U;   /* valid Code 39, not used */
static uint16_t const PAT_F    = 0x058U;   /* course code */
static uint16_t const PAT_Z    = 0x0D0U;   /* sample *Z* from Barcode Sample.pdf */

/* Element widths used by build(); half size for the small PDF print. */
static int32_t narrow_um = NARROW_UM;
static int32_t wide_um = WIDE_UM;

/* Barcode as a list of element widths in um, starting with a bar. */
static int32_t elements[MAX_ELEMENTS];
static int32_t barcode_len_um;


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


static void
build(uint16_t data_pattern, bool reversed)
{
    uint16_t const chars[3] = { PAT_STAR, data_pattern, PAT_STAR };
    int32_t forward[MAX_ELEMENTS];
    uint32_t n = 0U;
    uint32_t c, e;

    for (c = 0U; c < 3U; c++)
    {
        for (e = 0U; e < 9U; e++)
        {
            forward[n++] = ((chars[c] >> (8U - e)) & 1U) ? wide_um : narrow_um;
        }

        if (c < 2U)
        {
            forward[n++] = narrow_um;   /* gap between characters */
        }
    }

    barcode_len_um = 0;

    for (e = 0U; e < MAX_ELEMENTS; e++)
    {
        elements[e] = reversed ? forward[MAX_ELEMENTS - 1U - e] : forward[e];
        barcode_len_um += elements[e];
    }
}


/* True if position (um from barcode start) is on a black bar. */
static bool
is_black_at(int32_t pos_um)
{
    int32_t edge = 0;
    uint32_t e;

    if ((pos_um < 0) || (pos_um >= barcode_len_um))
    {
        return false;
    }

    for (e = 0U; e < MAX_ELEMENTS; e++)
    {
        edge += elements[e];

        if (pos_um < edge)
        {
            return (e % 2U) == 0U;   /* even elements are bars */
        }
    }

    return false;
}


/*
 * Drive over the barcode at speed_mm_s, starting phase_um earlier
 * than the first sample. Returns the code reported on completion.
 */
static barcode_t
drive(int32_t speed_mm_s, int32_t phase_um, int32_t direction)
{
    uint32_t t_ms = 0U;
    int32_t pos_um;
    int32_t end_um = barcode_len_um + QUIET_UM;
    int64_t counts;
    barcode_t code = BARCODE_NONE;

    barcode_init();

    for (;;)
    {
        lcg = (lcg * 1103515245U) + 12345U;
        pos_um = -QUIET_UM + phase_um + (speed_mm_s * (int32_t)t_ms);

        if (jitter_ms > 0)
        {
            /* Sample taken up to jitter_ms early or late (um = mm/s * ms). */
            pos_um += speed_mm_s *
                      ((int32_t)((lcg >> 16) % (uint32_t)((2 * jitter_ms) + 1))
                       - jitter_ms);
        }

        if (pos_um > end_um)
        {
            break;
        }

        counts = ((int64_t)(pos_um + QUIET_UM) * COUNTS_PER_M) / 1000000;

        barcode_update(is_black_at(pos_um), t_ms,
                       (int32_t)(direction * counts));
        barcode_check_timeout(t_ms);

        if (barcode_available())
        {
            code = barcode_get_code();
            last_char = barcode_get_char();
            (void)barcode_get_command();

            if (code != BARCODE_NONE)
            {
                return code;
            }
        }

        t_ms += SAMPLE_MS;
    }

    return code;
}


static void
test_decodes_each_code(void)
{
    static struct { uint16_t pat; barcode_t code; nav_command_t nav; } const cases[] = {
        { PAT_A, BARCODE_A, NAV_LEFT },
        { PAT_B, BARCODE_B, NAV_RIGHT },
        { PAT_C, BARCODE_C, NAV_STRAIGHT },
        { PAT_D, BARCODE_D, NAV_UTURN },
        { PAT_F, BARCODE_F, NAV_RIGHT },
        { PAT_Z, BARCODE_Z, NAV_UTURN },
    };
    uint32_t i;
    uint32_t p;
    bool forward_ok;
    bool reversed_ok;
    bool backwards_ok;

    for (i = 0U; i < (sizeof cases / sizeof cases[0]); i++)
    {
        forward_ok = true;
        reversed_ok = true;
        backwards_ok = true;

        for (p = 0U; p < PHASES; p++)
        {
            int32_t phase = (int32_t)(p * (50U * SAMPLE_MS) / PHASES) * 1;

            build(cases[i].pat, false);
            forward_ok &= (drive(50, phase, 1) == cases[i].code);
            backwards_ok &= (drive(50, phase, -1) == cases[i].code);

            build(cases[i].pat, true);
            reversed_ok &= (drive(50, phase, 1) == cases[i].code);
        }

        check("decodes at 50 mm/s", forward_ok);
        check("decodes when read in the opposite direction", reversed_ok);
        check("decodes with decreasing encoder counts", backwards_ok);
    }

    /* Nav mapping. */
    for (i = 0U; i < (sizeof cases / sizeof cases[0]); i++)
    {
        nav_command_t nav = NAV_NONE;
        uint32_t t;

        build(cases[i].pat, false);
        barcode_init();

        for (t = 0U; t < 10000U; t += SAMPLE_MS)
        {
            int32_t pos = -QUIET_UM + (50 * (int32_t)t);
            barcode_update(is_black_at(pos), t,
                           (int32_t)(((int64_t)(pos + QUIET_UM) * COUNTS_PER_M) / 1000000));
            barcode_check_timeout(t);

            if (barcode_available())
            {
                nav = barcode_get_command();
                break;
            }
        }

        check("barcode maps to nav command", nav == cases[i].nav);
    }
}


static void
test_rejects(void)
{
    /* Valid Code 39 character that is not A-D. */
    build(PAT_E, false);
    check("unused character E is INVALID", drive(50, 0, 1) == BARCODE_INVALID);

    /* Sample *Z*: a course code, U-turn for testing. */
    build(PAT_Z, false);
    check("sample *Z* is BARCODE_Z", drive(50, 0, 1) == BARCODE_Z);
    check("sample *Z* raw character is 'Z'", last_char == 'Z');

    build(PAT_A, false);
    (void)drive(50, 0, 1);
    check("sample *A* raw character is 'A'", last_char == 'A');

    /* No start/stop: data pattern used for all three characters. */
    {
        uint32_t e;
        build(PAT_A, false);
        for (e = 0U; e < 9U; e++)
        {
            elements[e] = elements[10U + e];
            elements[20U + e] = elements[10U + e];
        }
        check("missing '*' start/stop is INVALID",
              drive(50, 0, 1) == BARCODE_INVALID);
    }

    /* All elements the same width: wide cannot be told from narrow. */
    {
        uint32_t e;
        build(PAT_A, false);
        for (e = 0U; e < MAX_ELEMENTS; e++)
        {
            elements[e] = NARROW_UM;
        }
        barcode_len_um = (int32_t)MAX_ELEMENTS * NARROW_UM;
        check("equal widths are INVALID", drive(50, 0, 1) == BARCODE_INVALID);
    }

    /* Too few edges, then timeout. */
    barcode_init();
    barcode_update(false, 0U, 0);
    barcode_update(true, 10U, 100);
    barcode_update(false, 20U, 200);
    barcode_update(true, 30U, 300);
    barcode_check_timeout(300U);
    check("incomplete barcode is INVALID",
          barcode_available() && (barcode_get_code() == BARCODE_INVALID));
    check("INVALID gives NAV_NONE", barcode_get_command() == NAV_NONE);
    check("reset after command", !barcode_available());
}


/* Report decode rate vs speed at 100 Hz (information, not pass/fail). */
static void
report_speed_sweep(int32_t narrow, char const * p_label)
{
    static int32_t const speeds[] = { 50, 100, 150, 200, 250, 300 };
    uint32_t s;
    uint32_t p;
    uint32_t ok;

    (void)printf("100 Hz decode rate, %u start offsets, +/-%d ms jitter, *A*, %s:\n",
                 (unsigned int)PHASES, (int)jitter_ms, p_label);

    narrow_um = narrow;
    wide_um = 3 * narrow;
    build(PAT_A, false);

    for (s = 0U; s < (sizeof speeds / sizeof speeds[0]); s++)
    {
        ok = 0U;

        for (p = 0U; p < PHASES; p++)
        {
            int32_t phase = (int32_t)(((int64_t)p * speeds[s] * (int32_t)SAMPLE_MS)
                                      / (int32_t)PHASES);

            if (drive(speeds[s], phase, 1) == BARCODE_A)
            {
                ok++;
            }
        }

        (void)printf("  %3d mm/s (%d.%d mm per sample): %3u%%\n",
                     (int)speeds[s],
                     (int)(speeds[s] / 100), (int)((speeds[s] / 10) % 10),
                     (unsigned int)((ok * 100U) / PHASES));
    }

    narrow_um = NARROW_UM;
    wide_um = WIDE_UM;
}

int
main(void)
{
    test_decodes_each_code();
    test_rejects();
    jitter_ms = 2;
    report_speed_sweep(3000, "narrow 3 mm (course spec)");
    report_speed_sweep(1500, "narrow 1.5 mm (small PDF print)");
    jitter_ms = 0;

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return (failures == 0U) ? 0 : 1;
}
