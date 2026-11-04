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
#define QUIET_UM           30000    /* white before the barcode */
#define TAIL_UM            100000   /* white driven after it: over the
                                       end gap of a 2x print (~36 mm) */
#define SAMPLE_MS          10U      /* Buddy 3 task period */
#define COUNTS_PER_M       12300    /* measured on the car 29 Oct: 3 mm ~ 37 counts */
#define END_GAP            320      /* BARCODE_END_GAP_MIN_COUNTS */
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
static uint16_t const PAT_B    = 0x049U;   /* valid Code 39, not used */
static uint16_t const PAT_E    = 0x118U;   /* valid Code 39, not used */
static uint16_t const PAT_F    = 0x058U;
static uint16_t const PAT_Z    = 0x0D0U;   /* sample *Z* from Barcode Sample.pdf */
static uint16_t const PAT_Y    = 0x190U;
static uint16_t const PAT_X    = 0x091U;

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
    int32_t end_um = barcode_len_um + TAIL_UM;
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
        barcode_check_end((int32_t)(direction * counts));

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
        { PAT_F, BARCODE_F, NAV_RIGHT },
        { PAT_Z, BARCODE_Z, NAV_STRAIGHT },
        { PAT_Y, BARCODE_Y, NAV_LEFT },
        { PAT_X, BARCODE_X, NAV_UTURN },
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
            int32_t enc = (int32_t)(((int64_t)(pos + QUIET_UM) * COUNTS_PER_M) / 1000000);
            barcode_update(is_black_at(pos), t, enc);
            barcode_check_end(enc);

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
    /* Valid Code 39 characters that are not course codes. */
    build(PAT_E, false);
    check("unused character E is INVALID", drive(50, 0, 1) == BARCODE_INVALID);
    build(PAT_B, false);
    check("unused character B is INVALID", drive(50, 0, 1) == BARCODE_INVALID);

    /* Sample *Z* from Barcode Sample.pdf. */
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

    /* A speck (too few edges) is dropped silently. */
    barcode_init();
    barcode_update(false, 0U, 0);
    barcode_update(true, 10U, 20);
    barcode_update(false, 20U, 40);
    barcode_check_end(40 + END_GAP);
    check("speck is not reported", !barcode_available());

    /* Part of a barcode, then the end gap. */
    {
        uint32_t e;
        barcode_init();
        barcode_update(false, 0U, 0);

        for (e = 1U; e <= 8U; e++)
        {
            barcode_update((e % 2U) == 1U, 10U * e, 37 * (int32_t)e);
        }
    }
    barcode_check_end(37 * 8);
    check("no end while still within the gap", !barcode_available());
    barcode_check_end((37 * 8) + END_GAP);
    check("incomplete barcode is INVALID",
          barcode_available() && (barcode_get_code() == BARCODE_INVALID));
    check("INVALID gives NAV_NONE", barcode_get_command() == NAV_NONE);
    check("reset after command", !barcode_available());
}


/*
 * Car stops for 2 s in the middle of the barcode (hand push, or the
 * car braking), with the sensor flickering on a bar boundary: the
 * capture must wait, not end early, and ignore the flicker.
 */
static void
test_stop_mid_barcode(void)
{
    uint32_t t;
    int32_t pos = -QUIET_UM;
    int32_t const stop_at_um = 70000;    /* inside the data character */
    uint32_t stopped_ms = 0U;
    barcode_t code = BARCODE_NONE;
    bool black;

    build(PAT_Z, false);
    barcode_init();

    for (t = 0U; t < 20000U; t += SAMPLE_MS)
    {
        int32_t enc = (int32_t)(((int64_t)(pos + QUIET_UM) * COUNTS_PER_M) / 1000000);

        black = is_black_at(pos);

        if ((stopped_ms > 0U) && (stopped_ms < 2000U) && ((t / SAMPLE_MS) % 2U == 0U))
        {
            black = !black;    /* flicker while stopped */
        }

        barcode_update(black, t, enc);
        barcode_check_end(enc);

        if (barcode_available())
        {
            code = barcode_get_code();
            (void)barcode_get_command();
            break;
        }

        if ((pos >= stop_at_um) && (stopped_ms < 2000U))
        {
            stopped_ms += SAMPLE_MS;
        }
        else
        {
            pos += 50 * (int32_t)SAMPLE_MS;
        }
    }

    check("2 s stop with flicker mid-barcode still decodes *Z*", code == BARCODE_Z);
}


/*
 * Dark specks shortly before the barcode (paper edge, tape) and flicker
 * at the start: seen on the car, they used to split the capture.
 */
static void
test_specks_before_barcode(void)
{
    uint32_t t;
    int32_t pos = -QUIET_UM;
    barcode_t code = BARCODE_NONE;
    bool black;

    build(PAT_F, false);
    barcode_init();

    /* Flicker in place before moving. */
    barcode_update(false, 0U, 0);
    barcode_update(true, 0U, 0);
    barcode_update(false, 0U, 1);
    barcode_update(true, 0U, 1);
    barcode_update(false, 0U, 1);

    for (t = 0U; t < 20000U; t += SAMPLE_MS)
    {
        int32_t enc = (int32_t)(((int64_t)(pos + QUIET_UM) * COUNTS_PER_M) / 1000000);

        /* Specks 12 and 7 mm before the first bar: within the end gap. */
        black = is_black_at(pos) ||
                ((pos >= -12000) && (pos < -10000)) ||
                ((pos >= -7000) && (pos < -6000));

        barcode_update(black, t, enc);
        barcode_check_end(enc);

        if (barcode_available())
        {
            code = barcode_get_code();
            (void)barcode_get_command();
            break;
        }

        pos += 50 * (int32_t)SAMPLE_MS;
    }

    check("specks + flicker before barcode still decodes *F*", code == BARCODE_F);
}


/*
 * Drive at 100 mm/s over two copies of the current barcode, the second
 * starting gap_mm after the first ends. Returns valid decodes seen.
 */
static uint32_t
count_valid_two_barcodes(int32_t gap_mm)
{
    int32_t second_um = barcode_len_um + (gap_mm * 1000);
    int32_t end_um = second_um + barcode_len_um + QUIET_UM;
    int32_t pos_um;
    int32_t enc;
    uint32_t t_ms;
    uint32_t valid = 0U;
    barcode_t code;

    barcode_init();

    for (t_ms = 0U; ; t_ms += SAMPLE_MS)
    {
        pos_um = -QUIET_UM + (100 * (int32_t)t_ms);

        if (pos_um > end_um)
        {
            break;
        }

        enc = (int32_t)(((int64_t)(pos_um + QUIET_UM) * COUNTS_PER_M) / 1000000);

        barcode_update(is_black_at(pos_um) || is_black_at(pos_um - second_um),
                       t_ms, enc);
        barcode_check_end(enc);

        if (barcode_available())
        {
            code = barcode_get_code();
            (void)barcode_get_command();

            if ((code != BARCODE_NONE) && (code != BARCODE_INVALID))
            {
                valid++;
            }
        }
    }

    return valid;
}


static void
test_slow_and_guard(void)
{
    /* Encoder count at the barcode's last edge in drive(). */
    int32_t const end_enc =
        (int32_t)(((int64_t)(barcode_len_um + QUIET_UM) * COUNTS_PER_M) / 1000000);
    uint32_t t;
    bool any;

    /* Distance-based end: slow cars are not cut off mid-barcode. */
    build(PAT_A, false);
    check("decodes at 20 mm/s", drive(20, 0, 1) == BARCODE_A);
    check("decodes at 10 mm/s", drive(10, 0, 1) == BARCODE_A);

    /* Duplicate guard: a barcode within 200 mm of the last one is ignored. */
    check("second barcode 30 mm later is ignored",
          count_valid_two_barcodes(30) == 1U);
    check("second barcode 250 mm later is read",
          count_valid_two_barcodes(250) == 2U);

    /* Car stopped right after a barcode: flicker must not re-trigger it. */
    (void)drive(50, 0, 1);
    any = false;
    for (t = 0U; t < 2000U; t += SAMPLE_MS)
    {
        barcode_update((t / SAMPLE_MS) % 2U == 0U, 100000U + t, end_enc + 300);
        barcode_check_end(end_enc + 300);
        any |= barcode_available();
    }
    check("guard holds while the car is not moving", !any);
}


/*
 * Every Code 39 character decodes (raw character), forwards and
 * backwards: the decoder does not depend on which letter is printed.
 */
static void
test_every_character(void)
{
    static char const chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    static uint16_t const pats[] = {
        0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U,
        0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U,
        0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
        0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U,
        0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x0A8U,
        0x0A2U, 0x08AU, 0x02AU,
    };
    uint32_t i;
    bool all_ok = true;

    for (i = 0U; i < (sizeof pats / sizeof pats[0]); i++)
    {
        build(pats[i], false);
        (void)drive(100, 0, 1);
        all_ok &= (last_char == chars[i]);

        build(pats[i], true);
        (void)drive(100, 0, 1);
        all_ok &= (last_char == chars[i]);
    }

    check("all 43 Code 39 characters decode both ways", all_ok);
}


/*
 * Twice the course spec size (narrow 6 mm, wide 18 mm, ~282 mm), as
 * measured on the 4 Nov track: decodes at the follow speed in every
 * phase, with no size setting changed.
 */
static void
test_large_print(void)
{
    uint32_t p;
    bool ok = true;

    narrow_um = 2 * NARROW_UM;
    wide_um = 2 * WIDE_UM;

    for (p = 0U; p < PHASES; p++)
    {
        int32_t phase = (int32_t)((p * 150U * SAMPLE_MS) / PHASES);

        build(PAT_Z, false);
        ok &= (drive(150, phase, 1) == BARCODE_Z);
        build(PAT_Z, true);
        ok &= (drive(150, phase, 1) == BARCODE_Z);
    }

    check("2x print decodes at 150 mm/s", ok);

    narrow_um = NARROW_UM;
    wide_um = WIDE_UM;
}


/* Drive over elements[] as it stands (after edits), return the code. */
static barcode_t
drive_edited(int32_t speed_mm_s)
{
    barcode_len_um = 0;

    {
        uint32_t e;

        for (e = 0U; e < MAX_ELEMENTS; e++)
        {
            barcode_len_um += elements[e];
        }
    }

    return drive(speed_mm_s, 0, 1);
}


/*
 * Faults seen on the 4 Nov track: a 1-2 mm white glitch inside a wide
 * bar, and the first or last narrow bar + space not seen.
 */
static void
test_track_faults(void)
{
    uint32_t e;

    narrow_um = 2 * NARROW_UM;
    wide_um = 2 * WIDE_UM;

    /* Glitch: wide bar 12 of *Z* (18 mm) read as 14 mm black, 2 mm
     * white, 2 mm black. That adds two elements, so the last two (the
     * stop's narrow space and bar) drop off the 29-element array:
     * glitch and missing end together, as in the 4 Nov capture 150. */
    build(PAT_Z, false);
    for (e = MAX_ELEMENTS - 1U; e > 14U; e--)
    {
        elements[e] = elements[e - 2U];
    }
    elements[12] = 14000;
    elements[13] = 2000;
    elements[14] = 2000;
    check("white glitch inside a wide bar still decodes *Z*",
          drive_edited(150) == BARCODE_Z);

    /* First narrow bar and space missing: white instead. */
    build(PAT_Z, true);
    elements[0] = 0;            /* bar not there at all */
    elements[1] += narrow_um;
    check("first bar + space missing still decodes",
          drive_edited(150) == BARCODE_Z);

    /* Last narrow bar and space missing. */
    build(PAT_Z, false);
    elements[MAX_ELEMENTS - 1U] = 0;
    elements[MAX_ELEMENTS - 2U] = 0;
    check("last bar + space missing still decodes",
          drive_edited(150) == BARCODE_Z);

    narrow_um = NARROW_UM;
    wide_um = WIDE_UM;
}


/*
 * Travel (um) past the barcode's last bar before it is reported, or -1
 * if not reported as *Z* within 200 mm.
 */
static int32_t
report_distance_um(int32_t speed_mm_s, int32_t phase_um, bool reversed)
{
    int32_t pos_um;
    int32_t enc;
    uint32_t t_ms;

    build(PAT_Z, reversed);
    barcode_init();

    for (t_ms = 0U; t_ms < 20000U; t_ms += SAMPLE_MS)
    {
        pos_um = -QUIET_UM + phase_um + (speed_mm_s * (int32_t)t_ms);

        if (pos_um > (barcode_len_um + 200000))
        {
            break;
        }

        enc = (int32_t)(((int64_t)(pos_um + QUIET_UM) * COUNTS_PER_M) / 1000000);
        barcode_update(is_black_at(pos_um), t_ms, enc);
        barcode_check_end(enc);

        if (barcode_available())
        {
            return (barcode_get_command() == NAV_STRAIGHT)
                   ? (pos_um - barcode_len_um) : -1;
        }
    }

    return -1;
}


/*
 * The read must finish inside the white after the barcode, before the
 * next junction line. Code 39 needs a quiet zone of at least 10 narrow
 * widths, so that is the shortest white tested, for every print size
 * and nothing tuned to one track. Also prints the white each print
 * actually needs.
 */
static void
test_reported_within_quiet_zone(void)
{
    static int32_t const narrows[] = { 3000, 6000, 9000 };
    static int32_t const speeds[] = { 100, 150 };
    uint32_t n;
    uint32_t s;
    uint32_t p;
    uint32_t r;
    int32_t worst;
    int32_t got;
    bool ok = true;

    for (n = 0U; n < (sizeof narrows / sizeof narrows[0]); n++)
    {
        narrow_um = narrows[n];
        wide_um = 3 * narrows[n];
        worst = 0;

        for (s = 0U; s < (sizeof speeds / sizeof speeds[0]); s++)
        {
            for (p = 0U; p < PHASES; p++)
            {
                for (r = 0U; r < 2U; r++)
                {
                    got = report_distance_um(speeds[s],
                              (int32_t)((p * (uint32_t)speeds[s] * SAMPLE_MS)
                                        / PHASES),
                              r != 0U);

                    if ((got < 0) || (got >= (10 * narrows[n])))
                    {
                        ok = false;
                    }

                    if (got > worst)
                    {
                        worst = got;
                    }
                }
            }
        }

        (void)printf("narrow %d mm: reported within %d mm of the last bar "
                     "(quiet zone 10 x narrow = %d mm)\n",
                     (int)(narrows[n] / 1000), (int)(worst / 1000),
                     (int)(narrows[n] / 100));
    }

    check("reported inside a 10 x narrow quiet zone, any print size", ok);

    narrow_um = NARROW_UM;
    wide_um = WIDE_UM;
}


/*
 * The 11 captures of the 4 Nov track run (robot/debug, widths in
 * encoder counts, already through the 4-count flicker filter). The
 * long ones are *Z* read backwards; the short ones were taken while
 * the car swerved and must not give a command.
 */
static void
test_track_captures(void)
{
    static struct { char const * p_widths; barcode_t code; } const caps[] = {
        { "B103 W73 B119 W118 B123 W95 B56 W37 B140 W37 B37 W34 B37 W52 B27 W108", BARCODE_INVALID },
        { "B205 W88 B224 W78 B77 W248 B54 W92 B57 W93 B57 W92 B166 W38 B17 W93 B218 W224 B44 W83 B56 W107 B195 W109 B225 W71 B35", BARCODE_Z },
        { "B91 W51 B112 W116 B137 W102 B218 W31 B44 W44 B32 W45 B31 W110", BARCODE_INVALID },
        { "B195 W68 B201 W69 B76 W250 B63 W96 B65 W80 B64 W98 B180 W33 B15 W82 B183 W240 B66 W83 B68 W101 B202 W124 B175 W80 B43", BARCODE_Z },
        { "W60 B121 W121 B127 W93 B39 W37 B156 W26 B39 W39 B40 W45 B32 W102", BARCODE_INVALID },
        { "B183 W75 B237 W61 B77 W251 B80 W77 B55 W68 B66 W84 B217 W98 B221 W247 B53 W91 B45 W82 B203 W93 B242 W75 B39 W251 B64", BARCODE_Z },
        { "B146 W48 B127 W110 B124 W114 B49 W30 B145 W27 B42 W46 B33 W38 B51 W93", BARCODE_INVALID },
        { "B30 W71 B215 W77 B247 W67 B82 W246 B45 W70 B57 W89 B64 W99 B170 W15 B35 W102 B201 W236 B68 W85 B69 W86 B189 W101 B207 W71 B43", BARCODE_Z },
        { "B103 W75 B121 W74 B17 W29 B127 W91 B67 W16 B170 W15 B46 W45 B34 W36 B36 W113", BARCODE_INVALID },
        { "B49 W94 B226 W70 B248 W44 B72 W223 B76 W82 B66 W100 B66 W83 B165 W35 B18 W76 B183 W229 B85 W70 B68 W99 B220 W92 B215 W75 B52 W255 B70", BARCODE_Z },
        { "B91 W62 B103 W111 B13 W40 B134 W80 B53 W35 B96 W57 B17 W67 B30 W56 B51 W91", BARCODE_INVALID },
    };
    uint32_t i;
    bool all_ok = true;

    for (i = 0U; i < (sizeof caps / sizeof caps[0]); i++)
    {
        char const * p_text = caps[i].p_widths;
        int32_t enc = 1000;
        int32_t width;
        int32_t step;
        bool black = false;
        uint32_t t_ms = 0U;
        barcode_t code = BARCODE_NONE;

        barcode_init();
        barcode_update(p_text[0] != 'B', t_ms, enc);

        /* One edge per element, the end-gap check every 10 counts. */
        while (*p_text != '\0')
        {
            black = (*p_text == 'B');
            width = 0;
            p_text++;

            while ((*p_text >= '0') && (*p_text <= '9'))
            {
                width = (width * 10) + (*p_text - '0');
                p_text++;
            }

            while (*p_text == ' ')
            {
                p_text++;
            }

            barcode_update(black, t_ms, enc);

            for (step = 0; step < width; step += 10)
            {
                barcode_check_end(enc + step);
            }

            enc += width;
            t_ms += SAMPLE_MS;
        }

        barcode_update(!black, t_ms, enc);

        for (step = 0; (step < 2000) && !barcode_available(); step += 10)
        {
            barcode_check_end(enc + step);
        }

        if (barcode_available())
        {
            code = barcode_get_code();
            (void)barcode_get_command();
        }

        if (code != caps[i].code)
        {
            (void)printf("  track capture %u: got %d, want %d\n",
                         (unsigned int)i, (int)code, (int)caps[i].code);
            all_ok = false;
        }
    }

    check("4 Nov track captures: *Z* read, swerve reads rejected", all_ok);
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
    test_stop_mid_barcode();
    test_slow_and_guard();
    test_specks_before_barcode();
    test_every_character();
    test_large_print();
    test_track_faults();
    test_reported_within_quiet_zone();
    test_track_captures();
    jitter_ms = 2;
    report_speed_sweep(6000, "narrow 6 mm (4 Nov track print)");
    report_speed_sweep(3000, "narrow 3 mm (course spec)");
    report_speed_sweep(1500, "narrow 1.5 mm (small PDF print)");
    jitter_ms = 0;

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return (failures == 0U) ? 0 : 1;
}
