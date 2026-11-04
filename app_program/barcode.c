/*
 * barcode.c
 *
 * Buddy 3 - Barcode acquisition and decoding framework
 *
 * Records black/white transitions together with:
 * - transition timestamp
 * - encoder position
 *
 * Decodes Code 39 barcodes of the form *X* (start, one data
 * character, stop). Nothing depends on the print size or on
 * which character is printed: the course sample *A* was
 * narrow ~3 mm / wide ~9 mm (141 mm), the track barcode of
 * 4 Nov about twice that, and both decode the same way.
 *
 * Widths come from encoder positions, so decoding does not
 * depend on speed. Exactly 9 of the 29 elements of *X* are
 * wide, so the widest elements are taken as wide - no fixed
 * mm threshold. Each run of elements is compared with every
 * Code 39 character, read forwards and backwards, and the
 * read is accepted only if exactly one character fits.
 *
 * Seen on the track 4 Nov (robot/debug captures):
 * - a 1-2 mm white glitch inside a wide bar splits it into
 *   three elements, so short glitches are merged back;
 * - the first or last narrow bar and space can be missed,
 *   so up to C39_MAX_MISSING elements may be absent at each
 *   end of the barcode.
 */

#include "barcode.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * Maximum number of transitions that can be stored
 * during one barcode capture.
 *
 * This is only a storage limit, NOT the assumed
 * barcode length.
 */
#define BARCODE_MAX_EDGES    64U

/* Elements (widths between edges) that fit in one capture. */
#define BARCODE_MAX_ELEMENTS (BARCODE_MAX_EDGES - 1U)

/*
 * End of barcode, in encoder counts (average of both wheels,
 * ~12.3 counts/mm, measured on the car 29 Oct). Distance, not
 * time, so the car stopping or slowing on a barcode does not
 * split the capture.
 *
 * A capture ends when the car travels BARCODE_END_GAP_FACTOR times
 * the widest element seen so far with no transition, so the gap
 * scales with the print. Until a wide element must have been seen
 * (the first BARCODE_END_GAP_EARLY_EDGES edges: reading backwards,
 * the first wide element is the third) the factor is
 * BARCODE_END_GAP_EARLY_FACTOR, because a wide element can be over
 * 3x the narrow one before it. Never less than
 * BARCODE_END_GAP_MIN_COUNTS (26 mm), so one bar of a large print
 * is not taken for the end.
 *
 * Upper bound: the barcode must be reported before the car reaches
 * the junction after it, which on the 4 Nov track was 70-140 mm on.
 * The 4 Nov print (wide ~250 counts) ends 41 mm after its last bar.
 */
#define BARCODE_END_GAP_MIN_COUNTS      320
#define BARCODE_END_GAP_FACTOR          2
#define BARCODE_END_GAP_EARLY_FACTOR    3
#define BARCODE_END_GAP_EARLY_EDGES     3U

/*
 * An edge that undoes the previous one within this many counts
 * (~0.3 mm, a narrow element is ~37) is sensor flicker, e.g. while
 * stopped on the boundary of a bar: both edges are dropped.
 */
#define BARCODE_MIN_ELEMENT_COUNTS   4

/* Captures with fewer edges are specks/noise and are dropped silently. */
#define BARCODE_MIN_EDGES        6U

/*
 * Duplicate guard: after a valid barcode, a capture that begins within
 * the guard and reads the same character (or is unreadable) is
 * dropped, so one barcode
 * gives at most one command (Tribunal 1). The guard covers the
 * barcode's length plus BARCODE_GUARD_GAP_FACTOR end gaps, so a U-turn
 * (back over the end gap, then the whole barcode) is covered for any
 * print size, and is never less than BARCODE_GUARD_MIN_COUNTS
 * (~200 mm). The sensor is still read during the guard: with a large
 * print the guard reaches past a different barcode close behind (4 Nov
 * track: *Z* ~330 mm after *A* lost its first five bars), and that one
 * must still be read.
 */
#define BARCODE_GUARD_MIN_COUNTS     2460
#define BARCODE_GUARD_GAP_FACTOR     3

/*
 * Glitch: an element inside the capture narrower than
 * 1/BARCODE_GLITCH_DIV of the median element (the median is a narrow
 * element, since 20 of the 29 are narrow). On the track a 1-2 mm
 * glitch split a wide bar; it is resolved by trying each way of
 * merging it with its neighbours, or keeping it: at 200 mm/s a
 * sample is 2 mm, so a real narrow element can measure that small
 * (see barcode_build_variant()). Only the first BARCODE_MAX_GLITCHES
 * get variants, to bound the decode time (4^2 = 16 variants; every
 * good track read of 4 Nov had one glitch).
 */
#define BARCODE_GLITCH_DIV           3U
#define BARCODE_MAX_GLITCHES         2U
#define BARCODE_GLITCH_CHOICES       4U    /* glitch_merge_t values */

/* Code 39: 9 elements per character (5 bars, 4 spaces), 3 wide. */
#define C39_ELEMENTS_PER_CHAR    9U
#define C39_WIDE_PER_CHAR        3U

/* Start '*', one data character, stop '*'. */
#define C39_CHARS                3U

/* 9 elements per character plus one narrow gap between characters. */
#define C39_ELEMENTS \
    ((C39_CHARS * (C39_ELEMENTS_PER_CHAR + 1U)) - 1U)

/* Wide elements in a whole *X* barcode. */
#define C39_WIDE_TOTAL           (C39_CHARS * C39_WIDE_PER_CHAR)

/*
 * Elements that may be missing at each end of the barcode (the first
 * or last narrow bar and space were missed on the track), and the
 * fewest elements a readable capture can therefore hold.
 */
#define C39_MAX_MISSING          2U
#define C39_MIN_ELEMENTS         (C39_ELEMENTS - (2U * C39_MAX_MISSING))

/* Element positions of the data character within *X*. */
#define C39_DATA_FIRST           (C39_ELEMENTS_PER_CHAR + 1U)
#define C39_DATA_LAST            (C39_DATA_FIRST + C39_ELEMENTS_PER_CHAR - 1U)

/* Positions of the stop character within *X*. */
#define C39_STOP_FIRST           (2U * (C39_ELEMENTS_PER_CHAR + 1U))

/* Widest widths kept per run: every wide element plus one narrow. */
#define C39_TOP_WIDTHS           (C39_WIDE_TOTAL + 1U)

/*
 * Narrowest wide element must be at least this percentage of the
 * widest narrow element. Nominal is 300% (9 mm / 3 mm); kept low
 * to tolerate edge timing error, since the pattern lookup and the
 * '*' start/stop check still have to pass.
 */
#define C39_MIN_WIDE_PERCENT     125U

#define C39_PATTERN_STAR         0x094U    /* n w n n w n w n n */

/* Returned by c39_decode() when the barcode cannot be read. */
#define C39_CHAR_NONE            '\0'


/* Code 39 character and its wide/narrow pattern (first element = bit 8). */
typedef struct
{
    char character;
    uint16_t pattern;
} c39_symbol_t;

/* Full Code 39 set, so any final course character can be read. */
static c39_symbol_t const c39_table[] =
{
    { '0', 0x034U }, { '1', 0x121U }, { '2', 0x061U }, { '3', 0x160U },
    { '4', 0x031U }, { '5', 0x130U }, { '6', 0x070U }, { '7', 0x025U },
    { '8', 0x124U }, { '9', 0x064U }, { 'A', 0x109U }, { 'B', 0x049U },
    { 'C', 0x148U }, { 'D', 0x019U }, { 'E', 0x118U }, { 'F', 0x058U },
    { 'G', 0x00DU }, { 'H', 0x10CU }, { 'I', 0x04CU }, { 'J', 0x01CU },
    { 'K', 0x103U }, { 'L', 0x043U }, { 'M', 0x142U }, { 'N', 0x013U },
    { 'O', 0x112U }, { 'P', 0x052U }, { 'Q', 0x007U }, { 'R', 0x106U },
    { 'S', 0x046U }, { 'T', 0x016U }, { 'U', 0x181U }, { 'V', 0x0C1U },
    { 'W', 0x1C0U }, { 'X', 0x091U }, { 'Y', 0x190U }, { 'Z', 0x0D0U },
    { '-', 0x085U }, { '.', 0x184U }, { ' ', 0x0C4U }, { '$', 0x0A8U },
    { '/', 0x0A2U }, { '+', 0x08AU }, { '%', 0x02AU },
};


/*
 * Course characters and the barcode they report.
 * Course barcodes are *A*, *F*, *Z*, *Y* and *X*.
 * HARDCODED for now: update this table and barcode_get_command()
 * if the codes change on exam day.
 * Characters not listed are reported as INVALID.
 */
typedef struct
{
    char character;
    barcode_t code;
} course_code_t;

static course_code_t const course_codes[] =
{
    { 'A', BARCODE_A },
    { 'F', BARCODE_F },
    { 'Z', BARCODE_Z },
    { 'Y', BARCODE_Y },
    { 'X', BARCODE_X },
};

/* Stored black/white transition data. */
static barcode_edge_t barcode_edges[BARCODE_MAX_EDGES];


/* Number of valid edges currently stored. */
static uint8_t edge_count = 0U;


/* Previous sensor state - an edge is recorded only on a change. */
static bool previous_state = false;


/* False until the first sensor state has been received. */
static bool state_valid = false;


/* True when the current capture has been marked complete. */
static bool barcode_ready = false;


/* True when acquisition overflowed the edge buffer. */
static bool barcode_overflow = false;


/*
 * Encoder position of the last recorded edge (or acquisition start,
 * if no edge has occurred yet). Used by barcode_check_end()
 * to detect the end of a barcode.
 */
static int32_t last_activity_position = 0;


/* Widest element of the current capture, for the end gap. */
static int32_t widest_counts = 0;


/* Result decoded when the capture was finished. */
static barcode_t decoded_code = BARCODE_NONE;
static char decoded_char = C39_CHAR_NONE;


/*
 * Duplicate guard. Survives barcode_reset() on purpose, so that
 * clearing a used capture does not re-arm the sensor early.
 */
static bool guard_active = false;
static int32_t guard_position = 0;
static int32_t guard_counts = BARCODE_GUARD_MIN_COUNTS;
static char guard_char = C39_CHAR_NONE;

/* The current capture began within the duplicate guard. */
static bool capture_in_guard = false;


/* How a glitch element is merged in one decode variant. */
typedef enum
{
    GLITCH_KEEP = 0,        /* Kept as it is (also every non-glitch).   */
    GLITCH_MERGE_BOTH,      /* Both neighbours and the glitch: one.     */
    GLITCH_MERGE_BEFORE,    /* Glitch + left neighbour join the one
                               before (same colour as the glitch).      */
    GLITCH_MERGE_AFTER      /* Glitch + right neighbour join the one
                               after (same colour as the glitch).       */
} glitch_merge_t;


/*
 * Decode work buffers: static so the decode does not need ~600 bytes
 * of the Buddy 3 task stack. Used only inside barcode_decode_char().
 */
static uint32_t raw_widths[BARCODE_MAX_ELEMENTS];
static uint32_t variant_widths[BARCODE_MAX_ELEMENTS];
static uint8_t glitch_choice[BARCODE_MAX_ELEMENTS];


/* Distance in encoder counts, either direction. */
static int32_t abs_counts(int32_t delta)
{
    return (delta < 0) ? -delta : delta;
}


/* Travel with no transition that ends the current capture (see above). */
static int32_t barcode_end_gap(void)
{
    int32_t gap;

    if (edge_count <= BARCODE_END_GAP_EARLY_EDGES)
    {
        gap = BARCODE_END_GAP_EARLY_FACTOR * widest_counts;
    }
    else
    {
        gap = BARCODE_END_GAP_FACTOR * widest_counts;
    }

    if (gap < BARCODE_END_GAP_MIN_COUNTS)
    {
        gap = BARCODE_END_GAP_MIN_COUNTS;
    }

    return gap;
}


/*
 * True if the element at this position (0 .. C39_ELEMENTS - 1) of *X*
 * is wide. Only for the start and stop characters and the gaps between
 * characters; the data character is what is being read.
 */
static bool c39_frame_is_wide(uint8_t position)
{
    uint32_t bit = 0U;

    if (position < C39_ELEMENTS_PER_CHAR)
    {
        bit = C39_PATTERN_STAR >> (C39_ELEMENTS_PER_CHAR - 1U - position);
    }
    else if (position >= C39_STOP_FIRST)
    {
        bit = C39_PATTERN_STAR >>
              (C39_ELEMENTS_PER_CHAR - 1U - (position - C39_STOP_FIRST));
    }
    else
    {
        /* Gap between characters: always narrow. */
    }

    return ((bit & 1U) != 0U);
}


/* Data character for a pattern, or C39_CHAR_NONE. */
static char c39_lookup(uint16_t pattern)
{
    uint8_t i;

    for (i = 0U; i < (sizeof c39_table / sizeof c39_table[0]); i++)
    {
        if (c39_table[i].pattern == pattern)
        {
            return c39_table[i].character;
        }
    }

    return C39_CHAR_NONE;
}


/*
 * Add one width to top, the C39_TOP_WIDTHS widest widths so far,
 * widest first (0 = empty slot; every real width is above 0).
 */
static void c39_top_add(uint32_t top[C39_TOP_WIDTHS], uint32_t width)
{
    uint8_t slot;

    if (width > top[C39_TOP_WIDTHS - 1U])
    {
        slot = (uint8_t)(C39_TOP_WIDTHS - 1U);

        while ((slot > 0U) && (top[slot - 1U] < width))
        {
            top[slot] = top[slot - 1U];
            slot--;
        }

        top[slot] = width;
    }
}


/*
 * Data character if a run of count element widths is *X* with
 * missing_start elements absent before it (in the barcode's own
 * order), read forwards or reversed; otherwise C39_CHAR_NONE.
 * The caller keeps count + missing_start within C39_MAX_MISSING of
 * C39_ELEMENTS. b_first_black is the colour of the run's first element
 * and top holds the run's widest widths (c39_top_add()).
 */
static char c39_match(uint32_t const * p_widths, uint8_t count,
                      uint8_t missing_start, bool b_reversed,
                      bool b_first_black,
                      uint32_t const top[C39_TOP_WIDTHS])
{
    uint8_t wide_count = (uint8_t)C39_WIDE_TOTAL;
    uint8_t position;
    uint8_t i;
    uint16_t data = 0U;
    uint32_t min_wide;
    bool b_wide;

    /* Bars are the even positions of *X*. */
    position = b_reversed ? (uint8_t)(missing_start + count - 1U)
                          : missing_start;

    if (b_first_black != ((position % 2U) == 0U))
    {
        return C39_CHAR_NONE;
    }

    /* Wide elements in the missing ends are not in the run. */
    for (position = 0U; position < missing_start; position++)
    {
        if (c39_frame_is_wide(position))
        {
            wide_count--;
        }
    }

    for (position = (uint8_t)(missing_start + count);
         position < C39_ELEMENTS;
         position++)
    {
        if (c39_frame_is_wide(position))
        {
            wide_count--;
        }
    }

    /* The widest wide_count elements are the wide ones; they must
     * stand clear of the widest narrow one. */
    min_wide = top[wide_count - 1U];

    if ((min_wide * 100U) < (top[wide_count] * C39_MIN_WIDE_PERCENT))
    {
        return C39_CHAR_NONE;
    }

    for (i = 0U; i < count; i++)
    {
        position = b_reversed ? (uint8_t)(missing_start + count - 1U - i)
                              : (uint8_t)(missing_start + i);
        b_wide = (p_widths[i] >= min_wide);

        if ((position >= C39_DATA_FIRST) && (position <= C39_DATA_LAST))
        {
            if (b_wide)
            {
                data |= (uint16_t)(1U << (C39_DATA_LAST - position));
            }
        }
        else if (b_wide != c39_frame_is_wide(position))
        {
            return C39_CHAR_NONE;
        }
        else
        {
            /* Start, stop or gap element as expected. */
        }
    }

    return c39_lookup(data);
}


/*
 * Median of the first count raw widths (count > 0). Sorts a copy in
 * variant_widths, which is rebuilt afterwards for each variant.
 */
static uint32_t barcode_median_width(uint8_t count)
{
    uint32_t width;
    uint8_t i;
    uint8_t slot;

    for (i = 0U; i < count; i++)
    {
        width = raw_widths[i];
        slot = i;

        while ((slot > 0U) && (variant_widths[slot - 1U] > width))
        {
            variant_widths[slot] = variant_widths[slot - 1U];
            slot--;
        }

        variant_widths[slot] = width;
    }

    return variant_widths[count / 2U];
}


/*
 * Median of every second raw width from first (one colour), count
 * widths in all. Sorts a copy in variant_widths, as
 * barcode_median_width() does.
 */
static uint32_t barcode_median_colour(uint8_t count, uint8_t first)
{
    uint32_t width;
    uint8_t n = 0U;
    uint8_t i;
    uint8_t slot;

    for (i = first; i < count; i = (uint8_t)(i + 2U))
    {
        width = raw_widths[i];
        slot = n;

        while ((slot > 0U) && (variant_widths[slot - 1U] > width))
        {
            variant_widths[slot] = variant_widths[slot - 1U];
            slot--;
        }

        variant_widths[slot] = width;
        n++;
    }

    return (n > 0U) ? variant_widths[n / 2U] : 0U;
}


/*
 * Bars read narrower and spaces wider than printed: the sensor spot
 * and the fixed threshold move every edge by about the same distance.
 * On a small print that shift is a large part of each width (4 Nov
 * small *A*: narrow bars 12-25 counts, narrow spaces 47-61, wide bars
 * from 73), so a narrow space came within 20% of a wide bar and the
 * read failed C39_MIN_WIDE_PERCENT. Most bars and most spaces of *X*
 * are narrow, so the median bar and the median space are both narrow
 * and half their difference is the shift. It is measured on each
 * capture, so any print size, sensor height or threshold is handled.
 * The shift is added back to every bar and taken off every space;
 * widths stay at least 1 so a short glitch stays a glitch.
 */
static void barcode_correct_edge_shift(uint8_t count, bool b_black0)
{
    uint8_t const first_bar = b_black0 ? 0U : 1U;
    uint8_t const first_space = b_black0 ? 1U : 0U;
    int32_t shift;
    int32_t width;
    uint8_t i;

    shift = ((int32_t)barcode_median_colour(count, first_space)
             - (int32_t)barcode_median_colour(count, first_bar)) / 2;

    for (i = 0U; i < count; i++)
    {
        width = (int32_t)raw_widths[i];
        width = ((i % 2U) == first_bar) ? (width + shift) : (width - shift);
        raw_widths[i] = (width < 1) ? 1U : (uint32_t)width;
    }
}


/*
 * Build decode variant number variant of the raw widths into
 * variant_widths: digit g (base BARCODE_GLITCH_CHOICES) of variant says
 * how glitch g is merged. Merging always removes two elements, so
 * bars and spaces still alternate. Glitches are never the first or
 * last element. Returns the element count, or 0 if this variant
 * cannot be built (no element on the side it merges into).
 */
static uint8_t barcode_build_variant(uint8_t raw_count,
                                     uint8_t const glitch_index[],
                                     uint8_t glitches,
                                     uint32_t variant)
{
    uint32_t digits = variant;
    uint32_t carry = 0U;
    uint32_t width;
    uint8_t out = 0U;
    uint8_t i;
    bool b_ok = true;

    for (i = 0U; i < raw_count; i++)
    {
        glitch_choice[i] = (uint8_t)GLITCH_KEEP;
    }

    for (i = 0U; i < glitches; i++)
    {
        glitch_choice[glitch_index[i]] =
            (uint8_t)(digits % BARCODE_GLITCH_CHOICES);
        digits /= BARCODE_GLITCH_CHOICES;
    }

    i = 0U;

    while (b_ok && (i < raw_count))
    {
        width = raw_widths[i] + carry;
        carry = 0U;

        switch (glitch_choice[i])
        {
            case GLITCH_MERGE_BOTH:
                b_ok = (out > 0U);

                if (b_ok)
                {
                    variant_widths[out - 1U] += width + raw_widths[i + 1U];
                }

                i = (uint8_t)(i + 2U);
                break;

            case GLITCH_MERGE_BEFORE:
                b_ok = (out > 1U);

                if (b_ok)
                {
                    variant_widths[out - 2U] += variant_widths[out - 1U]
                                                + width;
                    out--;
                }

                i++;
                break;

            case GLITCH_MERGE_AFTER:
                b_ok = ((i + 2U) < raw_count);
                carry = width + raw_widths[i + 1U];
                i = (uint8_t)(i + 2U);
                break;

            case GLITCH_KEEP:
            default:
                variant_widths[out] = width;
                out++;
                i++;
                break;
        }
    }

    return b_ok ? out : 0U;
}


/*
 * Try every run of one variant (count elements in variant_widths, the
 * first one black if b_black0) against *X*. Each character found goes
 * into *p_character; a second, different one sets *p_b_conflict.
 */
static void barcode_match_variant(uint8_t count, bool b_black0,
                                  char * p_character, bool * p_b_conflict)
{
    uint32_t top[C39_TOP_WIDTHS];
    uint8_t first;
    uint8_t length;
    uint8_t missing;
    uint8_t direction;
    uint8_t i;
    bool b_first_black;
    char found;

    for (first = 0U; (first + C39_MIN_ELEMENTS) <= count; first++)
    {
        /* Colours alternate from the first element. */
        b_first_black = ((first % 2U) == 0U) ? b_black0 : !b_black0;

        /* Widest of the shortest run; each longer run adds one. */
        for (i = 0U; i < C39_TOP_WIDTHS; i++)
        {
            top[i] = 0U;
        }

        for (i = 0U; i < (C39_MIN_ELEMENTS - 1U); i++)
        {
            c39_top_add(top, variant_widths[first + i]);
        }

        for (length = (uint8_t)C39_MIN_ELEMENTS;
             (length <= C39_ELEMENTS) && ((first + length) <= count);
             length++)
        {
            c39_top_add(top, variant_widths[first + length - 1U]);

            for (missing = 0U; missing <= C39_MAX_MISSING; missing++)
            {
                /* At most C39_MAX_MISSING absent at the far end too. */
                if (((missing + length) > C39_ELEMENTS) ||
                    ((missing + length + C39_MAX_MISSING) < C39_ELEMENTS))
                {
                    continue;
                }

                for (direction = 0U; direction < 2U; direction++)
                {
                    found = c39_match(&variant_widths[first], length,
                                      missing, (direction != 0U),
                                      b_first_black, top);

                    if (found == C39_CHAR_NONE)
                    {
                        /* No match for this run. */
                    }
                    else if (*p_character == C39_CHAR_NONE)
                    {
                        *p_character = found;
                    }
                    else if (*p_character != found)
                    {
                        *p_b_conflict = true;
                    }
                    else
                    {
                        /* Same character again. */
                    }
                }
            }
        }
    }
}


/*
 * Decode the captured edges into the data character, or C39_CHAR_NONE.
 * Specks or flicker before/after the barcode add extra elements, so
 * every run of elements is tried; glitches are tried every way they
 * can merge. The read counts only if every match gives the same
 * character. corrected selects the widths with the bar/space shift
 * removed (barcode_correct_edge_shift()).
 */
static char barcode_decode_widths(uint8_t raw_count, bool b_corrected)
{
    uint8_t glitch_index[BARCODE_MAX_GLITCHES];
    uint32_t variants = 1U;
    uint32_t variant;
    uint32_t median;
    uint8_t glitches = 0U;
    uint8_t i;
    bool b_conflict = false;
    char character = C39_CHAR_NONE;

    /* Absolute width, so reversing wheels still gives positive widths. */
    for (i = 0U; i < raw_count; i++)
    {
        raw_widths[i] = (uint32_t)abs_counts(
            barcode_edges[i + 1U].encoder_position
            - barcode_edges[i].encoder_position);
    }

    if (b_corrected)
    {
        barcode_correct_edge_shift(raw_count, barcode_edges[0].is_black);
    }

    median = barcode_median_width(raw_count);

    for (i = 1U;
         ((i + 1U) < raw_count) && (glitches < BARCODE_MAX_GLITCHES);
         i++)
    {
        if ((raw_widths[i] * BARCODE_GLITCH_DIV) < median)
        {
            glitch_index[glitches] = i;
            glitches++;
            variants *= BARCODE_GLITCH_CHOICES;
        }
    }

    for (variant = 0U; (variant < variants) && !b_conflict; variant++)
    {
        barcode_match_variant(
            barcode_build_variant(raw_count, glitch_index, glitches,
                                  variant),
            barcode_edges[0].is_black, &character, &b_conflict);
    }

    return b_conflict ? C39_CHAR_NONE : character;
}


/*
 * Decode the captured edges into the data character, or C39_CHAR_NONE.
 * The widths are tried as measured first, so a read that decodes as it
 * is never changes; only if that fails are they tried again with the
 * bar/space shift removed, which is what rescues small prints.
 */
static char barcode_decode_char(void)
{
    uint8_t raw_count;
    char character;

    if (barcode_overflow || (edge_count < (C39_MIN_ELEMENTS + 1U)))
    {
        return C39_CHAR_NONE;
    }

    raw_count = (uint8_t)(edge_count - 1U);
    character = barcode_decode_widths(raw_count, false);

    if (character == C39_CHAR_NONE)
    {
        character = barcode_decode_widths(raw_count, true);
    }

    return character;
}


/* Course barcode for a decoded character; INVALID if not a course code. */
static barcode_t course_code_of(char character)
{
    uint8_t i;

    for (i = 0U; i < (sizeof course_codes / sizeof course_codes[0]); i++)
    {
        if (course_codes[i].character == character)
        {
            return course_codes[i].code;
        }
    }

    return BARCODE_INVALID;
}


void barcode_init(void)
{
    guard_active = false;
    barcode_reset();
}


void barcode_reset(void)
{
    edge_count = 0U;

    previous_state = false;
    state_valid = false;

    barcode_ready = false;
    barcode_overflow = false;
    widest_counts = 0;
    capture_in_guard = false;
    decoded_code = BARCODE_NONE;
    decoded_char = C39_CHAR_NONE;
}


void barcode_update(
    bool is_black,
    uint32_t timestamp_ms,
    int32_t encoder_position
)
{
    /*
     * Do not overwrite a completed capture that is
     * waiting to be processed.
     */
    if (barcode_ready)
    {
        return;
    }


    /*
     * Duplicate guard ends once the car has moved guard_counts past
     * the last valid barcode. Measured either way, so reversing or a
     * U-turn over the same barcode stays covered. Captures are still
     * taken meanwhile; barcode_finish_capture() drops repeats.
     */
    if (guard_active &&
        (abs_counts(encoder_position - guard_position) >= guard_counts))
    {
        guard_active = false;
    }


    /*
     * First sample:
     * establish the initial black/white state.
     * No edge is recorded because no transition
     * has happened yet. Acquisition distance starts here.
     */
    if (!state_valid)
    {
        previous_state = is_black;
        state_valid = true;
        last_activity_position = encoder_position;
        return;
    }


    /*
     * No state change means there is no new edge.
     */
    if (is_black == previous_state)
    {
        return;
    }


    /*
     * A black/white transition has occurred.
     */
    previous_state = is_black;
    last_activity_position = encoder_position;


    /* Flicker: this edge undoes the last one almost in place. */
    if ((edge_count > 0U) &&
        (abs_counts(encoder_position -
                    barcode_edges[edge_count - 1U].encoder_position)
         < BARCODE_MIN_ELEMENT_COUNTS))
    {
        edge_count--;
        return;
    }


    /*
     * Store the edge if buffer space is available.
     */
    if (edge_count < BARCODE_MAX_EDGES)
    {
        /* A capture that starts within the guard may be a repeat. */
        if (edge_count == 0U)
        {
            capture_in_guard = guard_active;
        }

        /* The element this edge closes, for the end gap. */
        if ((edge_count > 0U) &&
            (abs_counts(encoder_position -
                        barcode_edges[edge_count - 1U].encoder_position)
             > widest_counts))
        {
            widest_counts = abs_counts(
                encoder_position -
                barcode_edges[edge_count - 1U].encoder_position);
        }

        barcode_edges[edge_count].is_black =
            is_black;

        barcode_edges[edge_count].timestamp_ms =
            timestamp_ms;

        barcode_edges[edge_count].encoder_position =
            encoder_position;

        edge_count++;
    }
    else
    {
        /*
         * Too many transitions for the storage buffer.
         *
         * Keep the overflow flag so the completed
         * capture can later be reported as invalid.
         */
        barcode_overflow = true;
    }
}


void barcode_finish_capture(void)
{
    /*
     * At least two edges are required to calculate
     * one edge-to-edge width.
     *
     * Even an overflowed capture is marked ready
     * so that barcode_get_code() can return
     * BARCODE_INVALID.
     */
    if (edge_count >= BARCODE_MIN_EDGES)
    {
        decoded_char = barcode_decode_char();
        decoded_code = course_code_of(decoded_char);

        /*
         * Begun within the duplicate guard: the same barcode again,
         * or a partial read of it, is dropped. A different course
         * barcode is reported.
         */
        if (capture_in_guard &&
            ((decoded_code == BARCODE_INVALID) ||
             (decoded_char == guard_char)))
        {
            barcode_reset();
            return;
        }

        barcode_ready = true;

        /*
         * Valid course barcode: start the duplicate guard at its last
         * edge, long enough to cover this barcode and its end gaps.
         */
        if (decoded_code != BARCODE_INVALID)
        {
            guard_active = true;
            guard_char = decoded_char;
            guard_position = last_activity_position;
            guard_counts = abs_counts(
                barcode_edges[edge_count - 1U].encoder_position
                - barcode_edges[0].encoder_position)
                + (BARCODE_GUARD_GAP_FACTOR * barcode_end_gap());

            if (guard_counts < BARCODE_GUARD_MIN_COUNTS)
            {
                guard_counts = BARCODE_GUARD_MIN_COUNTS;
            }
        }
    }
    else
    {
        /*
         * Too few edges to be a barcode (a stray edge or a
         * speck). Discard it so it is not merged into the next
         * capture.
         */
        barcode_reset();
    }
}


void barcode_check_end(int32_t encoder_position)
{
    /*
     * Nothing to end if no acquisition is in
     * progress, or a completed capture is already
     * waiting to be processed.
     */
    if (!state_valid || barcode_ready)
    {
        return;
    }

    /* Either direction counts as travel. */
    if (abs_counts(encoder_position - last_activity_position)
        >= barcode_end_gap())
    {
        barcode_finish_capture();
    }
}


bool barcode_available(void)
{
    return barcode_ready;
}


barcode_t barcode_get_code(void)
{
    /*
     * No complete barcode is ready yet.
     */
    if (!barcode_ready)
    {
        return BARCODE_NONE;
    }


    /* Decoded once in barcode_finish_capture(). */
    return decoded_code;
}


char barcode_get_char(void)
{
    return barcode_ready ? decoded_char : '\0';
}


uint8_t barcode_get_edge_count(void)
{
    return edge_count;
}


bool barcode_get_edge(uint8_t index, barcode_edge_t *p_edge)
{
    if ((p_edge == 0) || (index >= edge_count))
    {
        return false;
    }

    *p_edge = barcode_edges[index];
    return true;
}


nav_command_t barcode_get_command(void)
{
    barcode_t code;
    nav_command_t command;


    /*
     * Do not reset an acquisition that is still
     * in progress.
     */
    if (!barcode_ready)
    {
        return NAV_NONE;
    }


    code = barcode_get_code();


    /*
     * Convert the decoded barcode into the
     * corresponding navigation command.
     */
    switch (code)
    {
        case BARCODE_A:
        case BARCODE_Y:
            command = NAV_LEFT;
            break;

        case BARCODE_F:
            command = NAV_RIGHT;
            break;

        case BARCODE_Z:
            command = NAV_STRAIGHT;
            break;

        case BARCODE_X:
            command = NAV_UTURN;
            break;

        case BARCODE_NONE:
        case BARCODE_INVALID:
        default:
            command = NAV_NONE;
            break;
    }


    /*
     * Self-clearing behaviour:
     *
     * Once a completed barcode has been processed,
     * clear the acquisition state so that the next
     * barcode can be recorded.
     */
    barcode_reset();


    return command;
}