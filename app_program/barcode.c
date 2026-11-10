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

/* BARCODE_MIN_EDGES is in barcode.h: the recorder uses it too. */

/*
 * Raw acquisition buffer. One sample per encoder position is kept, so a
 * stationary sensor cannot fill it with repeated readings.
 *
 * 480 samples at the 100 Hz task rate is 4.8 s. The largest complete
 * saved scan (4 Nov 2026, 30 edges) used 283 samples from its first edge
 * to completion, so this leaves room for the preceding quiet distance.
 * The buffer is deliberately bounded: a longer capture is reported as
 * invalid instead of decoding an overwritten prefix.
 */
#define BARCODE_RAW_MAX_SAMPLES       480U

/*
 * Level learning: a 128-bin histogram is 32 ADC counts per bin. The
 * split is chosen by Otsu's between-class variance, then the median of
 * each class is the level. This is not the minimum/maximum, so one hot
 * or dark sample cannot define a level.
 */
#define BARCODE_LEVEL_BINS            128U
#define BARCODE_LEVEL_BIN_SHIFT       5U
#define BARCODE_LEVEL_BIN_MID         16U
#define BARCODE_LEVEL_MIN_SAMPLES     8U
#define BARCODE_LEVEL_MIN_CLASS       2U

/*
 * Credibility: the two levels must stand clear of within-level noise.
 * 64 counts is a floor for the 12-bit ADC, not a barcode threshold.
 * Low/high separation below this, or below four times the measured
 * noise, is treated as no readable barcode.
 */
#define BARCODE_LEVEL_MIN_SEPARATION  64U
#define BARCODE_LEVEL_NOISE_FACTOR    4U
#define BARCODE_LEVEL_GATE_DIV        8U

/*
 * Slow level drift is followed with 32-sample local midpoints, using
 * the 10th/90th percentile of each block (not its extrema). A block
 * needs 16 samples and a credible local span; otherwise the nearest
 * valid block midpoint is used.
 */
#define BARCODE_LEVEL_BLOCK           32U
#define BARCODE_LEVEL_LOCAL_MIN       16U
#define BARCODE_LEVEL_LOCAL_LOW_PCT   10U
#define BARCODE_LEVEL_LOCAL_HIGH_PCT  90U
#define BARCODE_LEVEL_LOCAL_DIV       4U
#define BARCODE_RAW_BLOCKS \
    ((BARCODE_RAW_MAX_SAMPLES + BARCODE_LEVEL_BLOCK - 1U) \
     / BARCODE_LEVEL_BLOCK)

/* Hysteresis follows the measured noise, capped at a quarter of the span. */
#define BARCODE_LEVEL_HYST_FACTOR     2U
#define BARCODE_LEVEL_MIN_HYST        8U

/*
 * If the midpoint read fails, try seven thresholds at 1/8 .. 7/8 of
 * the learned level span. These are fractions of the observed
 * separation, never fixed ADC values. The first decode is always the
 * midpoint. Seven trials stay bounded and each crosses the span once.
 */
#define BARCODE_RAW_TRIALS            7U
#define BARCODE_RAW_TRIAL_FIRST       1U
#define BARCODE_RAW_TRIAL_DIV         8U

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


/* Raw ADC samples for the current acquisition, oldest first. */
static uint16_t raw_adc[BARCODE_RAW_MAX_SAMPLES];
static int32_t raw_encoder[BARCODE_RAW_MAX_SAMPLES];
static uint16_t raw_time_ms[BARCODE_RAW_MAX_SAMPLES];

/* Histogram used only while learning the current capture's levels. */
static uint16_t raw_hist[BARCODE_LEVEL_BINS];

static uint16_t raw_count = 0U;
static uint32_t raw_start_ms = 0U;

/* True once this acquisition has been driven by the raw-sample API. */
static bool raw_mode = false;
static bool raw_overflow = false;

/* Levels and threshold learned from the current raw samples. */
static bool raw_levels_valid = false;
static uint16_t raw_low_level = 0U;
static uint16_t raw_high_level = 0U;
static uint16_t raw_threshold = 0U;
static uint16_t raw_hysteresis = 0U;

/* Most recent raw sample classified with the learned threshold. */
static bool raw_black_state = false;

/* First reconstructed raw edge has latched this capture's guard state. */
static bool raw_guard_latched = false;


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


/* Median ADC value of histogram bins [first, last), count non-zero. */
static uint16_t barcode_raw_hist_median(uint8_t first, uint8_t last,
                                        uint32_t count)
{
    uint32_t seen = 0U;
    uint8_t bin;

    for (bin = first; bin < last; bin++)
    {
        seen += raw_hist[bin];

        if (seen >= ((count + 1U) / 2U))
        {
            /* bin is at most 127, so the shifted value fits uint16_t. */
            return (uint16_t)(((uint16_t)bin << BARCODE_LEVEL_BIN_SHIFT)
                              + BARCODE_LEVEL_BIN_MID);
        }
    }

    /* last is at least 1 here; keep the bound explicit for the cast. */
    return (uint16_t)(((uint16_t)(last - 1U) << BARCODE_LEVEL_BIN_SHIFT)
                      + BARCODE_LEVEL_BIN_MID);
}


/*
 * Median difference between adjacent samples that both stay on one
 * level. Reusing the ADC histogram keeps the extra RAM at zero. A
 * difference histogram ignores slow level drift across a long bar.
 */
static bool barcode_raw_pair_noise(uint16_t low_limit, uint16_t high_limit,
                                   bool b_high, uint16_t *p_noise)
{
    uint32_t count = 0U;
    uint32_t difference;
    uint16_t i;
    uint8_t bin;
    bool b_pair;

    for (bin = 0U; bin < BARCODE_LEVEL_BINS; bin++)
    {
        raw_hist[bin] = 0U;
    }

    for (i = 1U; i < raw_count; i++)
    {
        if (b_high)
        {
            b_pair = ((int32_t)raw_adc[i - 1U] >= (int32_t)high_limit)
                     && ((int32_t)raw_adc[i] >= (int32_t)high_limit);
        }
        else
        {
            b_pair = ((int32_t)raw_adc[i - 1U] <= (int32_t)low_limit)
                     && ((int32_t)raw_adc[i] <= (int32_t)low_limit);
        }

        if (b_pair)
        {
            difference = (raw_adc[i] > raw_adc[i - 1U])
                         ? (uint32_t)(raw_adc[i] - raw_adc[i - 1U])
                         : (uint32_t)(raw_adc[i - 1U] - raw_adc[i]);
            bin = (uint8_t)(difference >> BARCODE_LEVEL_BIN_SHIFT);

            if (bin >= BARCODE_LEVEL_BINS)
            {
                bin = (uint8_t)(BARCODE_LEVEL_BINS - 1U);
            }

            raw_hist[bin] = (uint16_t)(raw_hist[bin] + 1U);
            count++;
        }
    }

    if (count < BARCODE_LEVEL_MIN_CLASS)
    {
        return false;
    }

    *p_noise = barcode_raw_hist_median(0U, BARCODE_LEVEL_BINS, count);
    return true;
}


/*
 * Learn robust low/high levels and within-level noise from raw_adc[].
 * Returns false until the samples contain two credible levels.
 */
/* Fill raw_hist[] from raw_adc[] and return the sample count. */
static uint32_t barcode_raw_histogram(void)
{
    uint32_t total = 0U;
    uint16_t i;
    uint8_t bin;

    for (bin = 0U; bin < BARCODE_LEVEL_BINS; bin++)
    {
        raw_hist[bin] = 0U;
    }

    for (i = 0U; i < raw_count; i++)
    {
        /* ADC is 12-bit; the clamp only protects against a bad caller. */
        bin = (uint8_t)(raw_adc[i] >> BARCODE_LEVEL_BIN_SHIFT);

        if (bin >= BARCODE_LEVEL_BINS)
        {
            bin = (uint8_t)(BARCODE_LEVEL_BINS - 1U);
        }

        raw_hist[bin] = (uint16_t)(raw_hist[bin] + 1U);
    }

    for (bin = 0U; bin < BARCODE_LEVEL_BINS; bin++)
    {
        total += raw_hist[bin];
    }

    return total;
}


/* Otsu split bin that best separates the two histogram classes. */
static bool barcode_raw_otsu_split(uint32_t total, uint8_t *p_split)
{
    uint32_t sum_all = 0U;
    uint32_t weight_low = 0U;
    uint32_t sum_low = 0U;
    uint32_t weight_high;
    uint32_t sum_high;
    uint64_t score;
    uint64_t best_score = 0U;
    uint64_t numerator;
    uint64_t denominator;
    int64_t difference;
    uint8_t split = 0U;
    uint8_t bin;
    uint8_t t;

    for (bin = 0U; bin < BARCODE_LEVEL_BINS; bin++)
    {
        sum_all += (uint32_t)raw_hist[bin] * (uint32_t)bin;
    }

    /* Otsu split: maximise the variance between the two classes. */
    for (t = 1U; t < BARCODE_LEVEL_BINS; t++)
    {
        weight_low += raw_hist[t - 1U];
        sum_low += (uint32_t)raw_hist[t - 1U] * (uint32_t)(t - 1U);

        if ((0U != weight_low) && (weight_low != total))
        {
            weight_high = total - weight_low;
            sum_high = sum_all - sum_low;
            difference = ((int64_t)sum_low * (int64_t)weight_high)
                         - ((int64_t)sum_high * (int64_t)weight_low);

            if (difference < 0)
            {
                difference = -difference;
            }

            numerator = (uint64_t)difference * (uint64_t)difference;
            denominator = (uint64_t)weight_low * (uint64_t)weight_high;
            score = numerator / denominator;

            if (score > best_score)
            {
                best_score = score;
                split = t;
            }
        }
    }

    *p_split = split;
    return ((0U != best_score) && (0U != split));
}


/*
 * Learn robust low/high levels and within-level noise from raw_adc[].
 * Returns false until the samples contain two credible levels.
 */
static bool barcode_raw_find_levels(void)
{
    uint32_t total;
    uint32_t low_count = 0U;
    uint32_t high_count = 0U;
    int32_t low_limit;
    int32_t high_limit;
    uint16_t low;
    uint16_t high;
    uint16_t separation;
    uint16_t gate;
    uint16_t noise_low;
    uint16_t noise_high;
    uint16_t noise;
    uint8_t split = 0U;
    uint8_t bin;

    if (raw_count < BARCODE_LEVEL_MIN_SAMPLES)
    {
        return false;
    }

    total = barcode_raw_histogram();

    if (!barcode_raw_otsu_split(total, &split))
    {
        return false;
    }

    for (bin = 0U; bin < split; bin++)
    {
        low_count += raw_hist[bin];
    }

    for (bin = split; bin < BARCODE_LEVEL_BINS; bin++)
    {
        high_count += raw_hist[bin];
    }

    if ((low_count < BARCODE_LEVEL_MIN_CLASS) ||
        (high_count < BARCODE_LEVEL_MIN_CLASS))
    {
        return false;
    }

    low = barcode_raw_hist_median(0U, split, low_count);
    high = barcode_raw_hist_median(split, BARCODE_LEVEL_BINS, high_count);

    if (high <= low)
    {
        return false;
    }

    separation = (uint16_t)(high - low);
    gate = (uint16_t)(separation / BARCODE_LEVEL_GATE_DIV);

    if (0U == gate)
    {
        gate = 1U;
    }

    low_limit = (int32_t)low + (int32_t)gate;
    high_limit = (int32_t)high - (int32_t)gate;

    if (!barcode_raw_pair_noise((uint16_t)low_limit, (uint16_t)high_limit,
                                false, &noise_low) ||
        !barcode_raw_pair_noise((uint16_t)low_limit, (uint16_t)high_limit,
                                true, &noise_high))
    {
        return false;
    }

    noise = (noise_low > noise_high) ? noise_low : noise_high;

    if ((uint32_t)separation < (BARCODE_LEVEL_MIN_SEPARATION
                                + ((uint32_t)noise
                                   * BARCODE_LEVEL_NOISE_FACTOR)))
    {
        return false;
    }

    raw_low_level = low;
    raw_high_level = high;
    raw_threshold = (uint16_t)((uint32_t)low
                               + ((uint32_t)separation / 2U));
    raw_hysteresis = (uint16_t)(((uint32_t)noise
                                 * BARCODE_LEVEL_HYST_FACTOR)
                                + BARCODE_LEVEL_MIN_HYST);

    if (raw_hysteresis > (separation / 4U))
    {
        raw_hysteresis = (uint16_t)(separation / 4U);
    }

    raw_levels_valid = true;
    return true;
}


/* Value at a histogram percentile, on the same bin grid as the levels. */
static uint16_t barcode_raw_hist_percentile(uint32_t count, uint8_t percent)
{
    uint32_t target;
    uint32_t seen = 0U;
    uint8_t bin;

    target = ((count * (uint32_t)percent) + 99U) / 100U;

    if (target < 1U)
    {
        target = 1U;
    }

    for (bin = 0U; bin < BARCODE_LEVEL_BINS; bin++)
    {
        seen += raw_hist[bin];

        if (seen >= target)
        {
            /* bin is at most 127, so the shifted value fits uint16_t. */
            return (uint16_t)(((uint16_t)bin << BARCODE_LEVEL_BIN_SHIFT)
                              + BARCODE_LEVEL_BIN_MID);
        }
    }

    return 4095U;
}


/* Local midpoint for one block, or 0 if the block has no two levels. */
static uint16_t barcode_raw_local_threshold(uint16_t first,
                                            uint16_t count)
{
    uint32_t total = 0U;
    uint32_t below = 0U;
    uint32_t above = 0U;
    uint16_t low;
    uint16_t high;
    uint16_t separation;
    uint16_t midpoint;
    uint16_t i;
    uint8_t bin;

    if (count < BARCODE_LEVEL_LOCAL_MIN)
    {
        return 0U;
    }

    for (bin = 0U; bin < BARCODE_LEVEL_BINS; bin++)
    {
        raw_hist[bin] = 0U;
    }

    for (i = 0U; i < count; i++)
    {
        bin = (uint8_t)(raw_adc[first + i] >> BARCODE_LEVEL_BIN_SHIFT);

        if (bin >= BARCODE_LEVEL_BINS)
        {
            bin = (uint8_t)(BARCODE_LEVEL_BINS - 1U);
        }

        raw_hist[bin] = (uint16_t)(raw_hist[bin] + 1U);
        total++;
    }

    low = barcode_raw_hist_percentile(total,
                                      BARCODE_LEVEL_LOCAL_LOW_PCT);
    high = barcode_raw_hist_percentile(total,
                                       BARCODE_LEVEL_LOCAL_HIGH_PCT);

    if (high <= low)
    {
        return 0U;
    }

    separation = (uint16_t)(high - low);

    if ((separation < BARCODE_LEVEL_MIN_SEPARATION) ||
        (separation < ((uint16_t)(raw_high_level - raw_low_level)
                       / BARCODE_LEVEL_LOCAL_DIV)))
    {
        return 0U;
    }

    midpoint = (uint16_t)((uint32_t)low
                          + ((uint32_t)separation / 2U));

    for (i = 0U; i < count; i++)
    {
        if (raw_adc[first + i] < midpoint)
        {
            below++;
        }
        else
        {
            above++;
        }
    }

    if ((below < BARCODE_LEVEL_MIN_CLASS) ||
        (above < BARCODE_LEVEL_MIN_CLASS))
    {
        return 0U;
    }

    return midpoint;
}


/* Collect valid local block midpoints, in sample order. */
static uint8_t barcode_raw_local_table(uint16_t centers[],
                                       uint16_t thresholds[])
{
    uint16_t start = 0U;
    uint16_t samples;
    uint16_t value;
    uint8_t count = 0U;

    while ((start < raw_count) && (count < BARCODE_RAW_BLOCKS))
    {
        samples = (uint16_t)(raw_count - start);

        if (samples > BARCODE_LEVEL_BLOCK)
        {
            samples = BARCODE_LEVEL_BLOCK;
        }

        value = barcode_raw_local_threshold(start, samples);

        if (0U != value)
        {
            centers[count] = (uint16_t)(start + (samples / 2U));
            thresholds[count] = value;
            count++;
        }

        start = (uint16_t)(start + samples);
    }

    return count;
}


/* Interpolate the local midpoint; nearest valid block outside the table. */
static uint16_t barcode_raw_local_at(uint16_t position, uint8_t count,
                                     uint16_t const centers[],
                                     uint16_t const thresholds[],
                                     uint16_t fallback)
{
    int32_t threshold;
    int32_t difference;
    uint32_t span;
    uint32_t offset;
    uint8_t i;

    if (0U == count)
    {
        return fallback;
    }

    if (position <= centers[0])
    {
        return thresholds[0];
    }

    if (position >= centers[count - 1U])
    {
        return thresholds[count - 1U];
    }

    for (i = 1U; i < count; i++)
    {
        if (position <= centers[i])
        {
            span = (uint32_t)(centers[i] - centers[i - 1U]);
            offset = (uint32_t)(position - centers[i - 1U]);
            difference = (int32_t)thresholds[i]
                         - (int32_t)thresholds[i - 1U];
            threshold = (int32_t)thresholds[i - 1U]
                        + ((difference * (int32_t)offset)
                           / (int32_t)span);
            /* Interpolation stays between two uint16_t endpoints. */
            return (uint16_t)threshold;
        }
    }

    return thresholds[count - 1U];
}


/* Local midpoint when enabled, otherwise the constant trial threshold. */
static uint16_t barcode_raw_sample_threshold(uint16_t position,
                                             uint16_t fallback,
                                             bool b_local, uint8_t count,
                                             uint16_t const centers[],
                                             uint16_t const thresholds[])
{
    if (!b_local)
    {
        return fallback;
    }

    return barcode_raw_local_at(position, count, centers, thresholds,
                                fallback);
}


/*
 * Encoder position where the ADC line crosses crossing, between two
 * adjacent samples. The result is clamped to the two sample positions.
 */
static int32_t barcode_raw_crossing(uint16_t previous_adc,
                                    int32_t previous_encoder,
                                    uint16_t current_adc,
                                    int32_t current_encoder,
                                    int32_t crossing)
{
    int32_t denominator;
    int32_t numerator;
    int64_t delta_encoder;
    int64_t quotient;
    int64_t result;
    int64_t lower;
    int64_t upper;

    denominator = (int32_t)current_adc - (int32_t)previous_adc;

    if (0 == denominator)
    {
        return current_encoder;
    }

    numerator = crossing - (int32_t)previous_adc;
    delta_encoder = (int64_t)current_encoder - (int64_t)previous_encoder;

    /* numerator is 16-bit and delta_encoder 32-bit, so this fits int64_t. */
    quotient = ((int64_t)numerator * delta_encoder) / (int64_t)denominator;
    result = (int64_t)previous_encoder + quotient;
    lower = (previous_encoder < current_encoder)
            ? (int64_t)previous_encoder : (int64_t)current_encoder;
    upper = (previous_encoder > current_encoder)
            ? (int64_t)previous_encoder : (int64_t)current_encoder;

    if (result < lower)
    {
        result = lower;
    }

    if (result > upper)
    {
        result = upper;
    }

    /* result is clamped between two int32_t endpoints, so this is safe. */
    return (int32_t)result;
}


/* Add one reconstructed edge, dropping in-place flicker as before. */
static void barcode_raw_add_edge(bool is_black, uint32_t timestamp_ms,
                                 int32_t encoder_position)
{
    if ((edge_count > 0U) &&
        (abs_counts(encoder_position
                    - barcode_edges[edge_count - 1U].encoder_position)
         < BARCODE_MIN_ELEMENT_COUNTS))
    {
        edge_count--;
        return;
    }

    if (edge_count < BARCODE_MAX_EDGES)
    {
        barcode_edges[edge_count].is_black = is_black;
        barcode_edges[edge_count].timestamp_ms = timestamp_ms;
        barcode_edges[edge_count].encoder_position = encoder_position;
        edge_count++;
    }
    else
    {
        barcode_overflow = true;
    }
}


/* Widest stored element, used by the distance-based end gap. */
static void barcode_raw_update_widest(void)
{
    int32_t widest = 0;
    int32_t width;
    uint16_t i;

    for (i = 1U; i < edge_count; i++)
    {
        width = abs_counts(barcode_edges[i].encoder_position
                           - barcode_edges[i - 1U].encoder_position);

        if (width > widest)
        {
            widest = width;
        }
    }

    widest_counts = widest;
}


/* Rebuild all live edges from raw samples with one threshold/hysteresis. */
static void barcode_raw_rebuild_edges(uint16_t threshold,
                                      uint16_t hysteresis, bool b_local)
{
    uint16_t local_center[BARCODE_RAW_BLOCKS];
    uint16_t local_threshold[BARCODE_RAW_BLOCKS];
    uint8_t local_count = 0U;
    int32_t low_cross;
    int32_t high_cross;
    int32_t previous_encoder;
    int32_t current_encoder;
    int32_t position;
    int32_t last_transition;
    uint16_t previous_adc;
    uint16_t current_adc;
    uint16_t sample_threshold;
    uint32_t timestamp_ms;
    uint16_t i;
    bool black;
    bool any_transition = false;

    edge_count = 0U;
    widest_counts = 0;
    barcode_overflow = raw_overflow;

    if (0U == raw_count)
    {
        raw_black_state = false;
        return;
    }

    if (b_local)
    {
        local_count = barcode_raw_local_table(local_center,
                                              local_threshold);
    }

    previous_adc = raw_adc[0];
    previous_encoder = raw_encoder[0];
    sample_threshold = barcode_raw_sample_threshold(
        0U, threshold, b_local, local_count, local_center,
        local_threshold);
    black = ((int32_t)previous_adc >= (int32_t)sample_threshold);
    last_transition = last_activity_position;

    for (i = 1U; i < raw_count; i++)
    {
        current_adc = raw_adc[i];
        current_encoder = raw_encoder[i];
        timestamp_ms = raw_start_ms + (uint32_t)raw_time_ms[i];
        sample_threshold = barcode_raw_sample_threshold(
            i, threshold, b_local, local_count, local_center,
            local_threshold);
        low_cross = (int32_t)sample_threshold - (int32_t)hysteresis;

        if (low_cross < 0)
        {
            low_cross = 0;
        }

        high_cross = (int32_t)sample_threshold + (int32_t)hysteresis;

        if (!black && ((int32_t)current_adc >= high_cross))
        {
            position = barcode_raw_crossing(previous_adc, previous_encoder,
                                            current_adc, current_encoder,
                                            high_cross);
            barcode_raw_add_edge(true, timestamp_ms, position);
            black = true;
            last_transition = position;
            any_transition = true;
        }
        else if (black && ((int32_t)current_adc <= low_cross))
        {
            position = barcode_raw_crossing(previous_adc, previous_encoder,
                                            current_adc, current_encoder,
                                            low_cross);
            barcode_raw_add_edge(false, timestamp_ms, position);
            black = false;
            last_transition = position;
            any_transition = true;
        }
        else
        {
            /* No threshold crossing between these two samples. */
        }

        previous_adc = current_adc;
        previous_encoder = current_encoder;
    }

    raw_black_state = black;

    if (any_transition)
    {
        last_activity_position = last_transition;
    }

    barcode_raw_update_widest();
}


/* Threshold at trial/8 of the learned level span; trials start at 1/8. */
static uint16_t barcode_raw_trial_threshold(uint8_t trial)
{
    uint32_t separation;
    uint32_t offset;

    separation = (uint32_t)raw_high_level - (uint32_t)raw_low_level;
    offset = (separation
              * (uint32_t)(BARCODE_RAW_TRIAL_FIRST + trial))
             / BARCODE_RAW_TRIAL_DIV;

    return (uint16_t)((uint32_t)raw_low_level + offset);
}


/*
 * Decode the raw capture. The midpoint is tried first; only if it fails
 * are thresholds at fractions of the observed level span tried. A valid
 * character wins only if every successful trial agrees on it.
 */
static char barcode_decode_raw_capture(void)
{
    char character;
    char trial_character;
    char unique = C39_CHAR_NONE;
    uint16_t accepted_threshold = 0U;
    uint16_t midpoint_threshold = raw_threshold;
    uint16_t trial_threshold;
    uint8_t trial;
    bool b_conflict = false;

    if (!raw_levels_valid || raw_overflow)
    {
        return C39_CHAR_NONE;
    }

    (void)barcode_raw_rebuild_edges(midpoint_threshold, raw_hysteresis,
                                    true);
    character = barcode_decode_char();

    if (C39_CHAR_NONE != character)
    {
        return character;
    }

    for (trial = 0U; (trial < BARCODE_RAW_TRIALS) && !b_conflict; trial++)
    {
        trial_threshold = barcode_raw_trial_threshold(trial);

        if (trial_threshold != raw_threshold)
        {
            (void)barcode_raw_rebuild_edges(trial_threshold,
                                            raw_hysteresis, false);
            trial_character = barcode_decode_char();

            if (C39_CHAR_NONE != trial_character)
            {
                if (C39_CHAR_NONE == unique)
                {
                    unique = trial_character;
                    accepted_threshold = trial_threshold;
                }
                else if (unique != trial_character)
                {
                    b_conflict = true;
                }
                else
                {
                    /* Another trial agrees with the first. */
                }
            }
        }
    }

    if (b_conflict)
    {
        unique = C39_CHAR_NONE;
    }

    /* Leave the live edges and reported threshold on the final choice. */
    if (C39_CHAR_NONE != unique)
    {
        raw_threshold = accepted_threshold;
    }

    (void)barcode_raw_rebuild_edges(
        raw_threshold, raw_hysteresis,
        (raw_threshold == midpoint_threshold));

    return unique;
}


/* Keep one sample per encoder position so a stationary car cannot fill it. */
static void barcode_raw_store_sample(uint16_t adc, uint32_t timestamp_ms,
                                     int32_t encoder_position)
{
    uint32_t elapsed;
    uint16_t time_delta;

    if (0U == raw_count)
    {
        raw_start_ms = timestamp_ms;
        raw_adc[0] = adc;
        raw_encoder[0] = encoder_position;
        raw_time_ms[0] = 0U;
        raw_count = 1U;
        return;
    }

    elapsed = timestamp_ms - raw_start_ms;
    time_delta = (elapsed > UINT16_MAX)
                 ? UINT16_MAX : (uint16_t)elapsed; /* Clamped, safe cast. */

    if (encoder_position == raw_encoder[raw_count - 1U])
    {
        raw_adc[raw_count - 1U] = adc;
        raw_time_ms[raw_count - 1U] = time_delta;
        return;
    }

    if (raw_count >= BARCODE_RAW_MAX_SAMPLES)
    {
        raw_overflow = true;
        return;
    }

    raw_adc[raw_count] = adc;
    raw_encoder[raw_count] = encoder_position;
    raw_time_ms[raw_count] = time_delta;
    raw_count++;
}


/* Clear all raw acquisition state; the duplicate guard is separate. */
static void barcode_raw_clear(void)
{
    raw_count = 0U;
    raw_start_ms = 0U;
    raw_mode = false;
    raw_overflow = false;
    raw_levels_valid = false;
    raw_low_level = 0U;
    raw_high_level = 0U;
    raw_threshold = 0U;
    raw_hysteresis = 0U;
    raw_black_state = false;
    raw_guard_latched = false;
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

    barcode_raw_clear();
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


void barcode_update_raw(
    uint16_t adc,
    uint32_t timestamp_ms,
    int32_t encoder_position
)
{
    /* Do not overwrite a completed capture waiting to be processed. */
    if (barcode_ready)
    {
        return;
    }

    if (!raw_mode)
    {
        barcode_raw_clear();
        raw_mode = true;
    }

    /* Same distance-based duplicate guard expiry as barcode_update(). */
    if (guard_active &&
        (abs_counts(encoder_position - guard_position) >= guard_counts))
    {
        guard_active = false;
    }

    barcode_raw_store_sample(adc, timestamp_ms, encoder_position);

    if (!state_valid)
    {
        state_valid = true;
        last_activity_position = encoder_position;
    }

    if (barcode_raw_find_levels())
    {
        barcode_raw_rebuild_edges(raw_threshold, raw_hysteresis, true);

        /* Latch once, while guard_active still describes capture start. */
        if (!raw_guard_latched && (edge_count > 0U))
        {
            capture_in_guard = guard_active;
            raw_guard_latched = true;
        }
    }
    else
    {
        raw_levels_valid = false;
        raw_low_level = 0U;
        raw_high_level = 0U;
        raw_threshold = 0U;
        raw_hysteresis = 0U;
        raw_black_state = false;
        edge_count = 0U;
        widest_counts = 0;
        barcode_overflow = raw_overflow;
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
        if (raw_mode)
        {
            decoded_char = barcode_decode_raw_capture();
        }
        else
        {
            decoded_char = barcode_decode_char();
        }

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


uint16_t barcode_get_learned_threshold(void)
{
    return raw_levels_valid ? raw_threshold : 0U;
}


bool barcode_get_black_state(void)
{
    return raw_levels_valid ? raw_black_state : false;
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