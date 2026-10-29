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
 * character, stop), confirmed from the course sample *A*:
 * narrow element ~3 mm, wide ~9 mm, total ~141 mm.
 *
 * Widths come from encoder positions, so decoding does not
 * depend on speed. In each character exactly 3 of the 9
 * elements are wide, so the 3 widest are taken as wide -
 * no fixed mm threshold is needed. Barcodes read in the
 * opposite direction are decoded by reversing the elements.
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

/*
 * End of barcode, in encoder counts (average of both wheels).
 * If the car travels this far with no transition while a capture
 * is in progress, the capture is assumed complete.
 * Measured on the car 29 Oct (after Buddy 2's encoder changes):
 * narrow 3 mm ~ 37 counts, wide 9 mm ~ 110, so ~12.3 counts/mm
 * (matches Motion's 2536 counts/rev, 57.7 mm wheel, x1.14 floor
 * scale).  240 counts ~ 19.5 mm: over twice the widest element and
 * less than the white gap around a barcode.
 * Distance, not time, so the car stopping or slowing on a
 * barcode does not split the capture.
 */
#define BARCODE_END_GAP_COUNTS   240

/*
 * An edge that undoes the previous one within this many counts
 * (~0.3 mm, a narrow element is ~37) is sensor flicker, e.g. while
 * stopped on the boundary of a bar: both edges are dropped.
 */
#define BARCODE_MIN_ELEMENT_COUNTS   4

/* Captures with fewer edges are specks/noise and are dropped silently. */
#define BARCODE_MIN_EDGES        6U

/*
 * Duplicate guard: after a valid barcode, ignore the sensor for this
 * many counts so one barcode gives at most one command (Tribunal 1).
 * 2460 counts ~ 200 mm at ~12.3 counts/mm: longer than the barcode
 * (141 mm) plus its white margins.
 */
#define BARCODE_GUARD_COUNTS     2460

/* Code 39: 9 elements per character (5 bars, 4 spaces), 3 wide. */
#define C39_ELEMENTS_PER_CHAR    9U
#define C39_WIDE_PER_CHAR        3U

/* Start '*', one data character, stop '*'. */
#define C39_CHARS                3U

/* 9 elements per character plus one narrow gap between characters. */
#define C39_ELEMENTS \
    ((C39_CHARS * (C39_ELEMENTS_PER_CHAR + 1U)) - 1U)

/* Edges from the first white->black edge to the last black->white. */
#define C39_EDGES                (C39_ELEMENTS + 1U)

/*
 * Narrowest wide element must be at least this percentage of the
 * widest narrow element. Nominal is 300% (9 mm / 3 mm); kept low
 * to tolerate edge timing error, since the pattern lookup and the
 * '*' start/stop check still have to pass.
 */
#define C39_MIN_WIDE_PERCENT     125U

#define C39_PATTERN_STAR         0x094U    /* n w n n w n w n n */
#define C39_PATTERN_BAD          0xFFFFU

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
 * Course barcodes are *A*, *F* and *Z*.
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


/* Result decoded when the capture was finished. */
static barcode_t decoded_code = BARCODE_NONE;
static char decoded_char = C39_CHAR_NONE;


/*
 * Duplicate guard. Survives barcode_reset() on purpose, so that
 * clearing a used capture does not re-arm the sensor early.
 */
static bool guard_active = false;
static int32_t guard_position = 0;


/* Distance in encoder counts, either direction. */
static int32_t abs_counts(int32_t delta)
{
    return (delta < 0) ? -delta : delta;
}


/*
 * Wide/narrow pattern of one 9-element character, or
 * C39_PATTERN_BAD if wide and narrow cannot be told apart.
 */
static uint16_t c39_char_pattern(uint32_t const widths[C39_ELEMENTS_PER_CHAR])
{
    uint16_t pattern = 0U;
    uint32_t min_wide = UINT32_MAX;
    uint32_t max_narrow = 0U;
    uint32_t best;
    uint8_t best_index;
    uint8_t wide;
    uint8_t i;

    /* Mark the 3 widest elements as wide. */
    for (wide = 0U; wide < C39_WIDE_PER_CHAR; wide++)
    {
        best = 0U;
        best_index = 0U;

        for (i = 0U; i < C39_ELEMENTS_PER_CHAR; i++)
        {
            if (((pattern & (1U << (8U - i))) == 0U) && (widths[i] > best))
            {
                best = widths[i];
                best_index = i;
            }
        }

        pattern |= (uint16_t)(1U << (8U - best_index));
    }

    for (i = 0U; i < C39_ELEMENTS_PER_CHAR; i++)
    {
        if ((pattern & (1U << (8U - i))) != 0U)
        {
            if (widths[i] < min_wide)
            {
                min_wide = widths[i];
            }
        }
        else if (widths[i] > max_narrow)
        {
            max_narrow = widths[i];
        }
    }

    if ((min_wide * 100U) < (max_narrow * C39_MIN_WIDE_PERCENT))
    {
        return C39_PATTERN_BAD;
    }

    return pattern;
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


/* Data character of the element widths, read forwards or backwards. */
static char c39_decode(uint32_t const widths[C39_ELEMENTS], bool reversed)
{
    uint32_t char_widths[C39_ELEMENTS_PER_CHAR];
    uint16_t patterns[C39_CHARS];
    uint8_t index;
    uint8_t c;
    uint8_t e;

    for (c = 0U; c < C39_CHARS; c++)
    {
        for (e = 0U; e < C39_ELEMENTS_PER_CHAR; e++)
        {
            /* Skip the narrow gap after each character. */
            index = (uint8_t)((c * (C39_ELEMENTS_PER_CHAR + 1U)) + e);

            char_widths[e] = reversed
                ? widths[C39_ELEMENTS - 1U - index]
                : widths[index];
        }

        patterns[c] = c39_char_pattern(char_widths);
    }

    if ((patterns[0] != C39_PATTERN_STAR) ||
        (patterns[C39_CHARS - 1U] != C39_PATTERN_STAR))
    {
        return C39_CHAR_NONE;
    }

    return c39_lookup(patterns[1]);
}


/*
 * Decode the captured edges into the data character, or C39_CHAR_NONE.
 * Specks or flicker before/after the barcode add extra edges, so every
 * window of C39_EDGES edges starting on a white->black edge is tried;
 * the '*' start/stop check rejects the wrong ones.
 */
static char barcode_decode_char(void)
{
    uint32_t widths[C39_ELEMENTS];
    uint8_t first;
    uint8_t i;
    char character = C39_CHAR_NONE;

    if (barcode_overflow || (edge_count < C39_EDGES))
    {
        return C39_CHAR_NONE;
    }

    for (first = 0U;
         (first + C39_EDGES <= edge_count) && (character == C39_CHAR_NONE);
         first++)
    {
        if (!barcode_edges[first].is_black)
        {
            continue;
        }

        /* Absolute width, so reversing wheels still gives positive widths. */
        for (i = 0U; i < C39_ELEMENTS; i++)
        {
            widths[i] = (uint32_t)abs_counts(
                barcode_edges[first + i + 1U].encoder_position
                - barcode_edges[first + i].encoder_position);
        }

        character = c39_decode(widths, false);

        if (character == C39_CHAR_NONE)
        {
            character = c39_decode(widths, true);
        }
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
     * Duplicate guard: ignore the sensor until the car has
     * moved BARCODE_GUARD_COUNTS past the last valid barcode.
     * Measured either way, so reversing or a U-turn over the
     * same barcode does not trigger it again.
     */
    if (guard_active)
    {
        if (abs_counts(encoder_position - guard_position)
            < BARCODE_GUARD_COUNTS)
        {
            return;
        }

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
        barcode_ready = true;

        /* Valid course barcode: start the duplicate guard at its last edge. */
        if (decoded_code != BARCODE_INVALID)
        {
            guard_active = true;
            guard_position = last_activity_position;
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
        >= BARCODE_END_GAP_COUNTS)
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
            command = NAV_LEFT;
            break;

        case BARCODE_F:
            command = NAV_RIGHT;
            break;

        case BARCODE_Z:
            command = NAV_STRAIGHT;
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