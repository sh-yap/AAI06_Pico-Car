#ifndef BARCODE_H_
#define BARCODE_H_

#include <stdbool.h>
#include <stdint.h>

#include "navigation.h"


/* Barcode identifiers. Course codes *A*, *F*, *Z*, *Y*, *X*; anything else is INVALID. */
typedef enum
{
    BARCODE_NONE = 0,
    BARCODE_A,
    BARCODE_F,
    BARCODE_Z,
    BARCODE_Y,
    BARCODE_X,
    BARCODE_INVALID
} barcode_t;

/* Captures with fewer edges are specks/noise and are dropped silently.  The
 * recorder also uses this to decide whether a failed pass is worth keeping. */
#define BARCODE_MIN_EDGES    (6U)


/* One black/white transition while passing over a barcode. */
typedef struct
{
    bool is_black;
    uint32_t timestamp_ms;     
    int32_t encoder_position;
} barcode_edge_t;


/*
 * CR1 Phase 4 storage helpers.
 *
 * Only the storage in barcode.c is packed; barcode_edge_t stays the value
 * type of barcode_get_edge().  The helpers are static inline so barcode.c
 * and the round-trip tests use one implementation.
 *
 * - Raw ADC: 12-bit samples, two per three bytes (bits 0-7 in the first
 *   byte of the triplet, bits 8-11 of the even sample in the low nibble of
 *   the second byte, bits 8-11 of the odd sample in its high nibble, bits
 *   0-7 of the odd sample in the third byte).
 * - Edge: one 32-bit word, bit 0 is_black and bits 1-31 the signed encoder
 *   position (two's complement, clamped to -1073741824..1073741823 counts =
 *   +-76 km at 14.0 counts/mm), then one word for timestamp_ms.
 * - Element width: counts, clamped to 16 bits (the guard is 2460 counts).
 */
#define BARCODE_ADC_MASK            (UINT32_C(0x0FFF))
#define BARCODE_ADC_EVEN_HI_MASK    (UINT8_C(0x0F))
#define BARCODE_ADC_ODD_HI_MASK     (UINT8_C(0xF0))
#define BARCODE_ADC_ODD_HI_SHIFT    (4U)

#define BARCODE_EDGE_POS_BITS       (31U)
#define BARCODE_EDGE_POS_MASK       (UINT32_C(0x7FFFFFFF))
#define BARCODE_EDGE_POS_SHIFT      (1U)
#define BARCODE_EDGE_POS_MAX        (INT32_C(1073741823))
#define BARCODE_EDGE_POS_MIN        (-INT32_C(1073741824))
#define BARCODE_EDGE_IS_BLACK       (UINT32_C(1) << 0)

#define BARCODE_WIDTH_MAX           (UINT16_C(0xFFFF))

/* Signed value of a two's-complement field held in bits 0..bits-1. */
static inline int32_t barcode_sign_extend(uint32_t field, uint8_t bits)
{
    return (int32_t)(field << (32U - bits)) >> (32U - bits);
}

/* Store one 12-bit ADC sample; indices 2k and 2k+1 share one triplet. */
static inline void barcode_adc_pack(uint8_t *p_store, uint16_t index,
                                    uint16_t adc)
{
    uint32_t offset = ((uint32_t)index / 2U) * 3U;
    uint16_t value = (uint16_t)((uint32_t)adc & BARCODE_ADC_MASK);

    if ((index % 2U) == 0U)
    {
        p_store[offset] = (uint8_t)(value & UINT16_C(0x00FF));
        p_store[offset + 1U] = (uint8_t)
            ((p_store[offset + 1U] & BARCODE_ADC_ODD_HI_MASK)
             | (uint8_t)(value >> 8));
    }
    else
    {
        p_store[offset + 1U] = (uint8_t)
            ((p_store[offset + 1U] & BARCODE_ADC_EVEN_HI_MASK)
             | (uint8_t)((value & UINT16_C(0x0F00))
                         >> BARCODE_ADC_ODD_HI_SHIFT));
        p_store[offset + 2U] = (uint8_t)(value & UINT16_C(0x00FF));
    }
}

/* Read one 12-bit ADC sample stored by barcode_adc_pack(). */
static inline uint16_t barcode_adc_unpack(uint8_t const *p_store,
                                          uint16_t index)
{
    uint32_t offset = ((uint32_t)index / 2U) * 3U;
    uint16_t value;

    if ((index % 2U) == 0U)
    {
        value = (uint16_t)((uint16_t)p_store[offset]
                           | ((uint16_t)(p_store[offset + 1U]
                                         & BARCODE_ADC_EVEN_HI_MASK) << 8));
    }
    else
    {
        value = (uint16_t)((((uint16_t)p_store[offset + 1U]
                             & BARCODE_ADC_ODD_HI_MASK) << 4)
                           | (uint16_t)p_store[offset + 2U]);
    }

    return value;
}

/* Clamp an encoder position to the 31-bit signed edge field. */
static inline int32_t barcode_edge_clamp_position(int32_t position)
{
    if (position > BARCODE_EDGE_POS_MAX)
    {
        return BARCODE_EDGE_POS_MAX;
    }

    if (position < BARCODE_EDGE_POS_MIN)
    {
        return BARCODE_EDGE_POS_MIN;
    }

    return position;
}

/* Pack is_black and the clamped encoder position into one edge word. */
static inline uint32_t barcode_edge_word(bool is_black,
                                         int32_t encoder_position)
{
    uint32_t word;

    word = (((uint32_t)barcode_edge_clamp_position(encoder_position))
            & BARCODE_EDGE_POS_MASK) << BARCODE_EDGE_POS_SHIFT;

    if (is_black)
    {
        word |= BARCODE_EDGE_IS_BLACK;
    }

    return word;
}

static inline bool barcode_edge_word_is_black(uint32_t word)
{
    return (word & BARCODE_EDGE_IS_BLACK) != 0U;
}

/* Signed encoder position of an edge word packed by barcode_edge_word(). */
static inline int32_t barcode_edge_word_position(uint32_t word)
{
    return barcode_sign_extend(
        (word >> BARCODE_EDGE_POS_SHIFT) & BARCODE_EDGE_POS_MASK,
        (uint8_t)BARCODE_EDGE_POS_BITS);
}

/* Saturating conversion of an element width in counts. */
static inline uint16_t barcode_width_counts(uint32_t width)
{
    return (width > (uint32_t)BARCODE_WIDTH_MAX)
           ? BARCODE_WIDTH_MAX : (uint16_t)width;
}

/*
 * Offset of position from the capture base, false if it leaves int16
 * (the clamped value is stored anyway; see barcode.c).
 */
static inline bool barcode_encoder_offset(int32_t base, int32_t position,
                                          int16_t *p_offset)
{
    int64_t offset = (int64_t)position - (int64_t)base;
    bool fits = true;

    if (offset > (int64_t)INT16_MAX)
    {
        offset = (int64_t)INT16_MAX;
        fits = false;
    }
    else if (offset < (int64_t)INT16_MIN)
    {
        offset = (int64_t)INT16_MIN;
        fits = false;
    }
    else
    {
        /* In range for the stored field. */
    }

    *p_offset = (int16_t)offset;
    return fits;
}


void barcode_init(void);

/*
 * Reset acquisition - used for a new barcode or to recover from an
 * invalid one. Does not clear the duplicate guard (barcode_init() does).
 */
void barcode_reset(void);

/* Update acquisition with the current black/white sensor state. */
void barcode_update(
    bool is_black,
    uint32_t timestamp_ms,
    int32_t encoder_position
);

/*
 * Update acquisition with one raw barcode ADC sample.
 *
 * The raw samples are buffered so the low (white) and high (black)
 * levels can be learned from this scan, then classified into edges.
 * Keep calling barcode_check_end() with the same encoder position.
 * Do not mix this with barcode_update() during one acquisition.
 */
void barcode_update_raw(
    uint16_t adc,
    uint32_t timestamp_ms,
    int32_t encoder_position
);

/*
 * Threshold currently learned from the raw samples, or 0 while no
 * credible black/white levels are available.
 */
uint16_t barcode_get_learned_threshold(void);

/* Live raw-sample classification; false while no threshold is learned. */
bool barcode_get_black_state(void);


/*
 * Mark the current barcode capture as complete.
 *
 * The exact condition that ends a barcode will be
 * connected later once the barcode format is confirmed.
 */
void barcode_finish_capture(void);

/*
 * Call every cycle with the current encoder position. If a capture
 * is in progress and the car has travelled the end gap (twice the
 * widest element so far, see barcode.c) with no transition,
 * automatically finishes the capture (calls
 * barcode_finish_capture() internally). Stopping does not end a
 * capture. Safe to call every cycle regardless of state.
 */
void barcode_check_end(int32_t encoder_position);

/* True once a complete barcode has been captured and is ready to decode. */
bool barcode_available(void);

/* Decoded barcode: NONE if no capture is ready, INVALID if unreadable. */
barcode_t barcode_get_code(void);

/*
 * Raw Code 39 data character of the ready capture (e.g. 'A', 'Z'),
 * or '\0' if none is ready or it could not be read. Lets a character
 * outside the course table still be logged or reported.
 */
char barcode_get_char(void);

/* Debug: number of edges stored in the current capture. */
uint8_t barcode_get_edge_count(void);

/* Debug: copy stored edge 'index'; false if out of range. */
bool barcode_get_edge(uint8_t index, barcode_edge_t *p_edge);

/*
 * Converts the decoded barcode to a nav command
 * (A->LEFT, F->RIGHT, Z->STRAIGHT).
 *
 * Self-clearing: internally resets acquisition state
 * before returning, so barcode_available() goes back
 * to false once this has been called. Callers should
 * not also call barcode_reset() after this.
 */
nav_command_t barcode_get_command(void);

#endif /* BARCODE_H_ */