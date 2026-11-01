#ifndef BARCODE_H_
#define BARCODE_H_

#include <stdbool.h>
#include <stdint.h>

#include "navigation.h"


/* Barcode identifiers. Course codes *A*, *F*, *Z*; anything else is INVALID. */
typedef enum
{
    BARCODE_NONE = 0,
    BARCODE_A,
    BARCODE_F,
    BARCODE_Z,
    BARCODE_INVALID
} barcode_t;


/* One black/white transition while passing over a barcode. */
typedef struct
{
    bool is_black;
    uint32_t timestamp_ms;     
    int32_t encoder_position;
} barcode_edge_t;


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
 * Mark the current barcode capture as complete.
 *
 * The exact condition that ends a barcode will be
 * connected later once the barcode format is confirmed.
 */
void barcode_finish_capture(void);

/*
 * Call every cycle with the current encoder position. If a capture
 * is in progress and the car has travelled BARCODE_END_GAP_COUNTS
 * with no transition, automatically finishes the capture (calls
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