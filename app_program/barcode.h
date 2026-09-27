#ifndef BARCODE_H_
#define BARCODE_H_

#include <stdbool.h>
#include <stdint.h>

#include "navigation.h"


/* Barcode identifiers. Patterns for A/B/C/D pending confirmed encoding. */
typedef enum
{
    BARCODE_NONE = 0,
    BARCODE_A,
    BARCODE_B,
    BARCODE_C,
    BARCODE_D,
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

/* Reset acquisition - used for a new barcode or to recover from an invalid one. */
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
 * Call every cycle with the current timestamp. If a capture is
 * in progress and no transition has occurred for BARCODE_TIMEOUT_MS,
 * automatically finishes the capture (calls barcode_finish_capture()
 * internally). Safe to call every cycle regardless of state.
 */
void barcode_check_timeout(uint32_t timestamp_ms);

/* True once a complete barcode has been captured and is ready to decode. */
bool barcode_available(void);

/* Decoded barcode. Incomplete until A/B/C/D encoding is confirmed. */
barcode_t barcode_get_code(void);

/*
 * Converts the decoded barcode to a nav command
 * (A->LEFT, B->RIGHT, C->STRAIGHT, D->UTURN).
 *
 * Self-clearing: internally resets acquisition state
 * before returning, so barcode_available() goes back
 * to false once this has been called. Callers should
 * not also call barcode_reset() after this.
 */
nav_command_t barcode_get_command(void);

#endif /* BARCODE_H_ */