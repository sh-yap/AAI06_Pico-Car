/*
 * barcode.c
 *
 * Buddy 3 - Barcode acquisition and decoding framework
 *
 * Records black/white transitions together with:
 * - transition timestamp
 * - encoder position
 *
 * These values will later be used to calculate
 * edge-to-edge barcode widths.
 *
 * The actual A/B/C/D barcode patterns are intentionally
 * not implemented until the official encoding is confirmed.
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
#define BARCODE_MAX_EDGES    32U

/*
 * TEMPORARY end-of-barcode timeout, in milliseconds.
 * If no transition occurs for this long while a capture
 * is in progress, the capture is assumed complete.
 */
#define BARCODE_TIMEOUT_MS   200U

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
 * Timestamp of the last recorded edge (or acquisition start,
 * if no edge has occurred yet). Used by barcode_check_timeout()
 * to detect the end of a barcode.
 */
static uint32_t last_activity_timestamp_ms = 0U;


void barcode_init(void)
{
    barcode_reset();
}


void barcode_reset(void)
{
    edge_count = 0U;

    previous_state = false;
    state_valid = false;

    barcode_ready = false;
    barcode_overflow = false;
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
     * First sample:
     * establish the initial black/white state.
     * No edge is recorded because no transition
     * has happened yet. Acquisition timing starts here.
     */
    if (!state_valid)
    {
        previous_state = is_black;
        state_valid = true;
        last_activity_timestamp_ms = timestamp_ms;
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
    last_activity_timestamp_ms = timestamp_ms;


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
    if (edge_count >= 2U)
    {
        barcode_ready = true;
    }
}


void barcode_check_timeout(uint32_t timestamp_ms)
{
    uint32_t elapsed_ms;

    /*
     * Nothing to time out if no acquisition is in
     * progress, or a completed capture is already
     * waiting to be processed.
     */
    if (!state_valid || barcode_ready)
    {
        return;
    }

    /*
     * Unsigned subtraction handles timestamp wraparound
     * correctly as long as the true elapsed time is less
     * than about half the uint32_t range.
     */
    elapsed_ms = timestamp_ms - last_activity_timestamp_ms;

    if (elapsed_ms >= BARCODE_TIMEOUT_MS)
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


    /*
     * Too many transitions were received.
     */
    if (barcode_overflow)
    {
        return BARCODE_INVALID;
    }


    /*
     * TODO:
     *
     * 1. Calculate edge-to-edge widths using:
     *
     *      barcode_edges[i].encoder_position
     *
     *    For example:
     *
     *      width =
     *          barcode_edges[i + 1].encoder_position
     *          - barcode_edges[i].encoder_position;
     *
     *
     * 2. Optionally compare timing using:
     *
     *      barcode_edges[i].timestamp_ms
     *
     *
     * 3. Normalise the measured widths.
     *
     *
     * 4. Compare the normalised pattern against the
     *    official A/B/C/D barcode encoding.
     *
     *
     * Do NOT implement the A/B/C/D patterns until
     * the encoding is confirmed by project specification.
     */


    /*
     * Decoding is intentionally incomplete for now.
     */
    return BARCODE_INVALID;
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

        case BARCODE_B:
            command = NAV_RIGHT;
            break;

        case BARCODE_C:
            command = NAV_STRAIGHT;
            break;

        case BARCODE_D:
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