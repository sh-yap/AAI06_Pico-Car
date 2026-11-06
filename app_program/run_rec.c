/** @file run_rec.c
 *
 * @brief Test-only run recorder ring buffer (see run_rec.h).
 */

#include <stdbool.h>
#include <stdint.h>

#include "run_rec.h"

#ifndef VC_RECORDER
#define VC_RECORDER              (0)    /* Recorder off unless built in.    */
#endif

#ifndef B4_RECORDER
#define B4_RECORDER              (0)    /* Buddy 4 ring off unless built in.*/
#endif

#if VC_RECORDER

/* The readout decodes this layout: fail the build if it changes. */
typedef char run_rec_record_size_check[
    (44U == sizeof(run_rec_record_t)) ? 1 : -1];

/* Likewise for the barcode ring. */
typedef char bc_rec_record_size_check[
    (36U == sizeof(bc_rec_record_t)) ? 1 : -1];

/* And the event log. */
typedef char ev_rec_size_check[
    ((8U == sizeof(ev_rec_record_t)) && (512U == sizeof(ev_rec_t))) ? 1 : -1];

#if B4_RECORDER
typedef char b4_rec_record_size_check[
    (40U == sizeof(b4_rec_record_t)) ? 1 : -1];

b4_rec_t  g_b4_rec;                     /* Zero until the first BEGIN.      */
#endif

run_rec_t g_run_rec;                    /* Zero until the first BEGIN.      */
bc_rec_t  g_bc_rec;                     /* Zero until the first BEGIN.      */
ev_rec_t  g_ev_rec;                     /* Zero until the first BEGIN.      */

/* Previous values for edge detection; only the VC task touches them. */
static bool     s_b_prev_lost;
static bool     s_b_prev_follow;
static bool     s_b_prev_ramp;
static bool     s_b_lost_open;          /* A LINE_LOST has no LINE_BACK yet.*/
static int8_t   s_prev_steer;
static uint32_t s_lost_ms;
static uint16_t s_prev_max_dmm;         /* Last highest height seen.        */


void
run_rec_begin (void)
{
    g_run_rec.frozen      = 1U;         /* A readout mid-reset sees frozen. */
    g_run_rec.write_index = 0U;
    g_run_rec.total       = 0U;
    g_run_rec.record_size = (uint16_t) sizeof(run_rec_record_t);
    g_run_rec.capacity    = (uint16_t) RUN_REC_CAPACITY;
    g_run_rec.magic       = RUN_REC_MAGIC;
    g_run_rec.frozen      = 0U;

    g_bc_rec.frozen       = 1U;         /* Same order as above.             */
    g_bc_rec.write_index  = 0U;
    g_bc_rec.total        = 0U;
    g_bc_rec.record_size  = (uint16_t) sizeof(bc_rec_record_t);
    g_bc_rec.capacity     = (uint16_t) BC_REC_CAPACITY;
    g_bc_rec.magic        = BC_REC_MAGIC;
    g_bc_rec.frozen       = 0U;

    g_ev_rec.frozen       = 1U;         /* Same order as above.             */
    g_ev_rec.count        = 0U;
    g_ev_rec.dropped      = 0U;
    g_ev_rec.record_size  = (uint16_t) sizeof(ev_rec_record_t);
    g_ev_rec.capacity     = (uint16_t) EV_REC_CAPACITY;
    g_ev_rec.magic        = EV_REC_MAGIC;
    g_ev_rec.frozen       = 0U;
    s_b_prev_lost         = false;
    s_b_prev_follow       = false;
    s_b_prev_ramp         = false;
    s_b_lost_open         = false;
    s_prev_steer          = 0;
    s_lost_ms             = 0U;
    s_prev_max_dmm        = 0U;

#if B4_RECORDER
    g_b4_rec.frozen       = 1U;         /* Same order as above.             */
    g_b4_rec.write_index  = 0U;
    g_b4_rec.total        = 0U;
    g_b4_rec.record_size  = (uint16_t) sizeof(b4_rec_record_t);
    g_b4_rec.capacity     = (uint16_t) B4_REC_CAPACITY;
    g_b4_rec.magic        = B4_REC_MAGIC;
    g_b4_rec.frozen       = 0U;
#endif
}


void
run_rec_add (run_rec_record_t const * p_record)
{
    run_rec_record_t * p_slot;

    if ((RUN_REC_MAGIC == g_run_rec.magic) && (0U == g_run_rec.frozen))
    {
        p_slot         = &g_run_rec.records[g_run_rec.write_index];
        *p_slot        = *p_record;
        p_slot->cycle  = (uint16_t) g_run_rec.total;    /* Wraps by design. */

        g_run_rec.write_index++;

        if (RUN_REC_CAPACITY <= g_run_rec.write_index)
        {
            g_run_rec.write_index = 0U;
        }

        g_run_rec.total++;
    }
}


/* Store one event, or count it as dropped when the log is full. */
static void
ev_rec_put (uint32_t ms, ev_rec_type_t type, int16_t value, uint8_t count)
{
    ev_rec_record_t * p_slot;

    if (EV_REC_CAPACITY > g_ev_rec.count)
    {
        p_slot        = &g_ev_rec.records[g_ev_rec.count];
        p_slot->ms    = ms;
        p_slot->value = value;
        p_slot->type  = (uint8_t) type;
        p_slot->count = count;
        g_ev_rec.count++;
    }
    else if (0xFFFFU > g_ev_rec.dropped)
    {
        g_ev_rec.dropped++;
    }
    else
    {
        /* Dropped count saturated. */
    }
}


void
ev_rec_step (uint32_t ms, bool b_lost, bool b_follow, int8_t steer,
             bool b_ramp)
{
    uint32_t since_ms;

    if ((EV_REC_MAGIC == g_ev_rec.magic) && (0U == g_ev_rec.frozen))
    {
        if (b_lost && (!s_b_prev_lost) && s_b_prev_follow)
        {
            ev_rec_put(ms, EV_REC_LINE_LOST, (int16_t) s_prev_steer, 0U);
            s_b_lost_open = true;
            s_lost_ms     = ms;
        }
        else if ((!b_lost) && s_b_prev_lost && s_b_lost_open)
        {
            since_ms = ms - s_lost_ms;          /* Wrap-safe. */
            if (32767U < since_ms)
            {
                since_ms = 32767U;
            }

            /* Cast: clamped to 32767 above, so it fits int16_t. */
            ev_rec_put(ms, EV_REC_LINE_BACK, (int16_t) since_ms, 0U);
            s_b_lost_open = false;
        }
        else
        {
            /* No line edge this step. */
        }

        if (b_ramp && (!s_b_prev_ramp))
        {
            ev_rec_put(ms, EV_REC_RAMP_START, 0, 0U);
        }
        else if ((!b_ramp) && s_b_prev_ramp)
        {
            ev_rec_put(ms, EV_REC_RAMP_END, 0, 0U);
        }
        else
        {
            /* No ramp edge this step. */
        }

        s_b_prev_lost   = b_lost;
        s_b_prev_follow = b_follow;
        s_b_prev_ramp   = b_ramp;
        s_prev_steer    = steer;
    }
}


void
ev_rec_hump (uint32_t ms, uint16_t height_dmm, uint8_t count, bool b_valid)
{
    uint8_t packed;

    if ((EV_REC_MAGIC == g_ev_rec.magic) && (0U == g_ev_rec.frozen))
    {
        packed = (uint8_t) (count & EV_REC_COUNT_MASK);
        if (b_valid)
        {
            packed = (uint8_t) (packed | EV_REC_VALID_BIT);
        }

        /* Cast: the terrain word's height is 12 bits at most. */
        ev_rec_put(ms, EV_REC_HUMP_DONE, (int16_t) height_dmm, packed);
    }
}


void
ev_rec_max (uint32_t ms, uint16_t highest_dmm)
{
    if ((EV_REC_MAGIC == g_ev_rec.magic) && (0U == g_ev_rec.frozen)
        && (highest_dmm != s_prev_max_dmm))
    {
        s_prev_max_dmm = highest_dmm;

        /* Cast: the terrain word's height is 12 bits at most. */
        ev_rec_put(ms, EV_REC_RUN_MAX, (int16_t) highest_dmm, 0U);
    }
}


void
ev_rec_impact (uint32_t ms, uint8_t count)
{
    if ((EV_REC_MAGIC == g_ev_rec.magic) && (0U == g_ev_rec.frozen))
    {
        ev_rec_put(ms, EV_REC_IMPACT, 0,
                   (uint8_t) (count & EV_REC_IMPACT_MASK));
    }
}


void
bc_rec_add (bc_rec_record_t const * p_record)
{
    uint32_t index;

    if ((BC_REC_MAGIC == g_bc_rec.magic) && (0U == g_bc_rec.frozen))
    {
        index = g_bc_rec.write_index;

        /* A reset by the VC between the checks above and here may leave
         * the index stale: stay inside the ring. */
        if (BC_REC_CAPACITY <= index)
        {
            index = 0U;
        }

        g_bc_rec.records[index] = *p_record;

        index++;

        if (BC_REC_CAPACITY <= index)
        {
            index = 0U;
        }

        g_bc_rec.write_index = index;
        g_bc_rec.total++;
    }
}


#if B4_RECORDER
void
b4_rec_add (b4_rec_record_t const * p_record)
{
    uint32_t index;

    if ((B4_REC_MAGIC == g_b4_rec.magic) && (0U == g_b4_rec.frozen))
    {
        index = g_b4_rec.write_index;

        /* A reset by the VC between the checks above and here may leave
         * the index stale: stay inside the ring. */
        if (B4_REC_CAPACITY <= index)
        {
            index = 0U;
        }

        g_b4_rec.records[index] = *p_record;

        index++;

        if (B4_REC_CAPACITY <= index)
        {
            index = 0U;
        }

        g_b4_rec.write_index = index;
        g_b4_rec.total++;
    }
}
#endif /* B4_RECORDER */


void
run_rec_freeze (void)
{
    g_run_rec.frozen = 1U;
    g_bc_rec.frozen  = 1U;
    g_ev_rec.frozen  = 1U;
#if B4_RECORDER
    g_b4_rec.frozen  = 1U;
#endif
}

#endif /* VC_RECORDER */

/*** end of file ***/
