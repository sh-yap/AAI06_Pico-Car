/** @file run_rec.c
 *
 * @brief Test-only run recorder ring buffer (see run_rec.h).
 */

#include <stdint.h>

#include "run_rec.h"

#ifndef VC_RECORDER
#define VC_RECORDER              (0)    /* Recorder off unless built in.    */
#endif

#if VC_RECORDER

/* The readout decodes this layout: fail the build if it changes. */
typedef char run_rec_record_size_check[
    (44U == sizeof(run_rec_record_t)) ? 1 : -1];

/* Likewise for the barcode ring. */
typedef char bc_rec_record_size_check[
    (36U == sizeof(bc_rec_record_t)) ? 1 : -1];

run_rec_t g_run_rec;                    /* Zero until the first BEGIN.      */
bc_rec_t  g_bc_rec;                     /* Zero until the first BEGIN.      */


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


void
run_rec_freeze (void)
{
    g_run_rec.frozen = 1U;
    g_bc_rec.frozen  = 1U;
}

#endif /* VC_RECORDER */

/*** end of file ***/
