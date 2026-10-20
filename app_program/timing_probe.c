/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * timing_probe.c
 * Measures how late a 10 ms control-style task starts, to show whether
 * WiFi/MQTT activity disturbs control timing (roadmap Stage 10.8).
 * Built with COMMS_DEMO=1 only.
 *
 * A cyclic handler (timer interrupt context) stamps the time and wakes the
 * probe task.  The task stamps the time again when it actually runs; the
 * difference is its wake-up latency.  The probe task has the priority a
 * subsystem control loop would have (below the radio and USB tasks), so
 * any delay caused by radio polling or MQTT publishing shows up here.
 *
 * Every PROBE_REPORT_MS it prints, on the serial console:
 *   [probe] latency us: avg=.. max=.. >1ms=.. >5ms=.. n=.. (radio busy=..)
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "comms.h"
#include "comms_demo.h"
#include "timing_probe.h"

#if COMMS_DEMO

#define PROBE_PERIOD_MS      10U
#define PROBE_REPORT_MS      10000U
#define PROBE_TASK_PRIORITY  5          /* same as a subsystem control loop */
#define PROBE_STACK_SIZE     1024
#define PROBE_LATE_1MS_US    1000U
#define PROBE_LATE_5MS_US    5000U

/* RP2040 TIMER: free-running 1 MHz counter, low word (datasheet 4.6.5). */
#define RP2040_TIMERAWL      (0x40054028UL)

static ID                g_probe_task;
static volatile uint32_t g_fired_us;


static inline uint32_t
time_us (void)
{
    return *(volatile uint32_t const *) RP2040_TIMERAWL;
}


static void
probe_cyclic_handler (void * p_exinf)
{
    (void) p_exinf;

    g_fired_us = time_us();
    (void) tk_wup_tsk(g_probe_task);
}


static void
probe_task (INT stacd, void * p_exinf)
{
    uint32_t latency_us;
    uint32_t sum_us     = 0U;
    uint32_t max_us     = 0U;
    uint32_t count      = 0U;
    uint32_t late_1ms   = 0U;
    uint32_t late_5ms   = 0U;
    uint32_t reports_at = PROBE_REPORT_MS / PROBE_PERIOD_MS;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
        latency_us = time_us() - g_fired_us;

        sum_us += latency_us;
        count++;

        if (latency_us > max_us)
        {
            max_us = latency_us;
        }

        if (latency_us > PROBE_LATE_1MS_US)
        {
            late_1ms++;
        }

        if (latency_us > PROBE_LATE_5MS_US)
        {
            late_5ms++;
        }

        if (count >= reports_at)
        {
            tm_printf((UB *) "[probe] latency us: avg=%u max=%u >1ms=%u"
                             " >5ms=%u n=%u  mqtt=%s\n",
                      sum_us / count, max_us, late_1ms, late_5ms, count,
                      comms_is_connected() ? "up" : "down");
            sum_us   = 0U;
            max_us   = 0U;
            count    = 0U;
            late_1ms = 0U;
            late_5ms = 0U;
        }
    }
}


bool
timing_probe_start (void)
{
    T_CTSK ctsk = {0};
    T_CCYC ccyc = {0};

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = (FP) probe_task;
    ctsk.itskpri = PROBE_TASK_PRIORITY;
    ctsk.stksz   = PROBE_STACK_SIZE;
    g_probe_task = tk_cre_tsk(&ctsk);

    if ((g_probe_task <= 0) || (E_OK != tk_sta_tsk(g_probe_task, 0)))
    {
        return false;
    }

    ccyc.cycatr = TA_HLNG | TA_STA | TA_PHS;
    ccyc.cychdr = (FP) probe_cyclic_handler;
    ccyc.cyctim = PROBE_PERIOD_MS;
    ccyc.cycphs = PROBE_PERIOD_MS;

    return (tk_cre_cyc(&ccyc) > 0);
}

#endif /* COMMS_DEMO */
