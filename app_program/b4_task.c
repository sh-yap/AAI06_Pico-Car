/** @file b4_task.c
 *
 * @brief Buddy 4 production task: sample, process, and publish terrain.
 *
 * The task owns the IMU, its I2C bus, and all b4_proc state. Other tasks
 * see Buddy 4 only through the terrain words in its message buffer and
 * ask for a run reset through an event flag, so no Buddy 4 data is shared
 * between tasks.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>

/* NOTE: No <stddef.h>: the kernel headers define size_t and NULL, and
 * the standard header clashes with them. */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "b4_cfg.h"
#include "b4_imu.h"
#include "b4_proc.h"
#include "b4_task.h"
#include "comms.h"

/* Below the VC and Buddy 3 (5): a late sample only delays a reading,
 * because every sample carries its own timestamp. */
#define B4_TASK_PRIORITY            (6)
#define B4_TASK_STACK_BYTES         (2048U)
#define B4_TASK_PERIOD_MS           (10U)     /* 100 Hz acquisition.     */
#define B4_TASK_PUBLISH_SAMPLES     (2U)      /* 50 Hz words, VC step.   */
#define B4_TASK_RETRY_MS            (500U)    /* Between IMU opens.      */

/* TEMPORARY: remove once the hump rule is decided; permanent terrain
 * telemetry belongs to Buddy 1.  1 repurposes the terrain fields on
 * robot/telemetry/terrain with every terrain word.  hump_dmm carries the
 * detector's own height, the absolute incline offset from the flat-ground
 * baseline converted with the lever, held at its 200 ms peak, in 0.1 mm.
 * tilt_mg carries the detectable height, the height of the live hump start
 * threshold, in 0.1 mm and not held.  It prints one console line per word
 * with t, roll change r, pitch change p, predicted pitch pp, magnitude
 * residual mr, the consistency flag c, the hump state h, the noise
 * estimate n in milli-g and the detectable height dh in micrometres. */
#ifndef B4_TASK_DIAG_TERRAIN
#define B4_TASK_DIAG_TERRAIN        (1)
#endif
#define B4_TASK_REPORT_MS           (5000U)   /* Console rate report.    */
/* NOTE: Two slots keep any queued word within 40 ms of its sample, so
 * the VC can judge freshness when it receives a word. */
#define B4_TASK_WORD_SLOTS          (2U)
#define B4_TASK_FLAG_RUN_RESET      (0x00000001U)
#define B4_TASK_FLAG_TICK           (0x00000002U)
#define B4_TASK_UM_PER_DMM          (100U)

#define B4_TASK_HEALTH_SHIFT        (0U)
#define B4_TASK_HEALTH_MASK         (0x3U)
#define B4_TASK_TURN_SHIFT          (2U)
#define B4_TASK_TURN_MASK           (0x3U)
#define B4_TASK_CURRENT_SHIFT       (4U)
#define B4_TASK_HIGHEST_SHIFT       (14U)
#define B4_TASK_HEIGHT_MASK         (0x3FFU)
#define B4_TASK_HEIGHT_MAX_DMM      (800U)
#define B4_TASK_VALID_SHIFT         (24U)
#define B4_TASK_HUMPS_SHIFT         (25U)
#define B4_TASK_HUMPS_MASK          (0xFU)
#define B4_TASK_IMPACTS_SHIFT       (29U)
#define B4_TASK_IMPACTS_MASK        (0x7U)

/* The kernel adds a small header to each message, as in Buddy 3. */
static UW               g_b4_task_word_buf[
    ((B4_TASK_WORD_SLOTS * sizeof(uint32_t)) / sizeof(UW))
    + (B4_TASK_WORD_SLOTS * 4U)];
static UW               g_b4_task_stack[B4_TASK_STACK_BYTES / sizeof(UW)];
static ID               g_b4_task_mbf_id = 0;
static ID               g_b4_task_flag_id = 0;
static b4_cfg_config_t  g_b4_task_config;
static uint32_t         g_b4_task_last_open_ms = 0U;
static uint8_t          g_b4_task_humps = 0U;
static uint8_t          g_b4_task_impacts = 0U;
static bool             g_b4_task_b_imu_open = false;
static volatile uint32_t g_b4_task_ticks = 0U;

/** @brief Return the kernel time in milliseconds.
 *
 * @return The low word of the system time, which wraps after 49 days.
 */
static uint32_t
b4_task_now_ms (void);

/** @brief Open the IMU when it is closed and the retry interval passed.
 *
 * @param now_ms The current time in milliseconds.
 */
static void
b4_task_open_imu (uint32_t now_ms);

/** @brief Read and process one sample, reopening the IMU after a fault.
 *
 * @return True when the sample was valid.
 */
static bool
b4_task_sample (void);

/** @brief Count completed humps and impacts from the latest status.
 *
 * @param p_status The status after the latest sample.
 */
static void
b4_task_count_events (b4_types_status_t const * p_status);

/** @brief Pack the latest status into a terrain word.
 *
 * @param p_status The status after the latest sample.
 * @return The packed word.
 */
static uint32_t
b4_task_pack (b4_types_status_t const * p_status);

#if B4_TASK_DIAG_TERRAIN
/** @brief TEMPORARY: report detector height, detectable height and state.
 *
 * The detector height is absolute and held at its peak over 200 ms. The
 * detectable height is not held. One console line carries the unheld
 * values.
 *
 * @param p_status The status after the latest sample.
 */
static void
b4_task_diag_terrain (b4_types_status_t const * p_status);

#endif /* B4_TASK_DIAG_TERRAIN */

/** @brief Convert micrometres to clamped 0.1 mm steps.
 *
 * @param height_um The height in micrometres.
 * @return The height in 0.1 mm steps, at most 800.
 */
static uint32_t
b4_task_height_dmm (uint32_t height_um);

/** @brief Release the sampling task every period.
 *
 * @param p_exinf The unused extended information.
 */
static void
b4_task_tick_handler (void * p_exinf);

/** @brief Sample at 100 Hz and publish a word every second sample.
 *
 * @param stacd The unused start code.
 * @param p_exinf The unused extended information.
 */
static void
b4_task_sample_task (INT stacd, void * p_exinf);

bool
b4_task_start (void)
{
    T_CMBF cmbf = {0};
    T_CFLG cflg = {0};
    T_CTSK ctsk = {0};
    T_CCYC ccyc = {0};
    ID     task_id = 0;
    bool   b_started = false;

    g_b4_task_config = b4_cfg_default();

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ)sizeof(g_b4_task_word_buf);
    cmbf.maxmsz = (INT)sizeof(uint32_t);
    cmbf.bufptr = g_b4_task_word_buf;
    g_b4_task_mbf_id = tk_cre_mbf(&cmbf);

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_b4_task_flag_id = tk_cre_flg(&cflg);

    if ((0 < g_b4_task_mbf_id) && (0 < g_b4_task_flag_id) &&
        (B4_RESULT_OK == b4_proc_init(&g_b4_task_config)))
    {
#if TK_SUPPORT_SMP
        /* NOTE: Pinned to processor 1, which owns the I2C interrupt. The
         * port supports a peripheral driver only from that processor
         * (docs/PORT_RP2040.md), as for Buddy 3 and the ADC. */
        ctsk.tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC;
        ctsk.assprc = TP_PRC1;
#else
        ctsk.tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF;
#endif
        ctsk.task    = b4_task_sample_task;
        ctsk.itskpri = B4_TASK_PRIORITY;
        ctsk.stksz   = (SZ)sizeof(g_b4_task_stack);
        ctsk.bufptr  = g_b4_task_stack;
        task_id      = tk_cre_tsk(&ctsk);
        b_started    = (0 < task_id) && (E_OK == tk_sta_tsk(task_id, 0));
    }

    if (true == b_started)
    {
        /* NOTE: A delay of one 10 ms tick ends anywhere in the next tick,
         * so a cyclic handler, as in timing_probe.c, sets the period. */
        ccyc.cycatr = TA_HLNG | TA_STA | TA_PHS;
        ccyc.cychdr = (FP)b4_task_tick_handler;
        ccyc.cyctim = B4_TASK_PERIOD_MS;
        ccyc.cycphs = B4_TASK_PERIOD_MS;
        b_started   = (0 < tk_cre_cyc(&ccyc));
    }

    return (b_started);
}

bool
b4_task_receive_word (uint32_t * p_word)
{
    bool b_received = false;

    if ((NULL != p_word) && (0 < g_b4_task_mbf_id))
    {
        b_received = ((INT)sizeof(*p_word) ==
                      tk_rcv_mbf(g_b4_task_mbf_id, p_word, TMO_POL));
    }

    return (b_received);
}

void
b4_task_unpack (uint32_t word, b4_task_terrain_t * p_terrain)
{
    if (NULL != p_terrain)
    {
        /* Each field is masked to its width, and both enums cover all four
         * two-bit codes, so every conversion below is exact. */
        p_terrain->health = (b4_types_health_t)(
            (word >> B4_TASK_HEALTH_SHIFT) & B4_TASK_HEALTH_MASK);
        p_terrain->turn = (b4_types_turn_t)(
            (word >> B4_TASK_TURN_SHIFT) & B4_TASK_TURN_MASK);
        p_terrain->current_height_dmm = (uint16_t)(
            (word >> B4_TASK_CURRENT_SHIFT) & B4_TASK_HEIGHT_MASK);
        p_terrain->highest_height_dmm = (uint16_t)(
            (word >> B4_TASK_HIGHEST_SHIFT) & B4_TASK_HEIGHT_MASK);
        p_terrain->b_height_valid =
            (0U != ((word >> B4_TASK_VALID_SHIFT) & 1U));
        p_terrain->hump_count = (uint8_t)(
            (word >> B4_TASK_HUMPS_SHIFT) & B4_TASK_HUMPS_MASK);
        p_terrain->impact_count = (uint8_t)(
            (word >> B4_TASK_IMPACTS_SHIFT) & B4_TASK_IMPACTS_MASK);
    }
}

bool
b4_task_is_ramp_active (b4_task_terrain_t const * p_terrain)
{
    bool b_active = false;

    if (NULL != p_terrain)
    {
        b_active = (false == p_terrain->b_height_valid) &&
                   (0U != p_terrain->current_height_dmm);
    }

    return (b_active);
}

void
b4_task_request_run_reset (void)
{
    if (0 < g_b4_task_flag_id)
    {
        (void)tk_set_flg(g_b4_task_flag_id, B4_TASK_FLAG_RUN_RESET);
    }
}

static uint32_t
b4_task_now_ms (void)
{
    SYSTIM system_time = {0};

    (void)tk_get_otm(&system_time);

    return ((uint32_t)system_time.lo);
}

static void
b4_task_open_imu (uint32_t now_ms)
{
    if ((false == g_b4_task_b_imu_open) &&
        ((now_ms - g_b4_task_last_open_ms) >= B4_TASK_RETRY_MS))
    {
        g_b4_task_last_open_ms = now_ms;
        g_b4_task_b_imu_open =
            (B4_RESULT_OK == b4_imu_init(&g_b4_task_config));
        tm_printf((UB *)"[b4] IMU open %s\n",
                  (UB *)(g_b4_task_b_imu_open ? "OK" : "failed, retrying"));
    }
}

static bool
b4_task_sample (void)
{
    b4_types_sample_t sample = {0};
    b4_types_health_t imu_health = B4_HEALTH_NOT_INITIALIZED;
    b4_types_fault_t  imu_fault = B4_FAULT_NONE;
    uint8_t           failures = 0U;
    bool              b_valid = false;

    if (true == g_b4_task_b_imu_open)
    {
        /* A failed read leaves the sample invalid, which aborts any hump
         * in progress and withdraws readiness inside b4_proc. */
        b_valid = (B4_RESULT_OK == b4_imu_read(&sample));
        (void)b4_proc_process(&sample);
        (void)b4_imu_get_health(&imu_health, &imu_fault, &failures);

        /* NOTE: A sensor that lost power, such as an unplugged Grove
         * cable, comes back powered down, so it is closed and reopened
         * instead of being read again. */
        if (B4_HEALTH_FAULT == imu_health)
        {
            (void)b4_imu_close();
            g_b4_task_b_imu_open = false;
            tm_printf((UB *)"[b4] IMU fault %u, reopening\n",
                      (UINT)imu_fault);
        }
    }

    return (b_valid);
}

static void
b4_task_count_events (b4_types_status_t const * p_status)
{
    /* Counters wrap inside their field widths when packed, so plain
     * eight-bit wrap-around here is harmless. */
    if (true == p_status->b_hump_completed_event)
    {
        g_b4_task_humps++;
    }

    if (true == p_status->b_impact_event)
    {
        g_b4_task_impacts++;
    }
}

static uint32_t
b4_task_pack (b4_types_status_t const * p_status)
{
    b4_types_hump_state_t state = p_status->hump_state;
    uint32_t health = (uint32_t)p_status->health;
    uint32_t current_dmm = 0U;
    uint32_t valid = 0U;
    uint32_t word = 0U;

    if (false == g_b4_task_b_imu_open)
    {
        health = (uint32_t)B4_HEALTH_FAULT;
    }

    if ((B4_HUMP_ASCENDING == state) || (B4_HUMP_PEAK == state) ||
        (B4_HUMP_DESCENDING == state))
    {
        /* A hump in progress is never reported as zero height, because
         * zero with the valid bit clear means no hump. */
        current_dmm = b4_task_height_dmm(p_status->active_hump_height_um);
        if (0U == current_dmm)
        {
            current_dmm = 1U;
        }
    }
    else if (true == p_status->b_hump_height_valid)
    {
        current_dmm = b4_task_height_dmm(p_status->hump_peak_height_um);
        valid = 1U;
    }
    else
    {
        /* No hump, or the last one has no valid height: report zero. */
    }

    /* The enums are below four and the counters are masked, so every
     * field fits its width. */
    word |= (health & B4_TASK_HEALTH_MASK) << B4_TASK_HEALTH_SHIFT;
    word |= ((uint32_t)p_status->turn & B4_TASK_TURN_MASK) <<
            B4_TASK_TURN_SHIFT;
    word |= current_dmm << B4_TASK_CURRENT_SHIFT;
    word |= b4_task_height_dmm(p_status->highest_hump_height_um) <<
            B4_TASK_HIGHEST_SHIFT;
    word |= valid << B4_TASK_VALID_SHIFT;
    word |= ((uint32_t)g_b4_task_humps & B4_TASK_HUMPS_MASK) <<
            B4_TASK_HUMPS_SHIFT;
    word |= ((uint32_t)g_b4_task_impacts & B4_TASK_IMPACTS_MASK) <<
            B4_TASK_IMPACTS_SHIFT;

    return (word);
}

#if B4_TASK_DIAG_TERRAIN
static void
b4_task_diag_terrain (b4_types_status_t const * p_status)
{
    comms_terrain_t terrain;
    int32_t pitch_change_mg = 0;
    int64_t height_dmm = 0;
    int64_t incline_abs_mg = 0;
    uint32_t detectable_dmm = 0U;
    static int32_t peak_dmm = 0;
    static uint8_t calls = 0U;

    /* The status values are bounded by a few times 4000 milli-g, so the
     * subtraction cannot overflow. */
    pitch_change_mg = p_status->pitch_mg - p_status->pitch_baseline_mg;
    tm_printf((UB *)"[b4t] t=%u r=%d p=%d pp=%d mr=%d c=%u h=%u"
              " n=%d dh=%u\n",
              (UINT)p_status->sample.timestamp_ms,
              (INT)p_status->roll_change_mg,
              (INT)pitch_change_mg,
              (INT)(pitch_change_mg - p_status->tilt_residual_mg),
              (INT)p_status->magnitude_residual_mg,
              (UINT)(true == p_status->b_tilt_consistent),
              (UINT)p_status->hump_state,
              (INT)p_status->noise_mg,
              (UINT)p_status->detectable_height_um);

    incline_abs_mg = (int64_t)p_status->incline_delta_mg;
    if (incline_abs_mg < 0)
    {
        incline_abs_mg = -incline_abs_mg;
    }
    else
    {
        /* Already a magnitude. */
    }

    /* The cast widens the lever so the product cannot overflow. */
    height_dmm = ((int64_t)g_b4_task_config.hump_lever_um * incline_abs_mg) /
                 (1000 * (int64_t)B4_TASK_UM_PER_DMM);
    if (height_dmm > (int64_t)B4_TASK_HEIGHT_MAX_DMM)
    {
        height_dmm = (int64_t)B4_TASK_HEIGHT_MAX_DMM;
    }
    else
    {
        /* Already within the telemetry range. */
    }

    terrain.motion = COMMS_MOTION_QUIET;
    if (true == p_status->b_impact_event)
    {
        terrain.motion = COMMS_MOTION_IMPACT;
    }
    else if ((B4_HUMP_CANDIDATE == p_status->hump_state) ||
             (B4_HUMP_ASCENDING == p_status->hump_state) ||
             (B4_HUMP_PEAK == p_status->hump_state))
    {
        terrain.motion = COMMS_MOTION_CLIMBING_HUMP;
    }
    else if (B4_HUMP_DESCENDING == p_status->hump_state)
    {
        terrain.motion = COMMS_MOTION_DESCENDING_HUMP;
    }
    else
    {
        /* Quiet, as set above. */
    }

    /* The helper clamps the height to the telemetry range. */
    detectable_dmm = b4_task_height_dmm(p_status->detectable_height_um);
    terrain.tilt_mg = (int32_t)detectable_dmm;
    /* The cast narrows a value clamped to 0..800 above. */
    if ((int32_t)height_dmm > peak_dmm)
    {
        peak_dmm = (int32_t)height_dmm;
    }
    else
    {
        /* The held peak is still the largest. */
    }

    terrain.hump_dmm = peak_dmm;
    comms_report_terrain(&terrain);

    calls++;
    if (10U <= calls)
    {
        peak_dmm = 0;
        calls = 0U;
    }
    else
    {
        /* The 200 ms window is still open. */
    }
}
#endif /* B4_TASK_DIAG_TERRAIN */

static uint32_t
b4_task_height_dmm (uint32_t height_um)
{
    uint32_t height_dmm = height_um / B4_TASK_UM_PER_DMM;

    if (height_dmm > B4_TASK_HEIGHT_MAX_DMM)
    {
        height_dmm = B4_TASK_HEIGHT_MAX_DMM;
    }

    return (height_dmm);
}

static void
b4_task_tick_handler (void * p_exinf)
{
    (void)p_exinf;

    g_b4_task_ticks++;
    (void)tk_set_flg(g_b4_task_flag_id, B4_TASK_FLAG_TICK);
}

static void
b4_task_sample_task (INT stacd, void * p_exinf)
{
    b4_types_status_t status;
    uint32_t          report_ms = b4_task_now_ms();
    uint32_t          last_ticks = g_b4_task_ticks;
    uint32_t          samples = 0U;
    uint32_t          valid_samples = 0U;
    uint32_t          overruns = 0U;
    uint32_t          word = 0U;
    UINT              pattern = 0U;

    (void)stacd;
    (void)p_exinf;

    /* Allow the first open attempt immediately. */
    g_b4_task_last_open_ms = report_ms - B4_TASK_RETRY_MS;

    for (;;)
    {
        uint32_t now_ms = 0U;
        uint32_t ticks = 0U;

        pattern = 0U;
        (void)tk_wai_flg(g_b4_task_flag_id,
                         B4_TASK_FLAG_TICK | B4_TASK_FLAG_RUN_RESET,
                         TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);

        if (0U != (pattern & B4_TASK_FLAG_RUN_RESET))
        {
            (void)b4_proc_clear_run_maximum();
        }

        if (0U == (pattern & B4_TASK_FLAG_TICK))
        {
            continue;
        }

        /* More than one tick since the last sample means a missed period. */
        ticks = g_b4_task_ticks;
        overruns += (ticks - last_ticks) - 1U;
        last_ticks = ticks;
        now_ms = b4_task_now_ms();

        b4_task_open_imu(now_ms);
        if (true == b4_task_sample())
        {
            valid_samples++;
        }

        (void)b4_proc_get_status(b4_task_now_ms(), &status);
        b4_task_count_events(&status);
        samples++;

        if (0U == (samples % B4_TASK_PUBLISH_SAMPLES))
        {
            /* Never wait: if the VC falls behind, a lost word only costs
             * freshness, and the counters still carry every event. */
            word = b4_task_pack(&status);
            (void)tk_snd_mbf(g_b4_task_mbf_id, &word, (INT)sizeof(word),
                             TMO_POL);
#if B4_TASK_DIAG_TERRAIN
            b4_task_diag_terrain(&status);
#endif
        }

        if ((now_ms - report_ms) >= B4_TASK_REPORT_MS)
        {
            tm_printf((UB *)"[b4] samples/5s=%u valid=%u overruns=%u"
                      " health=%u\n",
                      (UINT)samples, (UINT)valid_samples,
                      (UINT)overruns, (UINT)status.health);
            report_ms = now_ms;
            samples = 0U;
            valid_samples = 0U;
            overruns = 0U;
        }
    }
}
