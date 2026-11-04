/*
 * buddy3_task.c
 *
 * Buddy 3 - IR Line Following and Barcode Task
 *
 * Runs the Buddy 3 subsystem as a micro T-Kernel task.
 */

#include "buddy3_task.h"

#include "ir_sensor.h"
#include "line_follow.h"
#include "barcode.h"
#include "navigation.h"

#include "motion_encoder.h"
#include "comms.h"

#include <tm/tmonitor.h>

/* TEMPORARY (barcode diagnosis, 2026-11-04): send each finished capture to
 * robot/debug, so track runs on the power bank log it.  Same content as the
 * BUDDY3_DEBUG serial line, plus the threshold, the barcode ADC range seen
 * during the capture and its length.  Set to 0 or delete to remove. */
#define BUDDY3_SCAN_TO_MQTT      1

#ifndef VC_RECORDER
#define VC_RECORDER              (0)    /* Test-only recorder, run_rec.h.   */
#endif

#if VC_RECORDER
#include <stdint.h>
#include "run_rec.h"
#endif


/* Polling period.  A cyclic handler sets it, because a 10 ms delay runs
 * 20 ms on the 10 ms kernel tick (a relative wait adds one extra tick). */
#define BUDDY3_TASK_PERIOD_MS    10U

static ID g_buddy3_tid;


/* Wakes the Buddy 3 task once per period. */
static void
buddy3_tick_handler(void *p_exinf)
{
    (void)p_exinf;

    (void)tk_wup_tsk(g_buddy3_tid);
}

/*
 * Fallback threshold for the dedicated barcode IR sensor (GP28 / ADC2),
 * used when the START sweep did not see the sensor go black (it reads
 * beside the line, so it may not).  Measured 2026-10-27, room light, on the course barcode print, after
 * rotating the sensor so emitter and receiver sit side by side along
 * the bars (across the direction of travel), ~3-5 mm above the paper
 * (car stationary, sensor centred on each element):
 *   thin 3 mm white gap between bars ~1060, thin 3 mm bar ~3730.
 * 2026-10-29 driving test (*F*): white ~150-260, but a thin bar next to
 * a wide white space read only ~2080, so 2400 missed it and the capture
 * ended early.  PROVISIONAL: ~midway between thin gap (~1060) and that
 * thin bar (~2080).  Re-measure both (car still, sensor centred on the
 * thin bar beside a wide space, and on the thin gap between two wide
 * bars) and set the threshold midway.
 */
#define BARCODE_IR_THRESHOLD     1600U

/*
 * Output message buffer - default choice (message buffer, matching
 * the app_main.c demo pattern). Holds a few messages so a slightly
 * slow consumer does not force this task to block on every cycle.
 */
#define BUDDY3_OUTPUT_MBF_SLOTS  4U

/*
 * Debug output on the console. Set to 0 once calibration is done.
 * Prints a status line every BUDDY3_DEBUG_EVERY cycles
 * (20 x 10 ms = 200 ms) and one line per finished barcode capture.
 */
#define BUDDY3_DEBUG             1
#define BUDDY3_DEBUG_EVERY       20U

#if BUDDY3_DEBUG
static const char *const line_state_name[] = {
    "LEFT", "CENTRE", "RIGHT", "LOST", "JUNCTION"
};

/*
 * Print the finished capture: raw character, edge count, then each
 * element as B (bar) or W (space) with its width in encoder counts
 * (~12.3 counts/mm, so narrow ~37, wide ~110).
 */
static void debug_print_edges(void)
{
    uint8_t count = barcode_get_edge_count();
    uint8_t i;
    barcode_edge_t prev;
    barcode_edge_t edge;
    int32_t width;
    char raw = barcode_get_char();

    tm_printf((UB *)"[buddy3] edges=%d char=%c widths:",
              (INT)count, (raw != '\0') ? raw : '?');

    for (i = 1U; i < count; i++)
    {
        (void)barcode_get_edge((uint8_t)(i - 1U), &prev);
        (void)barcode_get_edge(i, &edge);
        width = edge.encoder_position - prev.encoder_position;

        if (width < 0)
        {
            width = -width;
        }

        tm_printf((UB *)" %c%d", prev.is_black ? 'B' : 'W', (INT)width);
    }

    tm_printf((UB *)"\n");
}
#endif

#if BUDDY3_SCAN_TO_MQTT
#define SCAN_TEXT_MAX    200U

static uint16_t g_scan_adc_min = 0xFFFFU;
static uint16_t g_scan_adc_max;

/* Append text at *p_len, never past the buffer (the rest is cut). */
static void scan_append(char *p_buf, uint32_t *p_len, char const *p_text)
{
    while (('\0' != *p_text) && (*p_len < (SCAN_TEXT_MAX - 1U)))
    {
        p_buf[*p_len] = *p_text;
        (*p_len)++;
        p_text++;
    }

    p_buf[*p_len] = '\0';
}

static void scan_append_uint(char *p_buf, uint32_t *p_len, uint32_t value)
{
    char     digits[11];
    uint32_t n = 0U;
    char     text[11];
    uint32_t i;

    do
    {
        digits[n] = (char)('0' + (value % 10U));
        n++;
        value /= 10U;
    } while ((0U != value) && (n < 10U));

    for (i = 0U; i < n; i++)
    {
        text[i] = digits[n - 1U - i];
    }

    text[n] = '\0';
    scan_append(p_buf, p_len, text);
}

/* Barcode ADC range while a capture is in progress (edges stored). */
static void scan_track_adc(uint16_t adc)
{
    if (0U == barcode_get_edge_count())
    {
        g_scan_adc_min = 0xFFFFU;
        g_scan_adc_max = 0U;
    }
    else
    {
        g_scan_adc_min = (adc < g_scan_adc_min) ? adc : g_scan_adc_min;
        g_scan_adc_max = (adc > g_scan_adc_max) ? adc : g_scan_adc_max;
    }
}

/* One line per finished capture, e.g.
 * "edges=31 char=? thr=1600 adc=210-2950 ms=480 len=812 B37 W36 B110 ..."
 * widths in encoder counts (narrow ~37, wide ~110). */
static void scan_publish(uint16_t threshold)
{
    char           text[SCAN_TEXT_MAX];
    uint32_t       len = 0U;
    uint8_t        count = barcode_get_edge_count();
    uint8_t        i;
    barcode_edge_t first;
    barcode_edge_t prev;
    barcode_edge_t edge;
    int32_t        width;
    char           raw = barcode_get_char();
    char           ch[2];

    ch[0] = (raw != '\0') ? raw : '?';
    ch[1] = '\0';
    text[0] = '\0';

    scan_append(text, &len, "edges=");
    scan_append_uint(text, &len, count);
    scan_append(text, &len, " char=");
    scan_append(text, &len, ch);
    scan_append(text, &len, " thr=");
    scan_append_uint(text, &len, threshold);
    scan_append(text, &len, " adc=");
    scan_append_uint(text, &len, (g_scan_adc_min <= g_scan_adc_max)
                                 ? g_scan_adc_min : 0U);
    scan_append(text, &len, "-");
    scan_append_uint(text, &len, g_scan_adc_max);

    if ((count > 1U) && barcode_get_edge(0U, &first)
        && barcode_get_edge((uint8_t)(count - 1U), &edge))
    {
        width = edge.encoder_position - first.encoder_position;
        scan_append(text, &len, " ms=");
        scan_append_uint(text, &len, edge.timestamp_ms - first.timestamp_ms);
        scan_append(text, &len, " len=");
        scan_append_uint(text, &len, (uint32_t)((width < 0) ? -width : width));
    }

    for (i = 1U; i < count; i++)
    {
        (void)barcode_get_edge((uint8_t)(i - 1U), &prev);
        (void)barcode_get_edge(i, &edge);
        width = edge.encoder_position - prev.encoder_position;
        scan_append(text, &len, prev.is_black ? " B" : " W");
        scan_append_uint(text, &len, (uint32_t)((width < 0) ? -width : width));
    }

    comms_debug_text(text);
}
#endif

ID buddy3_output_mbfid = -1;

static UW buddy3_output_mbf_buf[
    (BUDDY3_OUTPUT_MBF_SLOTS * sizeof(buddy3_packed_t)) / sizeof(UW)
    + BUDDY3_OUTPUT_MBF_SLOTS * 4U
];

static T_CMBF buddy3_output_cmbf = {
    .mbfatr  = TA_TFIFO,
    .bufsz   = sizeof buddy3_output_mbf_buf,
    .maxmsz  = sizeof(buddy3_packed_t),
    .bufptr  = buddy3_output_mbf_buf,
};


ID buddy3_output_mbf_create(void)
{
    buddy3_output_mbfid = tk_cre_mbf(&buddy3_output_cmbf);
    return buddy3_output_mbfid;
}

/*
 * Latest calibration request: the request code in bits 0-2, a counter
 * above it so a repeated request still shows as new. Written by the VC
 * task only and read by this task only, one aligned 32-bit word each way.
 */
static volatile uint32_t buddy3_cal_word = 0U;

void buddy3_calibration_request(buddy3_cal_request_t request)
{
    uint32_t count = (buddy3_cal_word >> 3) + 1U;

    buddy3_cal_word = (count << 3) | (uint32_t)request;
}

/*
 * Latest raw centre (bits 0-11) and right (bits 12-23) readings, for the
 * telemetry. Written by this task only, read by the VC task only, one
 * aligned 32-bit word.
 */
static volatile uint32_t buddy3_reading_word = 0U;

void buddy3_get_readings(uint16_t *p_centre, uint16_t *p_right)
{
    uint32_t word = buddy3_reading_word;

    *p_centre = (uint16_t)(word & 0xFFFU);
    *p_right = (uint16_t)((word >> 12) & 0xFFFU);
}

uint16_t buddy3_barcode_threshold(void)
{
    return line_cal_barcode_threshold(BARCODE_IR_THRESHOLD);
}

/* Console log of a finished sweep: what each sensor saw and the result. */
static void print_calibration(void)
{
    line_cal_result_t result;
    uint16_t barcode_threshold;

    line_cal_get_result(&result);
    barcode_threshold = buddy3_barcode_threshold();

    tm_printf((UB *)"[cal] b3 state=%d centre lo=%u hi=%u thr=%u full=%u"
                    " right lo=%u hi=%u thr=%u"
                    " barcode lo=%u hi=%u thr=%u (%s)\n",
              (INT)line_cal_state(),
              (UINT)result.centre.low, (UINT)result.centre.high,
              (UINT)result.centre.threshold, (UINT)result.centre_full_black,
              (UINT)result.right.low, (UINT)result.right.high,
              (UINT)result.right.threshold,
              (UINT)result.barcode.low, (UINT)result.barcode.high,
              (UINT)barcode_threshold,
              (UB *)((0U != result.barcode.threshold) ? "learned"
                                                       : "default"));
}

/* Act on a new request, then record this cycle's readings in the sweep. */
static void service_calibration(ir_readings_t readings)
{
    static uint32_t seen_word = 0U;
    static line_cal_state_t logged = LINE_CAL_IDLE;
    uint32_t word = buddy3_cal_word;

    if (word != seen_word)
    {
        seen_word = word;

        switch ((buddy3_cal_request_t)(word & 0x7U))
        {
            case BUDDY3_CAL_BEGIN:
                line_cal_begin();
                break;

            case BUDDY3_CAL_SWEEP:
                line_cal_sweep();
                break;

            case BUDDY3_CAL_END:
                (void)line_cal_end();
                break;

            case BUDDY3_CAL_ABORT:
            default:
                line_cal_abort();
                break;
        }
    }

    line_cal_update(readings);

    /* Log each finished sweep once, whether it passed or failed. */
    if (line_cal_state() != logged)
    {
        logged = line_cal_state();

        if (logged >= LINE_CAL_OK)
        {
            print_calibration();
        }
    }
}


void buddy3_task(INT stacd, void *exinf)
{
    ir_readings_t readings;
    line_state_t line_state;
    int8_t line_error;
    uint8_t line_sensors;
    barcode_t barcode_code;
    nav_command_t nav_command;

    SYSTIM sys_time;
    uint32_t timestamp_ms;

    motion_encoder_snapshot_t enc_snapshot;
    int32_t encoder_position;

    bool barcode_is_black;

    buddy3_status_t status;
    buddy3_packed_t packed;

    T_CCYC ccyc = {0};
    bool b_tick_ok;

#if BUDDY3_DEBUG
    uint32_t debug_cycle = 0U;
    bool barcode_was_ready;
#endif

#if VC_RECORDER
    bc_rec_record_t bc_rec;
    uint8_t bc_edges;
#endif

    (void)stacd;
    (void)exinf;

    ir_sensor_init();
    line_follow_init();
    barcode_init();

    g_buddy3_tid = tk_get_tid();
    ccyc.cycatr  = TA_HLNG | TA_STA | TA_PHS;
    ccyc.cychdr  = (FP)buddy3_tick_handler;
    ccyc.cyctim  = BUDDY3_TASK_PERIOD_MS;
    ccyc.cycphs  = BUDDY3_TASK_PERIOD_MS;
    b_tick_ok    = (0 < tk_cre_cyc(&ccyc));

    if (false == b_tick_ok)
    {
        tm_printf((UB*)"buddy3: cyclic handler failed, using tk_dly_tsk (20 ms loop)\n");
    }

    while (1)
    {
#if VC_RECORDER
        /* Bracket the sequential ADC reads and existing encoder snapshot
         * before calibration/debug output can delay them. OTM is the VC
         * clock; barcode decoding retains its existing system clock. */
        (void)tk_get_otm(&sys_time);
        bc_rec.ms = (uint32_t)sys_time.lo; /* Low 32 bits, wraps by design. */
#endif
        readings = ir_sensor_read();
#if VC_RECORDER
        motion_encoder_get_snapshot(&enc_snapshot);
        (void)tk_get_otm(&sys_time);
        bc_rec.sample_end_ms = (uint32_t)sys_time.lo; /* Same wrapping clock. */
        bc_rec.left_enc = enc_snapshot.left_counts;
        bc_rec.right_enc = enc_snapshot.right_counts;
        /* Hardware microseconds wrap at 2^32, retain exact short-run deltas. */
        bc_rec.encoder_us = (uint32_t)enc_snapshot.sampled_at_us;
#endif
        buddy3_reading_word = ((uint32_t)(readings.centre & 0xFFFU))
                              | ((uint32_t)(readings.right & 0xFFFU) << 12);
        service_calibration(readings);
        line_state = line_get_position(readings);
        line_error = line_get_error(readings);
        line_sensors = line_get_sensor_bits(readings);

        barcode_is_black = ir_sensor_is_black(
            readings.barcode, buddy3_barcode_threshold());

        tk_get_tim(&sys_time);
        timestamp_ms = sys_time.lo;

        /*
         * Average of both wheels - reasonable for a robot driving
         * roughly straight over a barcode. 
         */
#if !VC_RECORDER
        motion_encoder_get_snapshot(&enc_snapshot);
#endif
        encoder_position =
            (enc_snapshot.left_counts + enc_snapshot.right_counts) / 2;

        barcode_update(barcode_is_black, timestamp_ms, encoder_position);
        barcode_check_end(encoder_position);

#if BUDDY3_SCAN_TO_MQTT
        /* Before barcode_get_command(), which clears the capture. */
        scan_track_adc(readings.barcode);

        if (barcode_available())
        {
            scan_publish(buddy3_barcode_threshold());
        }
#endif

#if BUDDY3_DEBUG
        /* Sample before barcode_get_command(), which clears it. */
        barcode_was_ready = barcode_available();

        if (barcode_was_ready)
        {
            debug_print_edges();
        }
#endif

        /*
         * Read the code before barcode_get_command(), which clears it.
         * Both safely return NONE if nothing is ready.
         */
        barcode_code = barcode_get_code();

#if VC_RECORDER
        /* Sample the capture state before barcode_get_command() clears it.
         * Nothing here changes what Buddy 3 sends. */
        bc_edges           = barcode_get_edge_count();
        bc_rec.enc         = encoder_position;
        bc_rec.barcode_adc = readings.barcode;
        bc_rec.threshold   = buddy3_barcode_threshold();
        bc_rec.centre_adc  = readings.centre;
        bc_rec.right_adc   = readings.right;
        bc_rec.edges       = bc_edges;
        bc_rec.code        = (uint8_t)barcode_code;
        bc_rec.ch          = (uint8_t)barcode_get_char();
        bc_rec.flags       = barcode_is_black ? BC_REC_FLAG_BLACK : 0U;

        if (barcode_available())
        {
            bc_rec.flags |= BC_REC_FLAG_DONE;
        }
        else if (0U != bc_edges)
        {
            bc_rec.flags |= BC_REC_FLAG_ACTIVE;
        }
#endif

        nav_command = barcode_get_command();

#if VC_RECORDER
        bc_rec_add(&bc_rec);
#endif

#if BUDDY3_DEBUG
        if (barcode_was_ready)
        {
            tm_printf((UB *)"[buddy3] barcode captured at %u ms -> code=%d nav=%d\n",
                      timestamp_ms, (INT)barcode_code, (INT)nav_command);
        }

        if (++debug_cycle >= BUDDY3_DEBUG_EVERY)
        {
            debug_cycle = 0U;
            tm_printf((UB *)"[buddy3] C=%4u R=%4u B=%4u  %-8s err=%4d"
                           "  bits=%d  black=%d  enc=%d\n",
                      readings.centre, readings.right, readings.barcode,
                      line_state_name[line_state],
                      (INT)line_error, (INT)line_sensors,
                      (INT)barcode_is_black, (INT)encoder_position);
        }
#endif

        status.line_state   = line_state;
        status.line_error   = line_error;
        status.line_sensors = line_sensors;
        status.barcode      = barcode_code;
        status.nav_command  = nav_command;
        status.cal_state    = line_cal_state();

        packed = buddy3_status_pack(&status);

        (void)tk_snd_mbf(buddy3_output_mbfid, &packed,
                          (INT)sizeof(packed), TMO_POL);

        if (b_tick_ok)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
        else
        {
            tk_dly_tsk(BUDDY3_TASK_PERIOD_MS);
        }
    }
}