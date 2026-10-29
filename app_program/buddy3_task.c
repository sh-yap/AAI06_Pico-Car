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

#include <tm/tmonitor.h>


/* Polling period - may need adjusting once barcode timing is tested. */
#define BUDDY3_TASK_PERIOD_MS    10U

/*
 * Threshold for the dedicated barcode IR sensor (GP28 / ADC2).
 * Measured 2026-10-27, room light, on the course barcode print, after
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

#if BUDDY3_DEBUG
    uint32_t debug_cycle = 0U;
    bool barcode_was_ready;
#endif

    (void)stacd;
    (void)exinf;

    ir_sensor_init();
    line_follow_init();
    barcode_init();

    while (1)
    {
        readings = ir_sensor_read();
        line_state = line_get_position(readings);
        line_error = line_get_error(readings);
        line_sensors = line_get_sensor_bits(readings);

        barcode_is_black =
            ir_sensor_is_black(readings.barcode, BARCODE_IR_THRESHOLD);

        tk_get_tim(&sys_time);
        timestamp_ms = sys_time.lo;

        /*
         * Average of both wheels - reasonable for a robot driving
         * roughly straight over a barcode. 
         */
        motion_encoder_get_snapshot(&enc_snapshot);
        encoder_position =
            (enc_snapshot.left_counts + enc_snapshot.right_counts) / 2;

        barcode_update(barcode_is_black, timestamp_ms, encoder_position);
        barcode_check_end(encoder_position);

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
        nav_command = barcode_get_command();

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

        packed = buddy3_status_pack(&status);

        (void)tk_snd_mbf(buddy3_output_mbfid, &packed,
                          (INT)sizeof(packed), TMO_POL);

        tk_dly_tsk(BUDDY3_TASK_PERIOD_MS);
    }
}