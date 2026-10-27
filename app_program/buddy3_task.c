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
 * Measured 2026-10-27, room light:
 *   white paper <= 277, track tape >= 2725,
 *   inkjet barcode: wide bar ~600, thin bar ~240 (not visible in IR).
 */
#define BARCODE_IR_THRESHOLD     1300U

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
        barcode_check_timeout(timestamp_ms);

#if BUDDY3_DEBUG
        /* Sample before barcode_get_command(), which clears it. */
        barcode_was_ready = barcode_available();
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