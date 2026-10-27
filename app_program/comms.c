/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * comms.c
 * Communication API implementation.  See comms.h (callers) and
 * comms_link.h (radio task).
 *
 * Two sides, never blocking each other:
 *
 *   caller tasks (any core)             radio task (processor 1)
 *   -----------------------             ------------------------
 *   comms_report_*()  --> snapshot slots (seqlock) --+
 *   comms_event_*()   --> event message buffer ------+--> comms_link_next()
 *   comms_set_state() --> state/fault words ---------+      -> MQTT publish
 *   comms_get_command() <-- command message buffer <---- comms_link_on_command()
 *
 * Everything below "Radio side" runs only in the radio task, so its state
 * needs no locking.  Nothing here uses the heap; all storage is static.
 *
 * Payloads are written by comms_codec.c: packed binary by default (CR1
 * minimum-bit representation), or JSON when built with COMMS_FORMAT=json
 * (docs/BINARY_FORMAT.md).  Each message is described once below.  The
 * laptop decoder (tools/robot_decode.ps1) must list the same fields in the
 * same order and widths.
 */

#include <tk/tkernel.h>

#include "build_info.h"
#include "comms.h"
#include "comms_link.h"
#include "comms_codec.h"

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

#define COMMS_EVENT_QUEUE_LEN    16U    /* events waiting to be sent     */
#define COMMS_CMD_QUEUE_LEN      4U     /* commands waiting for pickup   */
#define COMMS_ACK_QUEUE_LEN      4U     /* acknowledgements to send      */
#define COMMS_TELEMETRY_MS       200U   /* max 5 Hz per telemetry topic  */
#define COMMS_REFRESH_MS         2000U  /* resend unchanged values       */
#define COMMS_HEARTBEAT_MS       1000U
#define COMMS_MBF_OVERHEAD       8U     /* per-message kernel header     */

#define QOS_0                    0U
#define QOS_1                    1U

/* ------------------------------------------------------------------ *
 *  Field widths in bits (docs/BINARY_FORMAT.md, from the team's shared
 *  data definitions).  Values outside a range are clamped.
 * ------------------------------------------------------------------ */

#define BITS_SPEED               10U    /* signed */
#define BITS_ENCODER             19U    /* signed */
#define BITS_DISTANCE            16U    /* signed */
#define BITS_LINE_POS            3U
#define BITS_LINE_ERROR          8U     /* signed */
#define BITS_SENSORS             2U     /* centre, right (Buddy 3)        */
#define BITS_MOTION_CLASS        3U
#define BITS_TILT                11U    /* signed */
#define BITS_HUMP                10U
#define BITS_SEQ                 16U
#define BITS_STATE               4U
#define BITS_FAULT               9U     /* nine faults (DATA_DEFS §6)     */
#define BITS_DROPS               8U
#define BITS_BARCODE             3U
#define BITS_NAV                 3U
#define BITS_OBST_DIST           12U
#define BITS_ANGLE               8U
#define BITS_WIDTH               10U
#define BITS_CLEARANCE           12U
#define BITS_ACTION              3U
#define BITS_RUN                 1U
#define BITS_RUN_MS              20U
#define BITS_COUNT               8U
#define BITS_CMD                 2U
#define BITS_OK                  1U

#define SEQ_MASK                 0xFFFFU   /* heartbeat number wraps     */
#define FAULT_MASK               0x1FFU    /* fault bits that are sent   */
#define RUN_BEGIN                1U        /* robot/event/run: 1 = BEGIN */
#define RUN_CODE_COUNT           2U        /* END, BEGIN                 */

/* Laptop command codes: ack field and 1-byte binary commands. */
#define CMD_CODE_UNKNOWN         0U
#define CMD_CODE_START           1U
#define CMD_CODE_STOP            2U
#define CMD_CODE_STATUS          3U
#define CMD_CODE_COUNT           4U

/* ------------------------------------------------------------------ *
 *  Types
 * ------------------------------------------------------------------ */

typedef enum
{
    EVENT_BARCODE = 0,
    EVENT_HUMP,
    EVENT_OBSTACLE,
    EVENT_MOTION,
    EVENT_RUN_BEGIN,
    EVENT_RUN_END
} event_type_t;

typedef struct
{
    uint32_t         time_ms;
    uint8_t          type;          /* event_type_t                      */
    uint8_t          code;          /* barcode character                 */
    uint8_t          kind;          /* nav command / motion class        */
    int32_t          value;         /* hump peak height, 0.1 mm          */
    comms_obstacle_t obstacle;
} comms_event_t;

/* Seqlock-protected latest value.  One writer task, one reader (radio). */
typedef struct
{
    volatile uint32_t seq;          /* odd while the writer is copying   */
    comms_motion_t    data;
} motion_slot_t;

typedef struct
{
    volatile uint32_t seq;
    comms_line_t      data;
} line_slot_t;

typedef struct
{
    volatile uint32_t seq;
    comms_terrain_t   data;
} terrain_slot_t;

#if COMMS_JSON
/* ------------------------------------------------------------------ *
 *  Names used in JSON payloads only (order must match comms.h and
 *  tools/robot_decode.ps1).  Not compiled into the binary firmware.
 * ------------------------------------------------------------------ */

static char const * const g_state_names[COMMS_STATE_COUNT] =
{
    "INIT", "READY", "LINE_FOLLOW", "EXECUTE_NAV", "OBSTACLE_STOP",
    "OBSTACLE_SCAN", "BYPASS", "LINE_RECOVERY", "FINISHED", "SAFE_STOP"
};

static char const * const g_nav_names[COMMS_NAV_COUNT] =
{
    "NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
};

static char const * const g_line_names[COMMS_LINE_COUNT] =
{
    "LEFT", "CENTRE", "RIGHT", "LOST", "JUNCTION"
};

static char const * const g_motion_names[COMMS_MOTION_COUNT] =
{
    "QUIET", "ACCELERATING", "CLIMBING_HUMP", "DESCENDING_HUMP", "IMPACT"
};

static char const * const g_avoid_names[COMMS_AVOID_COUNT] =
{
    "STOP", "CONTINUE", "TURN_LEFT", "TURN_RIGHT", "REVERSE_RETRY"
};

static char const * const g_cmd_names[CMD_CODE_COUNT] =
{
    "unknown", "start", "stop", "status"
};

static char const * const g_run_names[RUN_CODE_COUNT] =
{
    "END", "BEGIN"
};
#endif /* COMMS_JSON */

/* ------------------------------------------------------------------ *
 *  Shared state (written by callers, read by the radio task)
 * ------------------------------------------------------------------ */

static ID                g_event_mbf;
static ID                g_cmd_mbf;
static ID                g_drop_mtx;     /* guards g_events_dropped      */
static volatile bool     g_b_ready;
static volatile bool     g_b_connected;
static volatile uint32_t g_state = (uint32_t) COMMS_STATE_INIT;
static volatile uint32_t g_fault;
static volatile uint32_t g_events_sent;
static volatile uint32_t g_events_dropped;
static volatile uint32_t g_telemetry_sent;
static volatile uint32_t g_commands_received;

static motion_slot_t     g_motion_slot;
static line_slot_t       g_line_slot;
static terrain_slot_t    g_terrain_slot;

static UW g_event_buf[(COMMS_EVENT_QUEUE_LEN
                       * (sizeof(comms_event_t) + COMMS_MBF_OVERHEAD))
                      / sizeof(UW)];
static UW g_cmd_buf[(COMMS_CMD_QUEUE_LEN
                     * (sizeof(uint32_t) + COMMS_MBF_OVERHEAD))
                    / sizeof(UW)];

/* ------------------------------------------------------------------ *
 *  Helpers
 * ------------------------------------------------------------------ */

/* Orders memory accesses between the two RP2040 cores (no caches, so a
   data memory barrier is sufficient) and stops compiler reordering. */
static inline void
memory_barrier (void)
{
    __asm__ volatile ("dmb" ::: "memory");
}


static uint32_t
now_ms (void)
{
    SYSTIM tim;

    (void) tk_get_otm(&tim);
    return (uint32_t) tim.lo;
}


/* Byte copy: <string.h> would pull in <stddef.h>, whose size_t clashes
   with the kernel headers. */
static void
copy_bytes (uint8_t * p_dst, uint8_t const * p_src, uint32_t size)
{
    uint32_t i;

    for (i = 0U; i < size; i++)
    {
        p_dst[i] = p_src[i];
    }
}


/* Seqlock write: seq is odd while the copy is in progress. */
static void
slot_write (volatile uint32_t * p_seq, void * p_dst, void const * p_src,
            uint32_t size)
{
    (*p_seq)++;
    memory_barrier();
    copy_bytes((uint8_t *) p_dst, (uint8_t const *) p_src, size);
    memory_barrier();
    (*p_seq)++;
}


/* Seqlock read: false if the writer was busy or wrote during the copy
   (try again on the next poll).  *p_seq_out is the value's seq. */
static bool
slot_read (volatile uint32_t const * p_seq, void const * p_src,
           void * p_dst, uint32_t size, uint32_t * p_seq_out)
{
    uint32_t const seq = *p_seq;
    bool           b_ok = false;

    memory_barrier();

    if (0U == (seq & 1U))
    {
        copy_bytes((uint8_t *) p_dst, (uint8_t const *) p_src, size);
        memory_barrier();
        *p_seq_out = seq;
        b_ok       = (seq == *p_seq);
    }

    return b_ok;
}


static void
count_drop (void)
{
    if (E_OK == tk_loc_mtx(g_drop_mtx, TMO_FEVR))
    {
        g_events_dropped++;             /* Callers may be on both cores. */
        (void) tk_unl_mtx(g_drop_mtx);
    }
}


static bool
queue_event (comms_event_t * p_event)
{
    bool b_queued = false;

    if (g_b_ready)
    {
        p_event->time_ms = now_ms();

        if (E_OK == tk_snd_mbf(g_event_mbf, p_event,
                               (INT) sizeof(*p_event), TMO_POL))
        {
            b_queued = true;
        }
        else
        {
            count_drop();
        }
    }

    return b_queued;
}

/* ------------------------------------------------------------------ *
 *  Caller side
 * ------------------------------------------------------------------ */

bool
comms_init (void)
{
    T_CMBF cmbf = {0};
    T_CMTX cmtx = {0};

    if (!g_b_ready)
    {
        cmbf.mbfatr = TA_TFIFO;
        cmbf.bufsz  = (SZ) sizeof(g_event_buf);
        cmbf.maxmsz = (INT) sizeof(comms_event_t);
        cmbf.bufptr = g_event_buf;
        g_event_mbf = tk_cre_mbf(&cmbf);

        cmbf.bufsz  = (SZ) sizeof(g_cmd_buf);
        cmbf.maxmsz = (INT) sizeof(uint32_t);
        cmbf.bufptr = g_cmd_buf;
        g_cmd_mbf   = tk_cre_mbf(&cmbf);

        cmtx.mtxatr = TA_TFIFO | TA_INHERIT;
        g_drop_mtx  = tk_cre_mtx(&cmtx);

        g_b_ready = (0 < g_event_mbf) && (0 < g_cmd_mbf) && (0 < g_drop_mtx);
    }

    return g_b_ready;
}


bool
comms_is_connected (void)
{
    return g_b_connected;
}


void
comms_get_stats (comms_stats_t * p_stats)
{
    if (NULL != p_stats)
    {
        p_stats->b_connected       = g_b_connected;
        p_stats->events_sent       = g_events_sent;
        p_stats->events_dropped    = g_events_dropped;
        p_stats->telemetry_sent    = g_telemetry_sent;
        p_stats->commands_received = g_commands_received;
    }
}


void
comms_set_state (comms_state_t state)
{
    if ((uint32_t) state < (uint32_t) COMMS_STATE_COUNT)
    {
        g_state = (uint32_t) state;
    }
}


void
comms_set_fault (uint32_t fault_flags)
{
    g_fault = fault_flags;      /* One 32-bit store: never torn. */
}


bool
comms_begin_run (void)
{
    comms_event_t event = {0};

    event.type = (uint8_t) EVENT_RUN_BEGIN;
    return queue_event(&event);
}


bool
comms_end_run (void)
{
    comms_event_t event = {0};

    event.type = (uint8_t) EVENT_RUN_END;
    return queue_event(&event);
}


bool
comms_get_command (comms_cmd_t * p_cmd)
{
    uint32_t cmd;
    bool     b_got = false;

    if (g_b_ready && (NULL != p_cmd)
        && ((INT) sizeof(cmd) == tk_rcv_mbf(g_cmd_mbf, &cmd, TMO_POL)))
    {
        *p_cmd = (comms_cmd_t) cmd;
        b_got  = true;
    }

    return b_got;
}


void
comms_report_motion (comms_motion_t const * p_motion)
{
    if (NULL != p_motion)
    {
        slot_write(&g_motion_slot.seq, &g_motion_slot.data, p_motion,
                   sizeof(*p_motion));
    }
}


void
comms_report_line (comms_line_t const * p_line)
{
    if (NULL != p_line)
    {
        slot_write(&g_line_slot.seq, &g_line_slot.data, p_line,
                   sizeof(*p_line));
    }
}


void
comms_report_terrain (comms_terrain_t const * p_terrain)
{
    if (NULL != p_terrain)
    {
        slot_write(&g_terrain_slot.seq, &g_terrain_slot.data, p_terrain,
                   sizeof(*p_terrain));
    }
}


bool
comms_event_barcode (char code, comms_nav_t command)
{
    comms_event_t event = {0};

    event.type = (uint8_t) EVENT_BARCODE;
    event.code = (uint8_t) code;
    event.kind = (uint8_t) command;
    return queue_event(&event);
}


bool
comms_event_hump (int32_t peak_dmm)
{
    comms_event_t event = {0};

    event.type  = (uint8_t) EVENT_HUMP;
    event.value = peak_dmm;
    return queue_event(&event);
}


bool
comms_event_obstacle (comms_obstacle_t const * p_obstacle)
{
    comms_event_t event = {0};
    bool          b_queued = false;

    if (NULL != p_obstacle)
    {
        event.type     = (uint8_t) EVENT_OBSTACLE;
        event.obstacle = *p_obstacle;
        b_queued       = queue_event(&event);
    }

    return b_queued;
}


bool
comms_event_motion (comms_motion_class_t motion)
{
    comms_event_t event = {0};

    event.type = (uint8_t) EVENT_MOTION;
    event.kind = (uint8_t) motion;
    return queue_event(&event);
}

/* ------------------------------------------------------------------ *
 *  Radio side: everything below runs only in the radio task
 * ------------------------------------------------------------------ */

typedef struct
{
    uint32_t last_seq;      /* newest seq already looked at           */
    uint32_t last_hash;     /* content of the value last published    */
    uint32_t last_sent_ms;
    bool     b_force;       /* resend even if unchanged               */
} snapshot_tx_t;

typedef struct
{
    uint8_t cmd;            /* CMD_CODE_*                             */
    bool    b_ok;
} ack_t;

static snapshot_tx_t g_motion_tx;
static snapshot_tx_t g_line_tx;
static snapshot_tx_t g_terrain_tx;

static uint32_t g_published_state = 0xFFFFFFFFU;
static uint32_t g_published_fault = 0xFFFFFFFFU;
static uint32_t g_heartbeat_ms;
static uint32_t g_heartbeat_seq;
static bool     g_b_info_pending;

/* Acknowledgements of commands received, oldest first. */
static ack_t    g_acks[COMMS_ACK_QUEUE_LEN];
static uint32_t g_ack_head;
static uint32_t g_ack_count;

/* Run summary, built from the events as they are published. */
static bool     g_b_run_active;
static uint32_t g_run_start_ms;
static int32_t  g_run_start_mm;
static int32_t  g_last_distance_mm;
static int32_t  g_max_hump_dmm;
static uint32_t g_hump_count;
static uint32_t g_barcode_count;
static uint32_t g_obstacle_count;

/* --- small helpers --- */

/* Negative distances/sizes mean "invalid": send 0. */
static uint32_t
non_negative (int32_t value)
{
    return (0 > value) ? 0U : (uint32_t) value;
}


/* Barcode character -> 3-bit code.  Covers both the design's A-D and the
 * course's A, F, Z until the team settles which set is used:
 * 0 NONE, 1 A, 2 B, 3 C, 4 D, 5 F, 6 Z, 7 INVALID. */
static uint32_t
barcode_code (char code)
{
    static char const g_letters[] = "ABCDFZ";
    uint32_t          result = 7U;
    uint32_t          i;

    if ('\0' == code)
    {
        result = 0U;
    }
    else
    {
        for (i = 0U; (7U == result) && ('\0' != g_letters[i]); i++)
        {
            if (g_letters[i] == code)
            {
                result = i + 1U;
            }
        }
    }

    return result;
}


/* A new session, or the laptop asked for "status": resend everything. */
static void
resend_all (void)
{
    g_b_info_pending     = true;
    g_published_state    = 0xFFFFFFFFU;
    g_motion_tx.b_force  = true;
    g_line_tx.b_force    = true;
    g_terrain_tx.b_force = true;
}


static void
ack_push (uint32_t cmd, bool b_ok)
{
    uint32_t slot;

    if (COMMS_ACK_QUEUE_LEN > g_ack_count)
    {
        slot = (g_ack_head + g_ack_count) % COMMS_ACK_QUEUE_LEN;
        g_acks[slot].cmd  = (uint8_t) cmd;
        g_acks[slot].b_ok = b_ok;
        g_ack_count++;
    }
}


static bool
ack_pop (ack_t * p_ack)
{
    bool b_got = false;

    if (0U < g_ack_count)
    {
        *p_ack      = g_acks[g_ack_head];
        g_ack_head  = (g_ack_head + 1U) % COMMS_ACK_QUEUE_LEN;
        g_ack_count--;
        b_got       = true;
    }

    return b_got;
}


/* FNV-1a over the field values: a cheap fingerprint of a snapshot. */
static uint32_t
hash_fields (int32_t const * p_fields, uint32_t count)
{
    uint8_t const * p_byte = (uint8_t const *) p_fields;
    uint32_t        hash   = 2166136261U;
    uint32_t        i;

    for (i = 0U; i < (count * (uint32_t) sizeof(int32_t)); i++)
    {
        hash ^= p_byte[i];
        hash *= 16777619U;
    }

    return hash;
}


/* A snapshot is due when it has ever been written, the per-topic rate
   limit has passed, and either a resend was asked for, the refresh
   interval passed (late subscribers catch up), or its content changed.
   An unchanged seq means nothing new was written: no hashing needed. */
static bool
snapshot_due (snapshot_tx_t * p_tx, uint32_t seq, int32_t const * p_fields,
              uint32_t count, uint32_t now, uint32_t * p_hash)
{
    uint32_t const age   = now - p_tx->last_sent_ms;
    bool           b_due = false;

    *p_hash = p_tx->last_hash;

    if ((0U != seq) && (age >= COMMS_TELEMETRY_MS))
    {
        if (p_tx->b_force || (age >= COMMS_REFRESH_MS))
        {
            *p_hash = hash_fields(p_fields, count);
            b_due   = true;
        }
        else if (seq != p_tx->last_seq)
        {
            *p_hash        = hash_fields(p_fields, count);
            b_due          = (*p_hash != p_tx->last_hash);
            p_tx->last_seq = seq;       /* Looked at; don't hash again. */
        }
        else
        {
            /* Nothing new since the last look. */
        }
    }

    return b_due;
}


static void
snapshot_sent (snapshot_tx_t * p_tx, uint32_t seq, uint32_t hash,
               uint32_t now)
{
    p_tx->last_seq     = seq;
    p_tx->last_hash    = hash;
    p_tx->last_sent_ms = now;
    p_tx->b_force      = false;
    g_telemetry_sent++;
}

/* --- message producers: each message described once --- */

static void
build_event (comms_link_msg_t * p_msg, comms_event_t const * p_event)
{
    codec_writer_t                 w;
    comms_obstacle_t const * const p_obs = &p_event->obstacle;
    char                           code[2];

    switch ((event_type_t) p_event->type)
    {
        case EVENT_BARCODE:
            code[0] = (char) p_event->code;
            code[1] = '\0';
            g_barcode_count++;
            codec_begin(&w, p_msg, "robot/event/barcode", QOS_1, false,
                        p_event->time_ms);
            codec_put_symbol(&w, CODEC_KEY("code"), code,
                             barcode_code(code[0]), BITS_BARCODE);
            codec_put_name(&w, CODEC_KEY("command"),
                           CODEC_NAMES(g_nav_names), COMMS_NAV_COUNT,
                           p_event->kind, BITS_NAV);
            break;

        case EVENT_HUMP:
            g_hump_count++;

            if (p_event->value > g_max_hump_dmm)
            {
                g_max_hump_dmm = p_event->value;
            }

            codec_begin(&w, p_msg, "robot/event/hump", QOS_1, false,
                        p_event->time_ms);
            codec_put_uint(&w, CODEC_KEY("peak_dmm"),
                           non_negative(p_event->value), BITS_HUMP);
            codec_put_uint(&w, CODEC_KEY("max_dmm"),
                           non_negative(g_max_hump_dmm), BITS_HUMP);
            break;

        case EVENT_OBSTACLE:
            g_obstacle_count++;
            codec_begin(&w, p_msg, "robot/event/obstacle", QOS_1, false,
                        p_event->time_ms);
            codec_put_uint(&w, CODEC_KEY("dist_mm"),
                           non_negative(p_obs->distance_mm), BITS_OBST_DIST);
            codec_put_uint(&w, CODEC_KEY("angle_deg"),
                           non_negative(p_obs->angle_deg), BITS_ANGLE);
            codec_put_uint(&w, CODEC_KEY("width_mm"),
                           non_negative(p_obs->width_mm), BITS_WIDTH);
            codec_put_uint(&w, CODEC_KEY("clear_l_mm"),
                           non_negative(p_obs->clear_left_mm),
                           BITS_CLEARANCE);
            codec_put_uint(&w, CODEC_KEY("clear_r_mm"),
                           non_negative(p_obs->clear_right_mm),
                           BITS_CLEARANCE);
            codec_put_name(&w, CODEC_KEY("action"),
                           CODEC_NAMES(g_avoid_names), COMMS_AVOID_COUNT,
                           (uint32_t) p_obs->action, BITS_ACTION);
            break;

        case EVENT_MOTION:
            codec_begin(&w, p_msg, "robot/event/motion", QOS_1, false,
                        p_event->time_ms);
            codec_put_name(&w, CODEC_KEY("motion"),
                           CODEC_NAMES(g_motion_names), COMMS_MOTION_COUNT,
                           p_event->kind, BITS_MOTION_CLASS);
            break;

        case EVENT_RUN_BEGIN:
            g_b_run_active   = true;
            g_run_start_ms   = p_event->time_ms;
            g_run_start_mm   = g_last_distance_mm;
            g_max_hump_dmm   = 0;
            g_hump_count     = 0U;
            g_barcode_count  = 0U;
            g_obstacle_count = 0U;
            codec_begin(&w, p_msg, "robot/event/run", QOS_1, false,
                        p_event->time_ms);
            codec_put_name(&w, CODEC_KEY("run"), CODEC_NAMES(g_run_names),
                           RUN_CODE_COUNT, RUN_BEGIN, BITS_RUN);
            break;

        case EVENT_RUN_END:
        default:
            /* Retained, so a dashboard that connects later still sees
               the result of the last run. */
            codec_begin(&w, p_msg, "robot/summary", QOS_1, true,
                        p_event->time_ms);
            codec_put_uint(&w, CODEC_KEY("run_ms"), g_b_run_active
                           ? (p_event->time_ms - g_run_start_ms) : 0U,
                           BITS_RUN_MS);
            codec_put_int(&w, CODEC_KEY("dist_mm"),
                          g_last_distance_mm - g_run_start_mm, BITS_DISTANCE);
            codec_put_uint(&w, CODEC_KEY("max_hump_dmm"),
                           non_negative(g_max_hump_dmm), BITS_HUMP);
            codec_put_uint(&w, CODEC_KEY("humps"), g_hump_count, BITS_COUNT);
            codec_put_uint(&w, CODEC_KEY("barcodes"), g_barcode_count,
                           BITS_COUNT);
            codec_put_uint(&w, CODEC_KEY("obstacles"), g_obstacle_count,
                           BITS_COUNT);
            g_b_run_active = false;
            break;
    }

    codec_end(&w);
    g_events_sent++;
}


static bool
next_motion (comms_link_msg_t * p_msg, uint32_t now)
{
    comms_motion_t motion;
    codec_writer_t w;
    int32_t        fields[5];
    uint32_t       seq;
    uint32_t       hash;
    bool           b_built = false;

    if (slot_read(&g_motion_slot.seq, &g_motion_slot.data, &motion,
                  sizeof(motion), &seq))
    {
        g_last_distance_mm = motion.distance_mm;
        fields[0] = motion.speed_left_mm_s;
        fields[1] = motion.speed_right_mm_s;
        fields[2] = motion.encoder_left;
        fields[3] = motion.encoder_right;
        fields[4] = motion.distance_mm;

        if (snapshot_due(&g_motion_tx, seq, fields, 5U, now, &hash))
        {
            codec_begin(&w, p_msg, "robot/telemetry/motion", QOS_0, false,
                        now);
            codec_put_int(&w, CODEC_KEY("spd_l_mm_s"), fields[0], BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("spd_r_mm_s"), fields[1], BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("enc_l"), fields[2], BITS_ENCODER);
            codec_put_int(&w, CODEC_KEY("enc_r"), fields[3], BITS_ENCODER);
            codec_put_int(&w, CODEC_KEY("dist_mm"), fields[4], BITS_DISTANCE);
            codec_end(&w);
            snapshot_sent(&g_motion_tx, seq, hash, now);
            b_built = true;
        }
    }

    return b_built;
}


static bool
next_line (comms_link_msg_t * p_msg, uint32_t now)
{
    comms_line_t   line;
    codec_writer_t w;
    int32_t        fields[3];
    uint32_t       seq;
    uint32_t       hash;
    bool           b_built = false;

    if (slot_read(&g_line_slot.seq, &g_line_slot.data, &line,
                  sizeof(line), &seq))
    {
        fields[0] = (int32_t) line.position;
        fields[1] = line.error;
        fields[2] = (int32_t) line.sensors;

        if (snapshot_due(&g_line_tx, seq, fields, 3U, now, &hash))
        {
            codec_begin(&w, p_msg, "robot/telemetry/line", QOS_0, false,
                        now);
            codec_put_name(&w, CODEC_KEY("pos"), CODEC_NAMES(g_line_names),
                           COMMS_LINE_COUNT, (uint32_t) line.position,
                           BITS_LINE_POS);
            codec_put_int(&w, CODEC_KEY("err"), line.error, BITS_LINE_ERROR);
            codec_put_uint(&w, CODEC_KEY("sensors"), line.sensors,
                           BITS_SENSORS);
            codec_end(&w);
            snapshot_sent(&g_line_tx, seq, hash, now);
            b_built = true;
        }
    }

    return b_built;
}


static bool
next_terrain (comms_link_msg_t * p_msg, uint32_t now)
{
    comms_terrain_t terrain;
    codec_writer_t  w;
    int32_t         fields[4];
    uint32_t        seq;
    uint32_t        hash;
    bool            b_built = false;

    if (slot_read(&g_terrain_slot.seq, &g_terrain_slot.data, &terrain,
                  sizeof(terrain), &seq))
    {
        fields[0] = (int32_t) terrain.motion;
        fields[1] = terrain.tilt_mg;
        fields[2] = terrain.hump_dmm;
        fields[3] = g_max_hump_dmm;

        if (snapshot_due(&g_terrain_tx, seq, fields, 4U, now, &hash))
        {
            codec_begin(&w, p_msg, "robot/telemetry/terrain", QOS_0, false,
                        now);
            codec_put_name(&w, CODEC_KEY("motion"),
                           CODEC_NAMES(g_motion_names), COMMS_MOTION_COUNT,
                           (uint32_t) terrain.motion, BITS_MOTION_CLASS);
            codec_put_int(&w, CODEC_KEY("tilt_mg"), terrain.tilt_mg,
                          BITS_TILT);
            codec_put_uint(&w, CODEC_KEY("hump_dmm"),
                           non_negative(terrain.hump_dmm), BITS_HUMP);
            codec_put_uint(&w, CODEC_KEY("max_hump_dmm"),
                           non_negative(g_max_hump_dmm), BITS_HUMP);
            codec_end(&w);
            snapshot_sent(&g_terrain_tx, seq, hash, now);
            b_built = true;
        }
    }

    return b_built;
}


bool
comms_link_next (comms_link_msg_t * p_msg)
{
    comms_event_t  event;
    codec_writer_t w;
    ack_t          ack;
    uint32_t       now;
    uint32_t       state;
    uint32_t       fault;
    bool           b_built = false;

    if (g_b_ready && (NULL != p_msg))
    {
        now     = now_ms();
        state   = g_state;
        fault   = g_fault & FAULT_MASK;
        b_built = true;

        if (ack_pop(&ack))
        {
            /* 1. Command acknowledgement: the sender is waiting for it. */
            codec_begin(&w, p_msg, "robot/event/command", QOS_1, false,
                        now);
            codec_put_name(&w, CODEC_KEY("cmd"), CODEC_NAMES(g_cmd_names),
                           CMD_CODE_COUNT, ack.cmd, BITS_CMD);
            codec_put_uint(&w, CODEC_KEY("ok"), ack.b_ok ? 1U : 0U, BITS_OK);
            codec_end(&w);
        }
        else if (g_b_info_pending)
        {
            /* Which firmware is this?  Once per session, retained, plain
               text: the build name IS the information. */
            g_b_info_pending = false;
            codec_text(p_msg, "robot/info", QOS_1, true, g_build_id);
        }
        else if ((state != g_published_state)
                 || (fault != g_published_fault))
        {
            /* 2. Robot state, retained, whenever it or the faults change. */
            g_published_state = state;
            g_published_fault = fault;
            codec_begin(&w, p_msg, "robot/state", QOS_1, true, now);
            codec_put_name(&w, CODEC_KEY("state"), CODEC_NAMES(g_state_names),
                           COMMS_STATE_COUNT, state, BITS_STATE);
            codec_put_uint(&w, CODEC_KEY("fault"), fault, BITS_FAULT);
            codec_end(&w);
        }
        else if ((INT) sizeof(event)
                 == tk_rcv_mbf(g_event_mbf, &event, TMO_POL))
        {
            /* 3. Queued events, oldest first. */
            build_event(p_msg, &event);
        }
        else if ((now - g_heartbeat_ms) >= COMMS_HEARTBEAT_MS)
        {
            /* 4. Heartbeat. */
            g_heartbeat_ms = now;
            g_heartbeat_seq++;
            codec_begin(&w, p_msg, "robot/heartbeat", QOS_0, false, now);
            codec_put_uint(&w, CODEC_KEY("seq"), g_heartbeat_seq & SEQ_MASK,
                           BITS_SEQ);
            codec_put_name(&w, CODEC_KEY("state"), CODEC_NAMES(g_state_names),
                           COMMS_STATE_COUNT, state, BITS_STATE);
            codec_put_uint(&w, CODEC_KEY("fault"), fault, BITS_FAULT);
            codec_put_uint(&w, CODEC_KEY("ev_drop"), g_events_dropped,
                           BITS_DROPS);
            codec_end(&w);
        }
        else
        {
            /* 5. Telemetry snapshots, in turn. */
            b_built = next_motion(p_msg, now) || next_line(p_msg, now)
                      || next_terrain(p_msg, now);
        }
    }

    return b_built;
}


void
comms_link_set_connected (bool b_connected)
{
    g_b_connected = b_connected;

    if (b_connected)
    {
        resend_all();           /* Fresh session: the current picture. */
    }
}


static bool
payload_is (uint8_t const * p_data, uint32_t len, char const * p_word)
{
    uint32_t i = 0U;

    while ((i < len) && ('\0' != p_word[i])
           && (p_data[i] == (uint8_t) p_word[i]))
    {
        i++;
    }

    return (i == len) && ('\0' == p_word[i]);
}


/* Text ("start") or 1-byte binary (0x01) command -> CMD_CODE_*. */
static uint32_t
command_code (uint8_t const * p_data, uint32_t len)
{
    uint32_t code = CMD_CODE_UNKNOWN;
    uint32_t n    = len;

    /* Ignore trailing whitespace, e.g. a newline from a terminal. */
    while ((0U < n) && (((uint8_t) '\n' == p_data[n - 1U])
                        || ((uint8_t) '\r' == p_data[n - 1U])
                        || ((uint8_t) ' ' == p_data[n - 1U])))
    {
        n--;
    }

    if ((1U == n) && (CMD_CODE_START <= p_data[0])
        && (CMD_CODE_STATUS >= p_data[0]))
    {
        code = p_data[0];                       /* Binary form. */
    }
    else if (payload_is(p_data, n, "start"))
    {
        code = CMD_CODE_START;
    }
    else if (payload_is(p_data, n, "stop"))
    {
        code = CMD_CODE_STOP;
    }
    else if (payload_is(p_data, n, "status"))
    {
        code = CMD_CODE_STATUS;
    }
    else
    {
        /* Unknown: acknowledged as not ok. */
    }

    return code;
}


void
comms_link_on_command (uint8_t const * p_data, uint32_t len)
{
    uint32_t const code = command_code(p_data, len);
    uint32_t       cmd  = (uint32_t) COMMS_CMD_NONE;
    bool           b_ok = false;

    g_commands_received++;

    switch (code)
    {
        case CMD_CODE_START:
            cmd = (uint32_t) COMMS_CMD_START;
            break;

        case CMD_CODE_STOP:
            cmd = (uint32_t) COMMS_CMD_STOP;
            break;

        case CMD_CODE_STATUS:
            resend_all();               /* Handled here, at once. */
            b_ok = true;
            break;

        default:
            break;
    }

    if ((uint32_t) COMMS_CMD_NONE != cmd)
    {
        /* Queue full means the controller is not reading commands. */
        b_ok = g_b_ready
               && (E_OK == tk_snd_mbf(g_cmd_mbf, &cmd, (INT) sizeof(cmd),
                                      TMO_POL));
    }

    ack_push(code, b_ok);
}
