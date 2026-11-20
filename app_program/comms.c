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

#include "barcode.h"
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
#define COMMS_TELEMETRY_RUN_MS   100U   /* motion and line during a run  */
#define COMMS_REFRESH_MS         2000U  /* resend unchanged values       */
#define COMMS_HEARTBEAT_MS       1000U
#define COMMS_MBF_OVERHEAD       8U     /* per-message kernel header     */

/* TEMPORARY (barcode diagnosis, 2026-11-04): robot/debug text lines.
 * Set to 0, or delete every COMMS_DEBUG_TEXT block, to remove. */
#define COMMS_DEBUG_TEXT         1
#define COMMS_DEBUG_QUEUE_LEN    4U
#define COMMS_DEBUG_TEXT_MAX     200U   /* below COMMS_LINK_PAYLOAD_MAX  */

#define QOS_0                    0U
#define QOS_1                    1U

/* ------------------------------------------------------------------ *
 *  Field widths in bits (docs/BINARY_FORMAT.md).  Values outside a
 *  range are clamped.  A width that the packed records also use comes
 *  from comms_codec.h, so the wire format and the stored format cannot
 *  drift apart.
 * ------------------------------------------------------------------ */

#define BITS_SPEED               CODEC_BITS_SPEED   /* signed           */
#define BITS_DRIVE               CODEC_BITS_DRIVE   /* signed, permille */
#define BITS_ENCODER             CODEC_BITS_ENCODER /* signed, counts   */
#define BITS_DISTANCE            CODEC_BITS_DISTANCE /* signed, mm      */
#define BITS_LINE_POS            CODEC_BITS_LINE_POS
#define BITS_LINE_ERROR          CODEC_BITS_LINE_ERROR     /* signed    */
#define BITS_SENSORS             CODEC_BITS_SENSORS
#define BITS_MOTION_CLASS        CODEC_BITS_MOTION_CLASS
#define BITS_TILT                CODEC_BITS_TILT     /* signed, mg      */
#define BITS_HUMP                CODEC_BITS_HUMP     /* 0.1 mm          */
#define BITS_TURN                CODEC_BITS_TURN     /* signed, deg/s   */
#define BITS_SEQ                 16U
#define BITS_STATE               4U
#define BITS_FAULT               10U    /* fault bits 0-9, see FAULT_MASK */
#define BITS_DROPS               8U
#define BITS_BARCODE             CODEC_BITS_BARCODE
#define BITS_NAV                 CODEC_BITS_NAV
#define BITS_OBST_DIST           CODEC_BITS_OBST_DIST
#define BITS_ANGLE               CODEC_BITS_ANGLE
#define BITS_WIDTH               CODEC_BITS_WIDTH
#define BITS_CLEARANCE           CODEC_BITS_CLEARANCE
#define BITS_ACTION              CODEC_BITS_ACTION
#define BITS_RANGE               CODEC_BITS_RANGE
#define BITS_SONAR_STATE         CODEC_BITS_SONAR_STATE
#define BITS_SILENT              CODEC_BITS_SILENT
#define BITS_RUN                 1U
#define BITS_RUN_MS              20U
#define BITS_COUNT               8U
#define BITS_CMD                 2U
#define BITS_OK                  1U
#define BITS_CAL_RESULT          CODEC_BITS_CAL_RESULT
#define BITS_CAL_OVERLAP         CODEC_BITS_CAL_OVERLAP
#define BITS_ADC                 CODEC_BITS_ADC
#define BITS_CAL_LEARNED         CODEC_BITS_CAL_LEARNED
#define BITS_CAL_FIRST_RIGHT     CODEC_BITS_CAL_FIRST_RIGHT
#define BITS_CAL_FLOOR           CODEC_BITS_CAL_FLOOR
#define BITS_CAL_SAMPLES         CODEC_BITS_CAL_SAMPLES
#define BITS_CAL_ANGLE           CODEC_BITS_CAL_ANGLE

/* Fault levels and counter limits. */
#define FAULT_MASK               0x3FFU    /* fault bits sent: 0-9       */
#define EVENT_DROPS_MAX          255U      /* 8-bit wire, saturates      */
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

/* Seqlock-protected latest value, stored packed.  One writer task, one
 * reader (the radio task); each word is written with one aligned store. */
typedef struct
{
    volatile uint32_t seq;          /* odd while the writer is copying   */
    uint32_t          words[CODEC_MOTION_WORDS];
} motion_slot_t;

typedef struct
{
    volatile uint32_t seq;
    uint32_t          words[CODEC_LINE_WORDS];
} line_slot_t;

typedef struct
{
    volatile uint32_t seq;
    uint32_t          words[CODEC_TERRAIN_WORDS];
} terrain_slot_t;

typedef struct
{
    volatile uint32_t seq;
    uint32_t          words[CODEC_SONAR_WORDS];
} sonar_slot_t;

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
    "QUIET", "ACCELERATING", "CLIMBING_HUMP", "DESCENDING_HUMP", "LIFTED",
    "TURNING"
};

static char const * const g_avoid_names[COMMS_AVOID_COUNT] =
{
    "STOP", "CONTINUE", "TURN_LEFT", "TURN_RIGHT", "REVERSE_RETRY"
};

static char const * const g_range_names[COMMS_RANGE_COUNT] =
{
    "OK", "NO_ECHO", "TOO_CLOSE", "TOO_FAR", "TIMEOUT", "NOT_READY"
};

static char const * const g_sonar_names[COMMS_SONAR_COUNT] =
{
    "INIT", "MONITOR", "SCANNING", "SIDE_WATCH", "LINE_RECOVERY",
    "SENSOR_FAULT"
};

static char const * const g_cmd_names[CMD_CODE_COUNT] =
{
    "unknown", "start", "stop", "status"
};

static char const * const g_run_names[RUN_CODE_COUNT] =
{
    "END", "BEGIN"
};

static char const * const g_cal_names[COMMS_CAL_COUNT] =
{
    "OK", "CONTRAST", "LINE_NOT_FOUND", "BUDDY4", "SENSOR_GAP"
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
static volatile uint16_t g_fault;           /* bits 0-9 (FAULT_MASK)      */
static volatile uint32_t g_events_sent;
static volatile uint8_t  g_events_dropped;  /* saturates, 8-bit wire      */
static volatile uint32_t g_telemetry_sent;
static volatile uint32_t g_commands_received;

static motion_slot_t     g_motion_slot;
static line_slot_t       g_line_slot;
static terrain_slot_t    g_terrain_slot;
static sonar_slot_t      g_sonar_slot;

static UW g_event_buf[(COMMS_EVENT_QUEUE_LEN
                       * ((CODEC_EVENT_WORDS * sizeof(uint32_t))
                          + COMMS_MBF_OVERHEAD))
                      / sizeof(UW)];
static UW g_cmd_buf[(COMMS_CMD_QUEUE_LEN
                     * (sizeof(uint32_t) + COMMS_MBF_OVERHEAD))
                    / sizeof(UW)];

#if COMMS_DEBUG_TEXT
static ID g_debug_mbf;
static UW g_debug_buf[(COMMS_DEBUG_QUEUE_LEN
                       * (COMMS_DEBUG_TEXT_MAX + COMMS_MBF_OVERHEAD))
                      / sizeof(UW)];
#endif

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


/* One event dropped: saturates at the 8-bit wire width, so the heartbeat
 * counter cannot wrap.  Callers may be on both cores. */
static void
count_drop (void)
{
    if (E_OK == tk_loc_mtx(g_drop_mtx, TMO_FEVR))
    {
        if (EVENT_DROPS_MAX > g_events_dropped)
        {
            g_events_dropped++;
        }

        (void) tk_unl_mtx(g_drop_mtx);
    }
}


static bool
queue_event (codec_event_t * p_event)
{
    uint32_t words[CODEC_EVENT_WORDS];
    bool     b_queued = false;

    if (g_b_ready)
    {
        p_event->time_ms = now_ms();
        codec_pack_event(words, p_event);

        if (E_OK == tk_snd_mbf(g_event_mbf, words, (INT) sizeof(words),
                               TMO_POL))
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
        cmbf.maxmsz = (INT) (CODEC_EVENT_WORDS * sizeof(uint32_t));
        cmbf.bufptr = g_event_buf;
        g_event_mbf = tk_cre_mbf(&cmbf);

        cmbf.bufsz  = (SZ) sizeof(g_cmd_buf);
        cmbf.maxmsz = (INT) sizeof(uint32_t);
        cmbf.bufptr = g_cmd_buf;
        g_cmd_mbf   = tk_cre_mbf(&cmbf);

        cmtx.mtxatr = TA_TFIFO | TA_INHERIT;
        g_drop_mtx  = tk_cre_mtx(&cmtx);

        g_b_ready = (0 < g_event_mbf) && (0 < g_cmd_mbf) && (0 < g_drop_mtx);

#if COMMS_DEBUG_TEXT
        /* Optional: a failure here only disables robot/debug. */
        cmbf.bufsz  = (SZ) sizeof(g_debug_buf);
        cmbf.maxmsz = (INT) COMMS_DEBUG_TEXT_MAX;
        cmbf.bufptr = g_debug_buf;
        g_debug_mbf = tk_cre_mbf(&cmbf);
#endif
    }

    return g_b_ready;
}


#if COMMS_DEBUG_TEXT
void
comms_debug_text (char const * p_text)
{
    char     text[COMMS_DEBUG_TEXT_MAX];
    uint32_t n = 0U;

    if (g_b_ready && (0 < g_debug_mbf) && (NULL != p_text))
    {
        while ((n < (COMMS_DEBUG_TEXT_MAX - 1U)) && ('\0' != p_text[n]))
        {
            text[n] = p_text[n];
            n++;
        }

        text[n] = '\0';
        (void) tk_snd_mbf(g_debug_mbf, text, (INT) (n + 1U), TMO_POL);
    }
}
#else
void
comms_debug_text (char const * p_text)
{
    (void) p_text;
}
#endif


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
    /* Fault bits 0-9 (FAULT_MASK); bit 4 is reserved.  Names and rules:
       VC_FAULT_* in vc_task.h (docs/DATA_REPRESENTATION.md section 9).
       One aligned 16-bit store by the single writer: never torn. */
    g_fault = (uint16_t) (fault_flags & FAULT_MASK);
}


bool
comms_begin_run (void)
{
    codec_event_t event = {0};

    event.type = (uint8_t) CODEC_EVENT_RUN_BEGIN;
    return queue_event(&event);
}


bool
comms_end_run (void)
{
    codec_event_t event = {0};

    event.type = (uint8_t) CODEC_EVENT_RUN_END;
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
    uint32_t words[CODEC_MOTION_WORDS];

    if (NULL != p_motion)
    {
        codec_pack_motion(words, p_motion);
        slot_write(&g_motion_slot.seq, g_motion_slot.words, words,
                   (uint32_t) sizeof(words));
    }
}


void
comms_report_line (comms_line_t const * p_line)
{
    uint32_t words[CODEC_LINE_WORDS];

    if (NULL != p_line)
    {
        codec_pack_line(words, p_line);
        slot_write(&g_line_slot.seq, g_line_slot.words, words,
                   (uint32_t) sizeof(words));
    }
}


void
comms_report_terrain (comms_terrain_t const * p_terrain)
{
    uint32_t words[CODEC_TERRAIN_WORDS];

    if (NULL != p_terrain)
    {
        codec_pack_terrain(words, p_terrain);
        slot_write(&g_terrain_slot.seq, g_terrain_slot.words, words,
                   (uint32_t) sizeof(words));
    }
}


void
comms_report_sonar (comms_sonar_t const * p_sonar)
{
    uint32_t words[CODEC_SONAR_WORDS];

    if (NULL != p_sonar)
    {
        codec_pack_sonar(words, p_sonar);
        slot_write(&g_sonar_slot.seq, g_sonar_slot.words, words,
                   (uint32_t) sizeof(words));
    }
}


bool
comms_event_barcode (char code, comms_nav_t command)
{
    codec_event_t event = {0};

    event.type = (uint8_t) CODEC_EVENT_BARCODE;
    event.code = (uint8_t) code;
    event.kind = (uint8_t) command;
    return queue_event(&event);
}


bool
comms_event_hump (int32_t peak_dmm)
{
    codec_event_t event = {0};

    event.type  = (uint8_t) CODEC_EVENT_HUMP;
    event.value = peak_dmm;
    return queue_event(&event);
}


bool
comms_event_obstacle (comms_obstacle_t const * p_obstacle)
{
    codec_event_t event = {0};
    bool          b_queued = false;

    if (NULL != p_obstacle)
    {
        event.type     = (uint8_t) CODEC_EVENT_OBSTACLE;
        event.obstacle = *p_obstacle;
        b_queued       = queue_event(&event);
    }

    return b_queued;
}


bool
comms_event_motion (comms_motion_class_t motion)
{
    codec_event_t event = {0};

    event.type = (uint8_t) CODEC_EVENT_MOTION;
    event.kind = (uint8_t) motion;
    return queue_event(&event);
}

bool
comms_event_calibration (comms_calibration_t const * p_calibration)
{
    codec_event_t event = {0};
    bool          b_queued = false;

    if (NULL != p_calibration)
    {
        event.type        = (uint8_t) CODEC_EVENT_CALIBRATION;
        event.calibration = *p_calibration;
        b_queued          = queue_event(&event);
    }

    return b_queued;
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
static snapshot_tx_t g_sonar_tx;

static uint32_t g_published_state = 0xFFFFFFFFU;
static uint32_t g_published_fault = 0xFFFFFFFFU;
static uint32_t g_heartbeat_ms;
static uint16_t g_heartbeat_seq;        /* wraps at the 16-bit wire width */
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

#if COMMS_DEBUG_TEXT
static char     g_debug_rx[COMMS_DEBUG_TEXT_MAX];   /* Radio task only. */
#endif

/* --- small helpers --- */

/* Negative distances/sizes mean "invalid": send 0. */
static uint32_t
non_negative (int32_t value)
{
    return (0 > value) ? 0U : (uint32_t) value;
}


/* Barcode character -> 6-bit code: 0 NONE, 1..43 Code 39 in the decoder's
 * table order ('0'..'9', 'A'..'Z', '-', '.', ' ', '$', '/', '+', '%'),
 * 44 INVALID (barcode.c owns the table; the laptop lists the same 45
 * names).  '?' is the failed-decode character vc_task sends. */
static uint32_t
barcode_code (char code)
{
    return (uint32_t) barcode_char_index(code);
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
    g_sonar_tx.b_force   = true;
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


/* FNV-1a over the packed words: a cheap fingerprint of a snapshot.  The
 * slot already holds the clamped values, so this hashes exactly what would
 * be published. */
static uint32_t
hash_words (uint32_t const * p_words, uint32_t count)
{
    uint8_t const * p_byte = (uint8_t const *) p_words;
    uint32_t        hash   = 2166136261U;
    uint32_t        i;

    for (i = 0U; i < (count * (uint32_t) sizeof(uint32_t)); i++)
    {
        hash ^= p_byte[i];
        hash *= 16777619U;
    }

    return hash;
}


/* Motion and line go twice as often while a run is active, so a run's
   steering can be followed; terrain and everything outside a run keep the
   slower rate. */
static uint32_t
run_period_ms (void)
{
    return g_b_run_active ? COMMS_TELEMETRY_RUN_MS : COMMS_TELEMETRY_MS;
}


/* A snapshot is due when it has ever been written, the per-topic rate
   limit has passed, and either a resend was asked for, the refresh
   interval passed (late subscribers catch up), or its content changed.
   An unchanged seq means nothing new was written: no hashing needed. */
static bool
snapshot_due (snapshot_tx_t * p_tx, uint32_t seq, uint32_t const * p_words,
              uint32_t count, uint32_t now, uint32_t period_ms,
              uint32_t * p_hash)
{
    uint32_t const age   = now - p_tx->last_sent_ms;
    bool           b_due = false;

    *p_hash = p_tx->last_hash;

    if ((0U != seq) && (age >= period_ms))
    {
        if (p_tx->b_force || (age >= COMMS_REFRESH_MS))
        {
            *p_hash = hash_words(p_words, count);
            b_due   = true;
        }
        else if (seq != p_tx->last_seq)
        {
            *p_hash        = hash_words(p_words, count);
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
build_calibration (codec_writer_t * p_w, comms_link_msg_t * p_msg,
                   codec_event_t const * p_event)
{
    comms_calibration_t const * const p_cal = &p_event->calibration;

    codec_begin(p_w, p_msg, "robot/event/calibration", QOS_1, false,
                p_event->time_ms);
    codec_put_name(p_w, CODEC_KEY("result"), CODEC_NAMES(g_cal_names),
                   COMMS_CAL_COUNT, (uint32_t) p_cal->result,
                   BITS_CAL_RESULT);
    codec_put_uint(p_w, CODEC_KEY("centre_low"), p_cal->centre_low, BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("centre_high"), p_cal->centre_high,
                   BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("centre_thr"), p_cal->centre_threshold,
                   BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("centre_full"), p_cal->centre_full_black,
                   BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("right_low"), p_cal->right_low, BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("right_high"), p_cal->right_high, BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("right_thr"), p_cal->right_threshold,
                   BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("barcode_low"), p_cal->barcode_low,
                   BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("barcode_high"), p_cal->barcode_high,
                   BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("barcode_thr"), p_cal->barcode_threshold,
                   BITS_ADC);
    codec_put_uint(p_w, CODEC_KEY("barcode_learned"),
                   p_cal->b_barcode_learned ? 1U : 0U, BITS_CAL_LEARNED);
    codec_put_uint(p_w, CODEC_KEY("first_right"),
                   p_cal->b_first_right ? 1U : 0U, BITS_CAL_FIRST_RIGHT);
    codec_put_int(p_w, CODEC_KEY("overlap"), p_cal->overlap_samples,
                  BITS_CAL_OVERLAP);
    codec_put_uint(p_w, CODEC_KEY("b4_floor_mg"), p_cal->b4_floor_mg,
                   BITS_CAL_FLOOR);
    codec_put_uint(p_w, CODEC_KEY("b4_samples"), p_cal->b4_samples,
                   BITS_CAL_SAMPLES);
    codec_put_uint(p_w, CODEC_KEY("sweep_deg"),
                   non_negative(p_cal->sweep_deg), BITS_CAL_ANGLE);
}


static void
build_event (comms_link_msg_t * p_msg, codec_event_t const * p_event)
{
    codec_writer_t                 w;
    comms_obstacle_t const * const p_obs = &p_event->obstacle;
    char                           code[2];

    switch ((codec_event_type_t) p_event->type)
    {
        case CODEC_EVENT_BARCODE:
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

        case CODEC_EVENT_HUMP:
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

        case CODEC_EVENT_OBSTACLE:
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

        case CODEC_EVENT_MOTION:
            codec_begin(&w, p_msg, "robot/event/motion", QOS_1, false,
                        p_event->time_ms);
            codec_put_name(&w, CODEC_KEY("motion"),
                           CODEC_NAMES(g_motion_names), COMMS_MOTION_COUNT,
                           p_event->kind, BITS_MOTION_CLASS);
            break;

        case CODEC_EVENT_CALIBRATION:
            build_calibration(&w, p_msg, p_event);
            break;

        case CODEC_EVENT_RUN_BEGIN:
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

        case CODEC_EVENT_RUN_END:
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
    uint32_t       words[CODEC_MOTION_WORDS];
    codec_writer_t w;
    uint32_t       seq;
    uint32_t       hash;
    bool           b_built = false;

    if (slot_read(&g_motion_slot.seq, g_motion_slot.words, words,
                  (uint32_t) sizeof(words), &seq))
    {
        codec_unpack_motion(words, &motion);
        g_last_distance_mm = motion.distance_mm;

        if (snapshot_due(&g_motion_tx, seq, words, CODEC_MOTION_WORDS, now,
                         run_period_ms(), &hash))
        {
            codec_begin(&w, p_msg, "robot/telemetry/motion", QOS_0, false,
                        now);
            codec_put_int(&w, CODEC_KEY("spd_l_mm_s"),
                          motion.speed_left_mm_s, BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("spd_r_mm_s"),
                          motion.speed_right_mm_s, BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("enc_l"), motion.encoder_left,
                          BITS_ENCODER);
            codec_put_int(&w, CODEC_KEY("enc_r"), motion.encoder_right,
                          BITS_ENCODER);
            codec_put_int(&w, CODEC_KEY("dist_mm"), motion.distance_mm,
                          BITS_DISTANCE);
            codec_put_int(&w, CODEC_KEY("tgt_l_mm_s"),
                          motion.target_left_mm_s, BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("tgt_r_mm_s"),
                          motion.target_right_mm_s, BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("slew_l_mm_s"),
                          motion.slewed_left_mm_s, BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("slew_r_mm_s"),
                          motion.slewed_right_mm_s, BITS_SPEED);
            codec_put_int(&w, CODEC_KEY("pwm_l"),
                          motion.drive_left_permille, BITS_DRIVE);
            codec_put_int(&w, CODEC_KEY("pwm_r"),
                          motion.drive_right_permille, BITS_DRIVE);
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
    uint32_t       words[CODEC_LINE_WORDS];
    codec_writer_t w;
    uint32_t       seq;
    uint32_t       hash;
    bool           b_built = false;

    if (slot_read(&g_line_slot.seq, g_line_slot.words, words,
                  (uint32_t) sizeof(words), &seq))
    {
        codec_unpack_line(words, &line);

        if (snapshot_due(&g_line_tx, seq, words, CODEC_LINE_WORDS, now,
                         run_period_ms(), &hash))
        {
            codec_begin(&w, p_msg, "robot/telemetry/line", QOS_0, false,
                        now);
            codec_put_name(&w, CODEC_KEY("pos"), CODEC_NAMES(g_line_names),
                           COMMS_LINE_COUNT, (uint32_t) line.position,
                           BITS_LINE_POS);
            codec_put_int(&w, CODEC_KEY("err"), line.error, BITS_LINE_ERROR);
            codec_put_uint(&w, CODEC_KEY("sensors"), line.sensors,
                           BITS_SENSORS);
            codec_put_uint(&w, CODEC_KEY("centre_adc"), line.centre_adc,
                           BITS_ADC);
            codec_put_uint(&w, CODEC_KEY("right_adc"), line.right_adc,
                           BITS_ADC);
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
    uint32_t        words[CODEC_TERRAIN_WORDS];
    uint32_t        fields[CODEC_TERRAIN_WORDS + 1U];
    codec_writer_t  w;
    uint32_t        seq;
    uint32_t        hash;
    bool            b_built = false;

    if (slot_read(&g_terrain_slot.seq, g_terrain_slot.words, words,
                  (uint32_t) sizeof(words), &seq))
    {
        codec_unpack_terrain(words, &terrain);

        /* The run's highest hump is tracked here, not in the slot, so it
           is hashed too: a new record is sent when it rises. */
        fields[0] = words[0];
        fields[1] = words[1];
        fields[CODEC_TERRAIN_WORDS] = (uint32_t) g_max_hump_dmm;

        if (snapshot_due(&g_terrain_tx, seq, fields,
                         CODEC_TERRAIN_WORDS + 1U, now, COMMS_TELEMETRY_MS,
                         &hash))
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
            codec_put_int(&w, CODEC_KEY("turn_dps"), terrain.turn_dps,
                          BITS_TURN);
            codec_end(&w);
            snapshot_sent(&g_terrain_tx, seq, hash, now);
            b_built = true;
        }
    }

    return b_built;
}


static bool
next_sonar (comms_link_msg_t * p_msg, uint32_t now)
{
    comms_sonar_t  sonar;
    uint32_t       words[CODEC_SONAR_WORDS];
    codec_writer_t w;
    uint32_t       seq;
    uint32_t       hash;
    bool           b_built = false;

    if (slot_read(&g_sonar_slot.seq, g_sonar_slot.words, words,
                  (uint32_t) sizeof(words), &seq))
    {
        codec_unpack_sonar(words, &sonar);

        if (snapshot_due(&g_sonar_tx, seq, words, CODEC_SONAR_WORDS, now,
                         COMMS_TELEMETRY_MS, &hash))
        {
            codec_begin(&w, p_msg, "robot/telemetry/obstacle", QOS_0, false,
                        now);
            codec_put_uint(&w, CODEC_KEY("dist_mm"),
                           non_negative(sonar.distance_mm), BITS_OBST_DIST);
            codec_put_name(&w, CODEC_KEY("status"), CODEC_NAMES(g_range_names),
                           COMMS_RANGE_COUNT, (uint32_t) sonar.status,
                           BITS_RANGE);
            codec_put_name(&w, CODEC_KEY("state"), CODEC_NAMES(g_sonar_names),
                           COMMS_SONAR_COUNT, (uint32_t) sonar.state,
                           BITS_SONAR_STATE);
            codec_put_uint(&w, CODEC_KEY("silent"), sonar.silent,
                           BITS_SILENT);
            codec_end(&w);
            snapshot_sent(&g_sonar_tx, seq, hash, now);
            b_built = true;
        }
    }

    return b_built;
}


/* One queued event, unpacked.  An unknown event type (a corrupted element)
 * is dropped and counted, as the out-of-range rule requires. */
static bool
next_event (codec_event_t * p_event)
{
    uint32_t words[CODEC_EVENT_WORDS];
    bool     b_got = false;

    if ((INT) sizeof(words) == tk_rcv_mbf(g_event_mbf, words, TMO_POL))
    {
        if (codec_unpack_event(words, p_event))
        {
            b_got = true;
        }
        else
        {
            count_drop();
        }
    }

    return b_got;
}


bool
comms_link_next (comms_link_msg_t * p_msg)
{
    codec_event_t  event;
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
        fault   = (uint32_t) g_fault;   /* Masked when it was set. */
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
        else if (next_event(&event))
        {
            /* 3. Queued events, oldest first. */
            build_event(p_msg, &event);
        }
#if COMMS_DEBUG_TEXT
        else if ((0 < g_debug_mbf)
                 && (0 < tk_rcv_mbf(g_debug_mbf, g_debug_rx, TMO_POL)))
        {
            /* TEMPORARY: debug text lines, after the real events. */
            codec_text(p_msg, "robot/debug", QOS_0, false, g_debug_rx);
        }
#endif
        else if ((now - g_heartbeat_ms) >= COMMS_HEARTBEAT_MS)
        {
            /* 4. Heartbeat. */
            g_heartbeat_ms = now;
            g_heartbeat_seq++;
            codec_begin(&w, p_msg, "robot/heartbeat", QOS_0, false, now);
            codec_put_uint(&w, CODEC_KEY("seq"), (uint32_t) g_heartbeat_seq,
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
                      || next_terrain(p_msg, now)
                      || next_sonar(p_msg, now);
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
