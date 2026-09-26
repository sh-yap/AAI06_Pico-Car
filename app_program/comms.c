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
 */

#include <tk/tkernel.h>

#include "build_info.h"
#include "comms.h"
#include "comms_link.h"

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

#define COMMS_EVENT_QUEUE_LEN    16U    /* events waiting to be sent     */
#define COMMS_CMD_QUEUE_LEN      4U     /* commands waiting for pickup   */
#define COMMS_TELEMETRY_MS       200U   /* max 5 Hz per telemetry topic  */
#define COMMS_REFRESH_MS         2000U  /* resend unchanged values       */
#define COMMS_HEARTBEAT_MS       1000U
#define COMMS_MSG_VERSION        1
#define COMMS_MBF_OVERHEAD       8U     /* per-message kernel header     */

#define QOS_0                    0U
#define QOS_1                    1U

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
    int32_t          value;         /* hump peak height                  */
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

/* ------------------------------------------------------------------ *
 *  Names used in the JSON payloads (order must match comms.h)
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
    "LOST", "LEFT", "CENTRE", "RIGHT", "JUNCTION"
};

static char const * const g_motion_names[COMMS_MOTION_COUNT] =
{
    "STATIONARY", "ACCELERATING", "TURNING", "CLIMBING_HUMP",
    "DESCENDING_HUMP", "IMPACT"
};

static char const * const g_avoid_names[COMMS_AVOID_COUNT] =
{
    "STOP", "CONTINUE", "TURN_LEFT", "TURN_RIGHT", "REVERSE_RETRY"
};

/* ------------------------------------------------------------------ *
 *  Shared state (written by callers, read by the radio task)
 * ------------------------------------------------------------------ */

static ID                g_event_mbf;
static ID                g_cmd_mbf;
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


static char const *
name_of (char const * const * p_names, uint32_t count, uint32_t index)
{
    return (index < count) ? p_names[index] : "UNKNOWN";
}


static bool
queue_event (comms_event_t * p_event)
{
    ER result;

    if (!g_b_ready)
    {
        return false;
    }

    p_event->time_ms = now_ms();
    result = tk_snd_mbf(g_event_mbf, p_event, (INT) sizeof(*p_event),
                        TMO_POL);

    if (E_OK != result)
    {
        g_events_dropped++;
        return false;
    }

    return true;
}

/* ------------------------------------------------------------------ *
 *  Caller side
 * ------------------------------------------------------------------ */

bool
comms_init (void)
{
    T_CMBF cmbf = {0};

    if (g_b_ready)
    {
        return true;
    }

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ) sizeof(g_event_buf);
    cmbf.maxmsz = (INT) sizeof(comms_event_t);
    cmbf.bufptr = g_event_buf;
    g_event_mbf = tk_cre_mbf(&cmbf);

    cmbf.bufsz  = (SZ) sizeof(g_cmd_buf);
    cmbf.maxmsz = (INT) sizeof(uint32_t);
    cmbf.bufptr = g_cmd_buf;
    g_cmd_mbf   = tk_cre_mbf(&cmbf);

    g_b_ready = (g_event_mbf > 0) && (g_cmd_mbf > 0);
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
    g_fault = fault_flags;
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
    INT      size;

    if (!g_b_ready || (NULL == p_cmd))
    {
        return false;
    }

    size = tk_rcv_mbf(g_cmd_mbf, &cmd, TMO_POL);

    if (size != (INT) sizeof(cmd))
    {
        return false;
    }

    *p_cmd = (comms_cmd_t) cmd;
    return true;
}


void
comms_report_motion (comms_motion_t const * p_motion)
{
    if (NULL != p_motion)
    {
        g_motion_slot.seq++;
        memory_barrier();
        g_motion_slot.data = *p_motion;
        memory_barrier();
        g_motion_slot.seq++;
    }
}


void
comms_report_line (comms_line_t const * p_line)
{
    if (NULL != p_line)
    {
        g_line_slot.seq++;
        memory_barrier();
        g_line_slot.data = *p_line;
        memory_barrier();
        g_line_slot.seq++;
    }
}


void
comms_report_terrain (comms_terrain_t const * p_terrain)
{
    if (NULL != p_terrain)
    {
        g_terrain_slot.seq++;
        memory_barrier();
        g_terrain_slot.data = *p_terrain;
        memory_barrier();
        g_terrain_slot.seq++;
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
comms_event_hump (int32_t peak_height_mm)
{
    comms_event_t event = {0};

    event.type  = (uint8_t) EVENT_HUMP;
    event.value = peak_height_mm;
    return queue_event(&event);
}


bool
comms_event_obstacle (comms_obstacle_t const * p_obstacle)
{
    comms_event_t event = {0};

    if (NULL == p_obstacle)
    {
        return false;
    }

    event.type     = (uint8_t) EVENT_OBSTACLE;
    event.obstacle = *p_obstacle;
    return queue_event(&event);
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
    uint32_t last_seq;      /* seq of the value last published        */
    uint32_t last_hash;     /* content of the value last published    */
    uint32_t last_sent_ms;
    bool     b_force;       /* resend even if unchanged               */
} snapshot_tx_t;

static snapshot_tx_t g_motion_tx;
static snapshot_tx_t g_line_tx;
static snapshot_tx_t g_terrain_tx;

static uint32_t g_published_state = 0xFFFFFFFFU;
static uint32_t g_published_fault = 0xFFFFFFFFU;
static uint32_t g_heartbeat_ms;
static uint32_t g_heartbeat_seq;
static bool     g_b_info_pending;

/* Acknowledgement of the last command received. */
static bool         g_b_ack_pending;
static char const * g_p_ack_name;
static bool         g_b_ack_ok;

/* Run summary, built from the events as they are published. */
static bool     g_b_run_active;
static uint32_t g_run_start_ms;
static int32_t  g_run_start_mm;
static int32_t  g_last_distance_mm;
static int32_t  g_max_hump_mm;
static uint32_t g_hump_count;
static uint32_t g_barcode_count;
static uint32_t g_obstacle_count;

/* --- JSON builder: appends to p_msg->payload, truncating safely --- */

static void
add_raw (comms_link_msg_t * p_msg, char const * p_text)
{
    uint32_t pos = p_msg->payload_len;

    while (('\0' != *p_text) && ((pos + 1U) < COMMS_LINK_PAYLOAD_MAX))
    {
        p_msg->payload[pos] = *p_text;
        pos++;
        p_text++;
    }

    p_msg->payload[pos] = '\0';
    p_msg->payload_len  = (uint16_t) pos;
}


static void
add_raw_int (comms_link_msg_t * p_msg, int32_t value)
{
    char     digits[12];    /* "-2147483648" + terminator */
    uint32_t n = sizeof(digits) - 1U;
    uint32_t magnitude;

    digits[n] = '\0';
    magnitude = (value < 0) ? (0U - (uint32_t) value) : (uint32_t) value;

    do
    {
        n--;
        digits[n]  = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude > 0U) && (n > 1U));

    if (value < 0)
    {
        n--;
        digits[n] = '-';
    }

    add_raw(p_msg, &digits[n]);
}


static void
add_int (comms_link_msg_t * p_msg, char const * p_key, int32_t value)
{
    add_raw(p_msg, ",\"");
    add_raw(p_msg, p_key);
    add_raw(p_msg, "\":");
    add_raw_int(p_msg, value);
}


static void
add_text (comms_link_msg_t * p_msg, char const * p_key, char const * p_value)
{
    add_raw(p_msg, ",\"");
    add_raw(p_msg, p_key);
    add_raw(p_msg, "\":\"");
    add_raw(p_msg, p_value);
    add_raw(p_msg, "\"");
}


/* Starts a message: topic, delivery options and the common header
   {"v":1,"t":<uptime ms>  -- every payload carries version and time. */
static void
msg_begin (comms_link_msg_t * p_msg, char const * p_topic, uint8_t qos,
           bool b_retain, uint32_t time_ms)
{
    uint32_t i = 0U;

    while (('\0' != p_topic[i]) && ((i + 1U) < COMMS_LINK_TOPIC_MAX))
    {
        p_msg->topic[i] = p_topic[i];
        i++;
    }

    p_msg->topic[i]    = '\0';
    p_msg->qos         = qos;
    p_msg->b_retain    = b_retain ? 1U : 0U;
    p_msg->payload_len = 0U;

    add_raw(p_msg, "{\"v\":");
    add_raw_int(p_msg, COMMS_MSG_VERSION);
    add_int(p_msg, "t", (int32_t) time_ms);
}


static void
msg_end (comms_link_msg_t * p_msg)
{
    add_raw(p_msg, "}");
}

/* --- snapshot readers (seqlock) --- */

static bool
read_motion (comms_motion_t * p_out, uint32_t * p_seq)
{
    uint32_t seq = g_motion_slot.seq;

    memory_barrier();

    if (0U != (seq & 1U))
    {
        return false;               /* writer busy: try on the next poll */
    }

    *p_out = g_motion_slot.data;
    memory_barrier();
    *p_seq = seq;
    return (seq == g_motion_slot.seq);
}


static bool
read_line (comms_line_t * p_out, uint32_t * p_seq)
{
    uint32_t seq = g_line_slot.seq;

    memory_barrier();

    if (0U != (seq & 1U))
    {
        return false;
    }

    *p_out = g_line_slot.data;
    memory_barrier();
    *p_seq = seq;
    return (seq == g_line_slot.seq);
}


static bool
read_terrain (comms_terrain_t * p_out, uint32_t * p_seq)
{
    uint32_t seq = g_terrain_slot.seq;

    memory_barrier();

    if (0U != (seq & 1U))
    {
        return false;
    }

    *p_out = g_terrain_slot.data;
    memory_barrier();
    *p_seq = seq;
    return (seq == g_terrain_slot.seq);
}


/* FNV-1a over the raw bytes: a cheap fingerprint of a snapshot, so a value
   that is reported again but has not changed is not sent again. */
static uint32_t
hash_bytes (void const * p_data, uint32_t len)
{
    uint8_t const * p_byte = (uint8_t const *) p_data;
    uint32_t        hash   = 2166136261U;
    uint32_t        i;

    for (i = 0U; i < len; i++)
    {
        hash ^= p_byte[i];
        hash *= 16777619U;
    }

    return hash;
}


/* A snapshot is due when it has ever been written, the per-topic rate
   limit has elapsed, and either its content changed, a resend was asked
   for, or the refresh interval passed (so a late subscriber catches up). */
static bool
snapshot_due (snapshot_tx_t const * p_tx, uint32_t seq, uint32_t hash,
              uint32_t now)
{
    uint32_t age = now - p_tx->last_sent_ms;

    return (0U != seq)
        && (age >= COMMS_TELEMETRY_MS)
        && ((hash != p_tx->last_hash) || p_tx->b_force
            || (age >= COMMS_REFRESH_MS));
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

/* --- message producers --- */

static void
build_event (comms_link_msg_t * p_msg, comms_event_t const * p_event)
{
    char code[2];

    switch ((event_type_t) p_event->type)
    {
        case EVENT_BARCODE:
            code[0] = (char) p_event->code;
            code[1] = '\0';
            g_barcode_count++;
            msg_begin(p_msg, "robot/event/barcode", QOS_1, false,
                      p_event->time_ms);
            add_text(p_msg, "code", code);
            add_text(p_msg, "command",
                     name_of(g_nav_names, COMMS_NAV_COUNT, p_event->kind));
            break;

        case EVENT_HUMP:
            g_hump_count++;
            if (p_event->value > g_max_hump_mm)
            {
                g_max_hump_mm = p_event->value;
            }
            msg_begin(p_msg, "robot/event/hump", QOS_1, false,
                      p_event->time_ms);
            add_int(p_msg, "peak_mm", p_event->value);
            add_int(p_msg, "max_mm", g_max_hump_mm);
            break;

        case EVENT_OBSTACLE:
            g_obstacle_count++;
            msg_begin(p_msg, "robot/event/obstacle", QOS_1, false,
                      p_event->time_ms);
            add_int(p_msg, "dist_mm", p_event->obstacle.distance_mm);
            add_int(p_msg, "angle_deg", p_event->obstacle.angle_deg);
            add_int(p_msg, "width_mm", p_event->obstacle.width_mm);
            add_int(p_msg, "clear_l_mm", p_event->obstacle.clear_left_mm);
            add_int(p_msg, "clear_r_mm", p_event->obstacle.clear_right_mm);
            add_text(p_msg, "action",
                     name_of(g_avoid_names, COMMS_AVOID_COUNT,
                             (uint32_t) p_event->obstacle.action));
            break;

        case EVENT_MOTION:
            msg_begin(p_msg, "robot/event/motion", QOS_1, false,
                      p_event->time_ms);
            add_text(p_msg, "motion",
                     name_of(g_motion_names, COMMS_MOTION_COUNT,
                             p_event->kind));
            break;

        case EVENT_RUN_BEGIN:
            g_b_run_active   = true;
            g_run_start_ms   = p_event->time_ms;
            g_run_start_mm   = g_last_distance_mm;
            g_max_hump_mm    = 0;
            g_hump_count     = 0U;
            g_barcode_count  = 0U;
            g_obstacle_count = 0U;
            msg_begin(p_msg, "robot/event/run", QOS_1, false,
                      p_event->time_ms);
            add_text(p_msg, "run", "BEGIN");
            break;

        case EVENT_RUN_END:
        default:
            /* Retained, so a dashboard that connects later still sees
               the result of the last run. */
            msg_begin(p_msg, "robot/summary", QOS_1, true, p_event->time_ms);
            add_int(p_msg, "run_ms", g_b_run_active
                    ? (int32_t) (p_event->time_ms - g_run_start_ms) : 0);
            add_int(p_msg, "dist_mm", g_last_distance_mm - g_run_start_mm);
            add_int(p_msg, "max_hump_mm", g_max_hump_mm);
            add_int(p_msg, "humps", (int32_t) g_hump_count);
            add_int(p_msg, "barcodes", (int32_t) g_barcode_count);
            add_int(p_msg, "obstacles", (int32_t) g_obstacle_count);
            g_b_run_active = false;
            break;
    }

    msg_end(p_msg);
    g_events_sent++;
}


static bool
next_snapshot (comms_link_msg_t * p_msg, uint32_t now)
{
    comms_motion_t  motion;
    comms_line_t    line;
    comms_terrain_t terrain;
    int32_t         fields[5];      /* the values that go on the wire */
    uint32_t        seq;
    uint32_t        hash;

    if (read_motion(&motion, &seq))
    {
        g_last_distance_mm = motion.distance_mm;
        fields[0] = motion.speed_left_mm_s;
        fields[1] = motion.speed_right_mm_s;
        fields[2] = motion.encoder_left;
        fields[3] = motion.encoder_right;
        fields[4] = motion.distance_mm;
        hash      = hash_bytes(fields, sizeof(fields));

        if (snapshot_due(&g_motion_tx, seq, hash, now))
        {
            msg_begin(p_msg, "robot/telemetry/motion", QOS_0, false, now);
            add_int(p_msg, "spd_l_mm_s", motion.speed_left_mm_s);
            add_int(p_msg, "spd_r_mm_s", motion.speed_right_mm_s);
            add_int(p_msg, "enc_l", motion.encoder_left);
            add_int(p_msg, "enc_r", motion.encoder_right);
            add_int(p_msg, "dist_mm", motion.distance_mm);
            msg_end(p_msg);
            snapshot_sent(&g_motion_tx, seq, hash, now);
            return true;
        }
    }

    if (read_line(&line, &seq))
    {
        fields[0] = (int32_t) line.position;
        fields[1] = line.error;
        fields[2] = (int32_t) line.sensors;
        hash      = hash_bytes(fields, 3U * sizeof(fields[0]));

        if (snapshot_due(&g_line_tx, seq, hash, now))
        {
            msg_begin(p_msg, "robot/telemetry/line", QOS_0, false, now);
            add_text(p_msg, "pos", name_of(g_line_names, COMMS_LINE_COUNT,
                                           (uint32_t) line.position));
            add_int(p_msg, "err", line.error);
            add_int(p_msg, "sensors", (int32_t) line.sensors);
            msg_end(p_msg);
            snapshot_sent(&g_line_tx, seq, hash, now);
            return true;
        }
    }

    if (read_terrain(&terrain, &seq))
    {
        fields[0] = (int32_t) terrain.motion;
        fields[1] = terrain.pitch_deg;
        fields[2] = terrain.roll_deg;
        fields[3] = terrain.hump_height_mm;
        fields[4] = g_max_hump_mm;
        hash      = hash_bytes(fields, sizeof(fields));

        if (snapshot_due(&g_terrain_tx, seq, hash, now))
        {
            msg_begin(p_msg, "robot/telemetry/terrain", QOS_0, false, now);
            add_text(p_msg, "motion",
                     name_of(g_motion_names, COMMS_MOTION_COUNT,
                             (uint32_t) terrain.motion));
            add_int(p_msg, "pitch_deg", terrain.pitch_deg);
            add_int(p_msg, "roll_deg", terrain.roll_deg);
            add_int(p_msg, "hump_mm", terrain.hump_height_mm);
            add_int(p_msg, "max_hump_mm", g_max_hump_mm);
            msg_end(p_msg);
            snapshot_sent(&g_terrain_tx, seq, hash, now);
            return true;
        }
    }

    return false;
}


bool
comms_link_next (comms_link_msg_t * p_msg)
{
    comms_event_t event;
    uint32_t      now;
    uint32_t      state;
    uint32_t      fault;

    if (!g_b_ready || (NULL == p_msg))
    {
        return false;
    }

    now   = now_ms();
    state = g_state;
    fault = g_fault;

    /* 1. Command acknowledgement: the sender is waiting to see it. */
    if (g_b_ack_pending)
    {
        g_b_ack_pending = false;
        msg_begin(p_msg, "robot/event/command", QOS_1, false, now);
        add_text(p_msg, "cmd", g_p_ack_name);
        add_int(p_msg, "ok", g_b_ack_ok ? 1 : 0);
        msg_end(p_msg);
        return true;
    }

    /* Which firmware is this?  Once per session, retained. */
    if (g_b_info_pending)
    {
        g_b_info_pending = false;
        msg_begin(p_msg, "robot/info", QOS_1, true, now);
        add_text(p_msg, "build", g_build_id);
        msg_end(p_msg);
        return true;
    }

    /* 2. Robot state, retained, whenever it or the fault word changes. */
    if ((state != g_published_state) || (fault != g_published_fault))
    {
        g_published_state = state;
        g_published_fault = fault;
        msg_begin(p_msg, "robot/state", QOS_1, true, now);
        add_text(p_msg, "state",
                 name_of(g_state_names, COMMS_STATE_COUNT, state));
        add_int(p_msg, "fault", (int32_t) fault);
        msg_end(p_msg);
        return true;
    }

    /* 3. Queued events, oldest first. */
    if (tk_rcv_mbf(g_event_mbf, &event, TMO_POL) == (INT) sizeof(event))
    {
        build_event(p_msg, &event);
        return true;
    }

    /* 4. Heartbeat. */
    if ((now - g_heartbeat_ms) >= COMMS_HEARTBEAT_MS)
    {
        g_heartbeat_ms = now;
        g_heartbeat_seq++;
        msg_begin(p_msg, "robot/heartbeat", QOS_0, false, now);
        add_int(p_msg, "seq", (int32_t) g_heartbeat_seq);
        add_text(p_msg, "state",
                 name_of(g_state_names, COMMS_STATE_COUNT, state));
        add_int(p_msg, "fault", (int32_t) fault);
        add_int(p_msg, "ev_drop", (int32_t) g_events_dropped);
        msg_end(p_msg);
        return true;
    }

    /* 5. Telemetry snapshots. */
    return next_snapshot(p_msg, now);
}


void
comms_link_set_connected (bool b_connected)
{
    g_b_connected = b_connected;

    if (b_connected)
    {
        /* Fresh session: resend the current picture straight away. */
        g_b_info_pending     = true;
        g_published_state    = 0xFFFFFFFFU;
        g_motion_tx.b_force  = true;
        g_line_tx.b_force    = true;
        g_terrain_tx.b_force = true;
    }
}


static bool
payload_is (uint8_t const * p_data, uint32_t len, char const * p_word)
{
    uint32_t i = 0U;

    while ((i < len) && ('\0' != p_word[i]) && (p_data[i] == (uint8_t) p_word[i]))
    {
        i++;
    }

    return (i == len) && ('\0' == p_word[i]);
}


void
comms_link_on_command (uint8_t const * p_data, uint32_t len)
{
    uint32_t cmd = (uint32_t) COMMS_CMD_NONE;

    g_commands_received++;

    /* Ignore trailing whitespace, e.g. a newline from a terminal. */
    while ((len > 0U) && ((p_data[len - 1U] == (uint8_t) '\n')
                          || (p_data[len - 1U] == (uint8_t) '\r')
                          || (p_data[len - 1U] == (uint8_t) ' ')))
    {
        len--;
    }

    g_b_ack_ok = true;

    if (payload_is(p_data, len, "start"))
    {
        cmd          = (uint32_t) COMMS_CMD_START;
        g_p_ack_name = "start";
    }
    else if (payload_is(p_data, len, "stop"))
    {
        cmd          = (uint32_t) COMMS_CMD_STOP;
        g_p_ack_name = "stop";
    }
    else if (payload_is(p_data, len, "status"))
    {
        /* Handled here: resend everything now. */
        comms_link_set_connected(true);
        g_p_ack_name = "status";
    }
    else
    {
        g_p_ack_name = "unknown";
        g_b_ack_ok   = false;
    }

    if ((uint32_t) COMMS_CMD_NONE != cmd)
    {
        /* Queue full means the controller is not reading commands. */
        g_b_ack_ok = g_b_ready
                  && (E_OK == tk_snd_mbf(g_cmd_mbf, &cmd,
                                         (INT) sizeof(cmd), TMO_POL));
    }

    g_b_ack_pending = true;
}
