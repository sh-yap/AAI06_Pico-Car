/** @file run_rec.h
 *
 * @brief Test-only run recorder: one record per 20 ms Vehicle Controller
 *        cycle in a RAM ring buffer, read out with the debug probe.
 *
 * Built only with VC_RECORDER=1 (build_make/pico_rp2040.mk); the normal
 * image has none of it.  The ring starts empty when the START calibration
 * begins (BEGIN), keeps the newest RUN_REC_CAPACITY records and freezes when
 * the run ends or the calibration fails, so it can
 * be read after STOP.  Only the VC task writes it.  It never blocks,
 * allocates or prints.  Readout: tools/run_rec_read.ps1.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h> and <stdbool.h> before
 * this header.
 */

#ifndef RUN_REC_H
#define RUN_REC_H

#ifndef B4_RECORDER
#define B4_RECORDER         (0)             /* Buddy 4 ring off unless built.*/
#endif

#define RUN_REC_MAGIC       (0x31434552UL)  /* "REC1" little endian.        */
#define RUN_REC_CAPACITY    (1500U)         /* 30 s at 20 ms.               */
#define RUN_REC_FLAG_RAMP   (0x01U)         /* Terrain: ramp active.        */

/* One VC cycle, 44 bytes, no padding.  Keep tools/run_rec_read.ps1 in step. */
typedef struct
{
    uint32_t ms;                /* Kernel time at the start of the cycle.   */
    uint16_t cycle;             /* Cycles since BEGIN, wraps at 65536.      */
    uint16_t centre_adc;        /* Raw centre line sensor, 0..4095.         */
    uint16_t right_adc;         /* Raw right line sensor, 0..4095.          */
    uint8_t  line_state;        /* vc_line_t as the VC saw it.              */
    int8_t   line_error;        /* -127 left .. +127 right.                 */
    uint8_t  sensors;           /* Line sensor bits, 1 = black.             */
    uint8_t  vc_state;          /* vc_state_t after the step.               */
    int8_t   steer;             /* VC steer output.                         */
    uint8_t  flags;             /* RUN_REC_FLAG_*.                          */
    int16_t  speed_mm_s;        /* VC forward speed output.                 */
    int16_t  target_left_mm_s;  /* Motion wheel targets.                    */
    int16_t  target_right_mm_s;
    int16_t  left_mm_s;         /* Measured wheel speeds.                   */
    int16_t  right_mm_s;
    uint16_t overruns;          /* VC late-start count since boot, low 16.  */
    int16_t  left_pwm_permille; /* Signed PWM Motion applied this cycle.    */
    int16_t  right_pwm_permille;
    int16_t  left_i_permille;   /* PID integral term (ki * integral).       */
    int16_t  right_i_permille;
    int16_t  learn_fwd_gain;    /* Learned forward gain, 0.001 mm/s per
                                   permille per count.                      */
    int16_t  learn_fwd_offset;  /* Learned forward offset, 0.1 mm/s.        */
    int16_t  learn_turn_gain;   /* Learned turning gain, 0.001 mm/s per
                                   permille per count.                      */
    uint16_t learn_state;       /* Bits 0-1 turning source (0 default line,
                                   1 pivot seed, 2 driving), bit 2 forward
                                   from driving.                            */
} run_rec_record_t;

/* Header first, so a readout finds everything by the one symbol g_run_rec.
 * magic is 0 until the first BEGIN (nothing recorded since boot).  The
 * newest record is at write_index - 1; when total is below capacity the
 * records are 0 .. total - 1, else the oldest is at write_index. */
typedef struct
{
    uint32_t         magic;         /* RUN_REC_MAGIC once a run began.      */
    uint16_t         record_size;   /* sizeof(run_rec_record_t).            */
    uint16_t         capacity;      /* RUN_REC_CAPACITY.                    */
    uint32_t         write_index;   /* Next slot to write.                  */
    uint32_t         total;         /* Records written since BEGIN.         */
    uint32_t         frozen;        /* 1 once the run ended.                */
    run_rec_record_t records[RUN_REC_CAPACITY];
} run_rec_t;

extern run_rec_t g_run_rec;

/* Run BEGIN: empty the ring (and the barcode and Buddy 4 rings) and start
 * recording. */
void run_rec_begin(void);

/* Append one record (cycle is filled in here).  Ignored when frozen or
 * before the first BEGIN. */
void run_rec_add(run_rec_record_t const * p_record);

/* Run END: stop writing (all rings), keep the contents. */
void run_rec_freeze(void);

/* ---- Barcode sensor trace ------------------------------------------------
 * A second ring, g_bc_rec: one record per Buddy 3 cycle (10 ms) so the
 * barcode sensor can be studied at full rate.  It begins and freezes with
 * the run ring above (run_rec_begin / run_rec_freeze).  Only Buddy 3 writes
 * records; the VC task resets and freezes it, so a record torn at the
 * boundary is possible and accepted.  No locks.  Keep tools/run_rec_read.ps1
 * in step. */

#define BC_REC_MAGIC        (0x31524342UL)  /* "BCR1" little endian.        */
#if B4_RECORDER
#define BC_REC_CAPACITY     (500U)          /* 5 s: RAM goes to the B4 ring. */
#else
#define BC_REC_CAPACITY     (2000U)         /* 20 s at 10 ms.               */
#endif
#define BC_REC_FLAG_BLACK   (0x01U)         /* Barcode sensor read black.   */
#define BC_REC_FLAG_DONE    (0x02U)         /* A capture finished this cycle.*/
#define BC_REC_FLAG_ACTIVE  (0x04U)         /* Edges stored, not finished.  */

/* One Buddy 3 cycle, 36 bytes, no padding. Legacy records were 20 bytes. */
typedef struct
{
    uint32_t ms;                /* Monotonic time BEFORE ADC acquisition.   */
    int32_t  enc;               /* Average wheel encoder position.          */
    uint16_t barcode_adc;       /* Raw barcode sensor, 0..4095.             */
    uint16_t threshold;         /* Black threshold in use.                  */
    uint16_t centre_adc;        /* Raw centre line sensor.                  */
    uint16_t right_adc;         /* Raw right line sensor.                   */
    uint8_t  flags;             /* BC_REC_FLAG_*.                           */
    uint8_t  edges;             /* Edges stored in the current capture.     */
    uint8_t  code;              /* barcode_t when a capture finished.       */
    uint8_t  ch;                /* Decoded character, else 0.               */
    int32_t  left_enc;          /* Individual sequence-validated snapshot.  */
    int32_t  right_enc;
    uint32_t encoder_us;        /* Snapshot hardware clock, low 32 bits.    */
    uint32_t sample_end_ms;     /* Monotonic time AFTER snapshot, same
                                   clock as ms and the VC recorder.         */
} bc_rec_record_t;

/* Same header shape as run_rec_t.  magic is 0 until the first BEGIN. */
typedef struct
{
    uint32_t        magic;          /* BC_REC_MAGIC once a run began.       */
    uint16_t        record_size;    /* sizeof(bc_rec_record_t).             */
    uint16_t        capacity;       /* BC_REC_CAPACITY.                     */
    uint32_t        write_index;    /* Next slot to write.                  */
    uint32_t        total;          /* Records written since BEGIN.         */
    uint32_t        frozen;         /* 1 once the run ended.                */
    bc_rec_record_t records[BC_REC_CAPACITY];
} bc_rec_t;

extern bc_rec_t g_bc_rec;

/* Append one record.  Ignored when frozen or before the first BEGIN. */
void bc_rec_add(bc_rec_record_t const * p_record);

#if B4_RECORDER
/* ---- Buddy 4 trace ---------------------------------------------------------
 * A third ring, g_b4_rec: one record per Buddy 4 cycle (10 ms) with the
 * accelerometer, incline, hump and magnetometer signals, so ramp crossings
 * and turns can be analysed offline.  Built only with B4_RECORDER=1, which
 * needs VC_RECORDER=1.  It begins and freezes with the run ring.  Only the
 * Buddy 4 task writes records; the VC task resets and freezes it, so a record
 * torn at the boundary is possible and accepted.  No locks.  Keep
 * tools/run_rec_read.ps1 in step. */

#define B4_REC_MAGIC        (0x31523442UL)  /* "B4R1" little endian.        */
#define B4_REC_CAPACITY     (1500U)         /* 15 s at 10 ms.               */
#define B4_REC_FLAG_VALID   (0x01U)         /* Accelerometer sample valid.  */
#define B4_REC_FLAG_MAG_NEW (0x02U)         /* Mag sample new and valid.    */
#define B4_REC_FLAG_HEIGHT  (0x04U)         /* Hump height valid.           */
#define B4_REC_FLAG_IMPACT  (0x08U)         /* Impact event this cycle.     */
#define B4_REC_FLAG_STARTED (0x10U)         /* Hump started this cycle.     */
#define B4_REC_FLAG_DONE    (0x20U)         /* Hump completed this cycle.   */
#define B4_REC_FLAG_STALE   (0x40U)         /* Sample stale.                */

/* One Buddy 4 cycle, 40 bytes, no padding. */
typedef struct
{
    uint32_t ms;                /* Kernel time, same clock as the VC ring.  */
    int16_t  accel_x_mg;        /* Calibrated sample, clamped to 16 bits.   */
    int16_t  accel_y_mg;
    int16_t  accel_z_mg;
    int16_t  incline_mg;        /* Filtered incline (roll_mg).              */
    int16_t  baseline_mg;       /* Flat-ground incline baseline.            */
    int16_t  delta_mg;          /* Incline offset the detector used.        */
    uint16_t noise_mg;          /* Learned noise median.                    */
    uint16_t active_height_cmm; /* Hump in progress, 0.01 mm, saturating.   */
    uint16_t final_height_cmm;  /* Last completed hump, 0.01 mm.            */
    int16_t  mag_x;             /* Latest magnetometer sample, counts.      */
    int16_t  mag_y;
    int16_t  mag_z;
    uint16_t left_enc;          /* Encoder counts, low 16 bits, wrap.       */
    uint16_t right_enc;
    int16_t  drive_speed_mm_s;  /* Mean absolute wheel speed.               */
    uint16_t dt_us;             /* Hardware us since previous record.       */
    uint8_t  hump_state;        /* b4_types_hump_state_t.                   */
    uint8_t  flags;             /* B4_REC_FLAG_*.                           */
    uint8_t  motion_turn;       /* Motion in bits 0-3, turn in bits 4-7.    */
    uint8_t  tilt;              /* b4_types_tilt_t.                         */
} b4_rec_record_t;

/* Same header shape as bc_rec_t.  magic is 0 until the first BEGIN. */
typedef struct
{
    uint32_t        magic;          /* B4_REC_MAGIC once a run began.       */
    uint16_t        record_size;    /* sizeof(b4_rec_record_t).             */
    uint16_t        capacity;       /* B4_REC_CAPACITY.                     */
    uint32_t        write_index;    /* Next slot to write.                  */
    uint32_t        total;          /* Records written since BEGIN.         */
    uint32_t        frozen;         /* 1 once the run ended.                */
    b4_rec_record_t records[B4_REC_CAPACITY];
} b4_rec_t;

extern b4_rec_t g_b4_rec;

/* Append one record.  Ignored when frozen or before the first BEGIN. */
void b4_rec_add(b4_rec_record_t const * p_record);
#endif /* B4_RECORDER */

/* ---- Whole-run event log --------------------------------------------------
 * A fourth, small log, g_ev_rec: the rings above keep only the newest few
 * seconds, so this one keeps the line losses, ramp edges and hump results of
 * the whole run.  When full it stops (earliest events kept) and counts
 * dropped; it never wraps.  It begins and freezes with the run ring.  Only
 * the VC task writes it (ev_rec_step at the 20 ms step, ev_rec_hump when a
 * terrain word shows a new hump count).  Keep tools/run_rec_read.ps1 in
 * step. */

#define EV_REC_MAGIC        (0x31564545UL)  /* "EEV1" little endian.        */
#define EV_REC_CAPACITY     (62U)           /* 16 + 62 * 8 = 512 bytes.     */
#define EV_REC_VALID_BIT    (0x80U)         /* HUMP_DONE count byte: the
                                               word's height was valid.     */
#define EV_REC_COUNT_MASK   (0x7FU)         /* HUMP_DONE count byte: hump
                                               count (4 bits in the word).  */

typedef enum
{
    EV_REC_LINE_LOST = 1,       /* Line LOST in FOLLOW_LINE; value = steer
                                   of the previous step.                    */
    EV_REC_LINE_BACK,           /* Line left LOST; value = ms since
                                   LINE_LOST, clamped to 32767.             */
    EV_REC_RAMP_START,          /* Ramp flag rose; value 0.                 */
    EV_REC_RAMP_END,            /* Ramp flag fell; value 0.                 */
    EV_REC_HUMP_DONE,           /* Hump count changed; value = newest
                                   height in 0.1 mm, count = hump count
                                   (EV_REC_COUNT_MASK) | EV_REC_VALID_BIT
                                   when the word's height was valid.        */
    EV_REC_RUN_MAX              /* The word's highest height changed;
                                   value = it in 0.1 mm.  Not logged for
                                   the reset to 0 at BEGIN.                 */
} ev_rec_type_t;

/* One event, 8 bytes, no padding. */
typedef struct
{
    uint32_t ms;                /* Kernel time, same clock as the VC ring.  */
    int16_t  value;             /* Per ev_rec_type_t.                       */
    uint8_t  type;              /* ev_rec_type_t.                           */
    uint8_t  count;             /* HUMP_DONE: count and valid bit, else 0.  */
} ev_rec_record_t;

/* Header first.  magic is 0 until the first BEGIN.  Records 0 .. count - 1
 * are valid, oldest first. */
typedef struct
{
    uint32_t        magic;          /* EV_REC_MAGIC once a run began.       */
    uint16_t        record_size;    /* sizeof(ev_rec_record_t).             */
    uint16_t        capacity;       /* EV_REC_CAPACITY.                     */
    uint16_t        count;          /* Events stored.                       */
    uint16_t        dropped;        /* Events lost because it was full.     */
    uint16_t        frozen;         /* 1 once the run ended.                */
    uint16_t        reserved;       /* 0.                                   */
    ev_rec_record_t records[EV_REC_CAPACITY];
} ev_rec_t;

extern ev_rec_t g_ev_rec;

/* Detect changes since the previous 20 ms step and log them.  Call once per
 * step, after the state change, with this step's values.  Ignored when
 * frozen or before the first BEGIN. */
void ev_rec_step(uint32_t ms, bool b_lost, bool b_follow, int8_t steer,
                 bool b_ramp);

/* Log a completed hump with its newest height and whether that height was
 * valid.  Ignored when frozen or before the first BEGIN. */
void ev_rec_hump(uint32_t ms, uint16_t height_dmm, uint8_t count,
                 bool b_valid);

/* Log the word's highest height when it differs from the last one seen
 * (the last one is 0 at BEGIN).  Ignored when frozen or before BEGIN. */
void ev_rec_max(uint32_t ms, uint16_t highest_dmm);

#endif /* RUN_REC_H */

/*** end of file ***/
