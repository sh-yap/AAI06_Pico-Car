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
 * BARR-C 4.2: no #include here.  Include <stdint.h> before this header.
 */

#ifndef RUN_REC_H
#define RUN_REC_H

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

/* Run BEGIN: empty the ring (and the barcode ring) and start recording. */
void run_rec_begin(void);

/* Append one record (cycle is filled in here).  Ignored when frozen or
 * before the first BEGIN. */
void run_rec_add(run_rec_record_t const * p_record);

/* Run END: stop writing (both rings), keep the contents. */
void run_rec_freeze(void);

/* ---- Barcode sensor trace ------------------------------------------------
 * A second ring, g_bc_rec: one record per Buddy 3 cycle (10 ms) so the
 * barcode sensor can be studied at full rate.  It begins and freezes with
 * the run ring above (run_rec_begin / run_rec_freeze).  Only Buddy 3 writes
 * records; the VC task resets and freezes it, so a record torn at the
 * boundary is possible and accepted.  No locks.  Keep tools/run_rec_read.ps1
 * in step. */

#define BC_REC_MAGIC        (0x31524342UL)  /* "BCR1" little endian.        */
#define BC_REC_CAPACITY     (2000U)         /* 20 s at 10 ms.               */
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

#endif /* RUN_REC_H */

/*** end of file ***/
