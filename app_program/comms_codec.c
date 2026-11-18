/** @file comms_codec.c
 *
 * @brief Binary / JSON message encoder (see comms_codec.h).
 *
 * Only the format selected by COMMS_JSON is compiled in.
 */

#include <stdint.h>
#include <stdbool.h>
#include "comms.h"
#include "comms_link.h"
#include "comms_codec.h"

/* ------------------------------------------------------------------------ *
 * Shared helpers
 * ------------------------------------------------------------------------ */

/* Largest value an unsigned field of this width holds (bits 1..32). */
static uint32_t
uint_max (uint32_t bits)
{
    return (32U <= bits) ? 0xFFFFFFFFU : ((1UL << bits) - 1U);
}


static uint32_t
clamp_uint (uint32_t value, uint32_t bits)
{
    uint32_t const max = uint_max(bits);

    return (value > max) ? max : value;
}


/* Clamp to a signed field (bits 2..32). */
static int32_t
clamp_int (int32_t value, uint32_t bits)
{
    int32_t result = value;
    int32_t max;
    int32_t min;

    if (32U > bits)
    {
        max = (int32_t) ((1UL << (bits - 1U)) - 1U);
        min = -max - 1;

        if (value > max)
        {
            result = max;
        }
        else if (value < min)
        {
            result = min;
        }
        else
        {
            /* In range. */
        }
    }

    return result;
}


/* Copy a string into a fixed buffer, always terminated; returns length. */
static uint32_t
copy_text (char * p_dst, uint32_t size, char const * p_src)
{
    uint32_t i = 0U;

    while (('\0' != p_src[i]) && ((i + 1U) < size))
    {
        p_dst[i] = p_src[i];
        i++;
    }

    p_dst[i] = '\0';
    return i;
}


static void
set_header (comms_link_msg_t * p_msg, char const * p_topic, uint8_t qos,
            bool b_retain)
{
    (void) copy_text(p_msg->topic, COMMS_LINK_TOPIC_MAX, p_topic);
    p_msg->qos         = qos;
    p_msg->b_retain    = b_retain ? 1U : 0U;
    p_msg->payload_len = 0U;
}

#if COMMS_JSON
/* ------------------------------------------------------------------------ *
 * JSON writer: appends text, truncating safely at the buffer end
 * ------------------------------------------------------------------------ */

static void
json_raw (comms_link_msg_t * p_msg, char const * p_text)
{
    uint32_t     pos    = p_msg->payload_len;
    char const * p_char = p_text;

    while (('\0' != *p_char) && ((pos + 1U) < COMMS_LINK_PAYLOAD_MAX))
    {
        p_msg->payload[pos] = *p_char;
        pos++;
        p_char++;
    }

    p_msg->payload[pos] = '\0';
    p_msg->payload_len  = (uint16_t) pos;
}


/* Sign + 32-bit magnitude: every int32_t and uint32_t value, using only
 * 32-bit division (the M0+ has no 64-bit divide). */
static void
json_number (comms_link_msg_t * p_msg, bool b_negative, uint32_t magnitude)
{
    char     digits[12];    /* "-4294967295" + terminator */
    uint32_t n    = sizeof(digits) - 1U;
    uint32_t rest = magnitude;

    digits[n] = '\0';

    do
    {
        n--;
        digits[n] = (char) ('0' + (char) (rest % 10U));
        rest     /= 10U;
    } while ((0U < rest) && (1U < n));

    if (b_negative)
    {
        n--;
        digits[n] = '-';
    }

    json_raw(p_msg, &digits[n]);
}


/* ,"key": */
static void
json_key (comms_link_msg_t * p_msg, char const * p_key)
{
    json_raw(p_msg, ",\"");
    json_raw(p_msg, p_key);
    json_raw(p_msg, "\":");
}


static void
json_string (comms_link_msg_t * p_msg, char const * p_key,
             char const * p_text)
{
    json_key(p_msg, p_key);
    json_raw(p_msg, "\"");
    json_raw(p_msg, p_text);
    json_raw(p_msg, "\"");
}

#else
/* ------------------------------------------------------------------------ *
 * Binary writer: packs bits LSB first, stops safely at the buffer end
 * ------------------------------------------------------------------------ */

static void
put_bits (codec_writer_t * p_w, uint32_t value, uint32_t bits)
{
    uint8_t * const p_bytes = (uint8_t *) p_w->p_msg->payload;
    uint32_t        i;
    uint32_t        byte;
    uint32_t        shift;

    for (i = 0U; (i < bits)
                 && ((p_w->bits / 8U) < COMMS_LINK_PAYLOAD_MAX); i++)
    {
        byte  = p_w->bits / 8U;
        shift = p_w->bits % 8U;

        if (0U == shift)
        {
            p_bytes[byte] = 0U;             /* Fresh byte. */
        }

        if (0U != ((value >> i) & 1U))
        {
            p_bytes[byte] = (uint8_t) (p_bytes[byte] | (1U << shift));
        }

        p_w->bits++;
    }

    p_w->p_msg->payload_len = (uint16_t) ((p_w->bits + 7U) / 8U);
}

#endif /* COMMS_JSON */

/* ------------------------------------------------------------------------ *
 * Public API
 * ------------------------------------------------------------------------ */

void
codec_begin (codec_writer_t * p_w, comms_link_msg_t * p_msg,
             char const * p_topic, uint8_t qos, bool b_retain,
             uint32_t time_ms)
{
    set_header(p_msg, p_topic, qos, b_retain);
    p_w->p_msg = p_msg;
    p_w->bits  = 0U;

#if COMMS_JSON
    json_raw(p_msg, "{\"v\":");
    json_number(p_msg, false, CODEC_JSON_VERSION);
    json_key(p_msg, "t");
    json_number(p_msg, false, time_ms);
#else
    put_bits(p_w, CODEC_VERSION, CODEC_VERSION_BITS);
    put_bits(p_w, time_ms, CODEC_TIME_BITS);
#endif
}


void
codec_put_uint (codec_writer_t * p_w, char const * p_key, uint32_t value,
                uint32_t bits)
{
    uint32_t const v = clamp_uint(value, bits);

#if COMMS_JSON
    json_key(p_w->p_msg, p_key);
    json_number(p_w->p_msg, false, v);
#else
    (void) p_key;
    put_bits(p_w, v, bits);
#endif
}


void
codec_put_int (codec_writer_t * p_w, char const * p_key, int32_t value,
               uint32_t bits)
{
    int32_t const v = clamp_int(value, bits);

#if COMMS_JSON
    json_key(p_w->p_msg, p_key);
    /* 0U - x gives |x| for every int32_t, including the minimum. */
    json_number(p_w->p_msg, (0 > v),
                (0 > v) ? (0U - (uint32_t) v) : (uint32_t) v);
#else
    (void) p_key;
    put_bits(p_w, (uint32_t) v, bits);     /* Two's complement, low bits. */
#endif
}


void
codec_put_name (codec_writer_t * p_w, char const * p_key,
                char const * const * p_names, uint32_t count,
                uint32_t index, uint32_t bits)
{
#if COMMS_JSON
    char const * p_text = "UNKNOWN";

    (void) bits;

    if (index < count)
    {
        p_text = p_names[index];
    }

    json_string(p_w->p_msg, p_key, p_text);
#else
    (void) p_key;
    (void) p_names;
    (void) count;
    put_bits(p_w, clamp_uint(index, bits), bits);
#endif
}


void
codec_put_symbol (codec_writer_t * p_w, char const * p_key,
                  char const * p_text, uint32_t code, uint32_t bits)
{
#if COMMS_JSON
    (void) code;
    (void) bits;
    json_string(p_w->p_msg, p_key, p_text);
#else
    (void) p_key;
    (void) p_text;
    put_bits(p_w, clamp_uint(code, bits), bits);
#endif
}


void
codec_end (codec_writer_t * p_w)
{
#if COMMS_JSON
    json_raw(p_w->p_msg, "}");
#else
    (void) p_w;                     /* Last byte already zero-padded. */
#endif
}


void
codec_text (comms_link_msg_t * p_msg, char const * p_topic, uint8_t qos,
            bool b_retain, char const * p_text)
{
    set_header(p_msg, p_topic, qos, b_retain);
    p_msg->payload_len = (uint16_t) copy_text(p_msg->payload,
                                              COMMS_LINK_PAYLOAD_MAX, p_text);
}

/* ------------------------------------------------------------------------ *
 * Packed records: explicit masks and shifts, no bit-fields (BARR-C 5.5.b)
 * ------------------------------------------------------------------------ */

/* Negative means "invalid here": the encoder sends 0, so this stores 0. */
static uint32_t
non_negative (int32_t value)
{
    return (0 > value) ? 0U : (uint32_t) value;
}


/* A clamped unsigned value in its field. */
static uint32_t
pack_uint (uint32_t value, uint32_t shift, uint32_t bits)
{
    return clamp_uint(value, bits) << shift;
}


/* A clamped two's complement value in its field. */
static uint32_t
pack_int (int32_t value, uint32_t shift, uint32_t bits)
{
    return ((uint32_t) clamp_int(value, bits) & uint_max(bits)) << shift;
}


/* An unsigned field, zero-extended. */
static uint32_t
unpack_uint (uint32_t word, uint32_t shift, uint32_t bits)
{
    return (word >> shift) & uint_max(bits);
}


/* A signed field, sign-extended from its own width. */
static int32_t
unpack_int (uint32_t word, uint32_t shift, uint32_t bits)
{
    uint32_t const mask  = uint_max(bits);
    uint32_t const raw   = (word >> shift) & mask;
    uint32_t const sign  = mask - (mask >> 1U);     /* Top bit of field   */
    int32_t        value = (int32_t) raw;

    if (0U != (raw & sign))
    {
        value = -(int32_t) ((mask - raw) + 1U);     /* Two's complement   */
    }

    return value;
}

/* ---- robot/telemetry/motion: 6 words ----
 *
 *   W0  spd_l [0:9]    spd_r [10:19]  tgt_l [20:29]
 *   W1  tgt_r [0:9]    slew_l [10:19] slew_r [20:29]
 *   W2  drive_l [0:10] drive_r [11:21]
 *   W3  enc_l [0:31]
 *   W4  enc_r [0:31]
 *   W5  dist_mm [0:31]
 *
 * Words are grouped by width (fields of one message, reordered); each word
 * has spare high bits.  The hash the radio task takes covers these words,
 * so a record with an unchanged stored value is not resent.
 *
 * The encoder counts and the travelled distance are stored full width:
 * they are boot-relative and the consumer subtracts two of them (run
 * summary, wheel difference), so clamping the stored copy would corrupt
 * the result.  The wire fields stay 19 s and 16 s and are clamped in
 * codec_put_int() at send time, exactly as before.
 */
#define M_W_SPEED            (0U)
#define M_W_TGT              (1U)
#define M_W_DRIVE            (2U)
#define M_W_ENC_L            (3U)
#define M_W_ENC_R            (4U)
#define M_W_DIST             (5U)

#define M_SPD_L_SHIFT        (0U)
#define M_SPD_R_SHIFT        (M_SPD_L_SHIFT + CODEC_BITS_SPEED)
#define M_TGT_L_SHIFT        (M_SPD_R_SHIFT + CODEC_BITS_SPEED)
#define M_TGT_R_SHIFT        (0U)
#define M_SLEW_L_SHIFT       (M_TGT_R_SHIFT + CODEC_BITS_SPEED)
#define M_SLEW_R_SHIFT       (M_SLEW_L_SHIFT + CODEC_BITS_SPEED)
#define M_DRIVE_L_SHIFT      (0U)
#define M_DRIVE_R_SHIFT      (M_DRIVE_L_SHIFT + CODEC_BITS_DRIVE)


void
codec_pack_motion (uint32_t * p_words, comms_motion_t const * p_motion)
{
    p_words[M_W_SPEED] =
        pack_int(p_motion->speed_left_mm_s, M_SPD_L_SHIFT,
                 CODEC_BITS_SPEED)
        | pack_int(p_motion->speed_right_mm_s, M_SPD_R_SHIFT,
                   CODEC_BITS_SPEED)
        | pack_int(p_motion->target_left_mm_s, M_TGT_L_SHIFT,
                   CODEC_BITS_SPEED);
    p_words[M_W_TGT] =
        pack_int(p_motion->target_right_mm_s, M_TGT_R_SHIFT,
                 CODEC_BITS_SPEED)
        | pack_int(p_motion->slewed_left_mm_s, M_SLEW_L_SHIFT,
                   CODEC_BITS_SPEED)
        | pack_int(p_motion->slewed_right_mm_s, M_SLEW_R_SHIFT,
                   CODEC_BITS_SPEED);
    p_words[M_W_DRIVE] =
        pack_int(p_motion->drive_left_permille, M_DRIVE_L_SHIFT,
                 CODEC_BITS_DRIVE)
        | pack_int(p_motion->drive_right_permille, M_DRIVE_R_SHIFT,
                   CODEC_BITS_DRIVE);
    p_words[M_W_ENC_L] = (uint32_t) p_motion->encoder_left;
    p_words[M_W_ENC_R] = (uint32_t) p_motion->encoder_right;
    p_words[M_W_DIST]  = (uint32_t) p_motion->distance_mm;
}


void
codec_unpack_motion (uint32_t const * p_words, comms_motion_t * p_motion)
{
    p_motion->speed_left_mm_s     = unpack_int(p_words[M_W_SPEED],
                                               M_SPD_L_SHIFT,
                                               CODEC_BITS_SPEED);
    p_motion->speed_right_mm_s    = unpack_int(p_words[M_W_SPEED],
                                               M_SPD_R_SHIFT,
                                               CODEC_BITS_SPEED);
    p_motion->target_left_mm_s    = unpack_int(p_words[M_W_SPEED],
                                               M_TGT_L_SHIFT,
                                               CODEC_BITS_SPEED);
    p_motion->target_right_mm_s   = unpack_int(p_words[M_W_TGT],
                                               M_TGT_R_SHIFT,
                                               CODEC_BITS_SPEED);
    p_motion->slewed_left_mm_s    = unpack_int(p_words[M_W_TGT],
                                               M_SLEW_L_SHIFT,
                                               CODEC_BITS_SPEED);
    p_motion->slewed_right_mm_s   = unpack_int(p_words[M_W_TGT],
                                               M_SLEW_R_SHIFT,
                                               CODEC_BITS_SPEED);
    p_motion->drive_left_permille = unpack_int(p_words[M_W_DRIVE],
                                               M_DRIVE_L_SHIFT,
                                               CODEC_BITS_DRIVE);
    p_motion->drive_right_permille = unpack_int(p_words[M_W_DRIVE],
                                                M_DRIVE_R_SHIFT,
                                                CODEC_BITS_DRIVE);
    p_motion->encoder_left        = (int32_t) p_words[M_W_ENC_L];
    p_motion->encoder_right       = (int32_t) p_words[M_W_ENC_R];
    p_motion->distance_mm         = (int32_t) p_words[M_W_DIST];
}

/* ---- robot/telemetry/line: 2 words ----
 *
 *   W0  pos [0:2]  err [3:10]  sensors [11:12]  centre_adc [13:24]
 *   W1  right_adc [0:11]
 */
#define L_W_A                (0U)
#define L_W_B                (1U)

#define L_POS_SHIFT          (0U)
#define L_ERR_SHIFT          (L_POS_SHIFT + CODEC_BITS_LINE_POS)
#define L_SENSORS_SHIFT      (L_ERR_SHIFT + CODEC_BITS_LINE_ERROR)
#define L_CENTRE_SHIFT       (L_SENSORS_SHIFT + CODEC_BITS_SENSORS)
#define L_RIGHT_SHIFT        (0U)


void
codec_pack_line (uint32_t * p_words, comms_line_t const * p_line)
{
    p_words[L_W_A] =
        pack_uint((uint32_t) p_line->position, L_POS_SHIFT,
                  CODEC_BITS_LINE_POS)
        | pack_int(p_line->error, L_ERR_SHIFT, CODEC_BITS_LINE_ERROR)
        | pack_uint((uint32_t) p_line->sensors, L_SENSORS_SHIFT,
                    CODEC_BITS_SENSORS)
        | pack_uint((uint32_t) p_line->centre_adc, L_CENTRE_SHIFT,
                    CODEC_BITS_ADC);
    p_words[L_W_B] = pack_uint((uint32_t) p_line->right_adc, L_RIGHT_SHIFT,
                               CODEC_BITS_ADC);
}


void
codec_unpack_line (uint32_t const * p_words, comms_line_t * p_line)
{
    p_line->position   = (comms_line_pos_t) unpack_uint(p_words[L_W_A],
                                                        L_POS_SHIFT,
                                                        CODEC_BITS_LINE_POS);
    p_line->error      = unpack_int(p_words[L_W_A], L_ERR_SHIFT,
                                    CODEC_BITS_LINE_ERROR);
    p_line->sensors    = (uint8_t) unpack_uint(p_words[L_W_A],
                                               L_SENSORS_SHIFT,
                                               CODEC_BITS_SENSORS);
    p_line->centre_adc = (uint16_t) unpack_uint(p_words[L_W_A],
                                                L_CENTRE_SHIFT,
                                                CODEC_BITS_ADC);
    p_line->right_adc  = (uint16_t) unpack_uint(p_words[L_W_B],
                                                L_RIGHT_SHIFT,
                                                CODEC_BITS_ADC);
}

/* ---- robot/telemetry/terrain: 2 words ----
 *
 *   W0  motion [0:2]  tilt_mg [3:13]  hump_dmm [14:23]
 *   W1  turn_dps [0:10]
 */
#define T_W_A                (0U)
#define T_W_B                (1U)

#define T_MOTION_SHIFT       (0U)
#define T_TILT_SHIFT         (T_MOTION_SHIFT + CODEC_BITS_MOTION_CLASS)
#define T_HUMP_SHIFT         (T_TILT_SHIFT + CODEC_BITS_TILT)
#define T_TURN_SHIFT         (0U)


void
codec_pack_terrain (uint32_t * p_words,
                    comms_terrain_t const * p_terrain)
{
    p_words[T_W_A] =
        pack_uint((uint32_t) p_terrain->motion, T_MOTION_SHIFT,
                  CODEC_BITS_MOTION_CLASS)
        | pack_int(p_terrain->tilt_mg, T_TILT_SHIFT, CODEC_BITS_TILT)
        | pack_uint(non_negative(p_terrain->hump_dmm), T_HUMP_SHIFT,
                    CODEC_BITS_HUMP);
    p_words[T_W_B] = pack_int(p_terrain->turn_dps, T_TURN_SHIFT,
                              CODEC_BITS_TURN);
}


void
codec_unpack_terrain (uint32_t const * p_words,
                      comms_terrain_t * p_terrain)
{
    p_terrain->motion   = (comms_motion_class_t) unpack_uint(p_words[T_W_A],
                                                             T_MOTION_SHIFT,
                                                       CODEC_BITS_MOTION_CLASS);
    p_terrain->tilt_mg  = unpack_int(p_words[T_W_A], T_TILT_SHIFT,
                                     CODEC_BITS_TILT);
    p_terrain->hump_dmm = (int32_t) unpack_uint(p_words[T_W_A],
                                                T_HUMP_SHIFT,
                                                CODEC_BITS_HUMP);
    p_terrain->turn_dps = unpack_int(p_words[T_W_B], T_TURN_SHIFT,
                                     CODEC_BITS_TURN);
}

/* ---- Queued event: 7 words ----
 *
 *   W0  time_ms [0:31]
 *
 *   W1  type [0:2], then by type:
 *         barcode     code [3:10] (raw character), nav [11:13]
 *         hump        peak_dmm [3:12]
 *         motion      motion class [3:5]
 *         run         no payload: the type is BEGIN or END
 *         obstacle    no payload (see W2, W3)
 *         calibration result [3:5], centre_low [6:17],
 *                     centre_high [18:29], learned [30],
 *                     first_right [31]
 *
 *   W2  obstacle: dist [0:11], angle [12:19], width [20:29]
 *       calibration: centre_thr [0:11], centre_full [12:23],
 *                    sweep_deg [24:31]
 *   W3  obstacle: clear_l [0:11], clear_r [12:23], action [24:26]
 *       calibration: right_low [0:11], right_high [12:23]
 *   W4  calibration: right_thr [0:11], barcode_low [12:23]
 *   W5  calibration: barcode_high [0:11], b4_samples [12:26]
 *   W6  calibration: barcode_thr [0:11], overlap [12:20] (signed),
 *                    b4_floor_mg [21:29]
 *
 * The type-specific fields of W1 overlap (only one type is stored per
 * element).  Unused words stay zero, so the element has no defined junk.
 */
#define E_W_TIME             (0U)
#define E_W_TYPE             (1U)
#define E_W_A                (2U)
#define E_W_B                (3U)
#define E_W_C                (4U)
#define E_W_D                (5U)
#define E_W_E                (6U)

#define E_CODE_BITS          (8U)     /* raw barcode character          */

#define E_TYPE_SHIFT         (0U)
#define E_CODE_SHIFT         (3U)
#define E_NAV_SHIFT          (E_CODE_SHIFT + E_CODE_BITS)
#define E_PEAK_SHIFT         (3U)
#define E_CLASS_SHIFT        (3U)

#define E_DIST_SHIFT         (0U)
#define E_ANGLE_SHIFT        (E_DIST_SHIFT + CODEC_BITS_OBST_DIST)
#define E_WIDTH_SHIFT        (E_ANGLE_SHIFT + CODEC_BITS_ANGLE)
#define E_CLEAR_L_SHIFT      (0U)
#define E_CLEAR_R_SHIFT      (E_CLEAR_L_SHIFT + CODEC_BITS_CLEARANCE)
#define E_ACTION_SHIFT       (E_CLEAR_R_SHIFT + CODEC_BITS_CLEARANCE)

#define E_CAL_RESULT_SHIFT   (3U)
#define E_CAL_C_LOW_SHIFT    (E_CAL_RESULT_SHIFT + CODEC_BITS_CAL_RESULT)
#define E_CAL_C_HIGH_SHIFT   (E_CAL_C_LOW_SHIFT + CODEC_BITS_ADC)
#define E_CAL_LEARNED_SHIFT  (E_CAL_C_HIGH_SHIFT + CODEC_BITS_ADC)
#define E_CAL_FIRST_R_SHIFT  (E_CAL_LEARNED_SHIFT + CODEC_BITS_CAL_LEARNED)
#define E_CAL_C_THR_SHIFT    (0U)
#define E_CAL_C_FULL_SHIFT   (E_CAL_C_THR_SHIFT + CODEC_BITS_ADC)
#define E_CAL_SWEEP_SHIFT    (E_CAL_C_FULL_SHIFT + CODEC_BITS_ADC)
#define E_CAL_R_LOW_SHIFT    (0U)
#define E_CAL_R_HIGH_SHIFT   (E_CAL_R_LOW_SHIFT + CODEC_BITS_ADC)
#define E_CAL_R_THR_SHIFT    (0U)
#define E_CAL_B_LOW_SHIFT    (E_CAL_R_THR_SHIFT + CODEC_BITS_ADC)
#define E_CAL_B_HIGH_SHIFT   (0U)
#define E_CAL_SAMPLES_SHIFT  (E_CAL_B_HIGH_SHIFT + CODEC_BITS_ADC)
#define E_CAL_B_THR_SHIFT    (0U)
#define E_CAL_OVERLAP_SHIFT  (E_CAL_B_THR_SHIFT + CODEC_BITS_ADC)
#define E_CAL_FLOOR_SHIFT    (E_CAL_OVERLAP_SHIFT + CODEC_BITS_CAL_OVERLAP)


static void
pack_calibration (uint32_t * p_words,
                  comms_calibration_t const * p_cal)
{
    p_words[E_W_TYPE] |=
        pack_uint((uint32_t) p_cal->result, E_CAL_RESULT_SHIFT,
                  CODEC_BITS_CAL_RESULT)
        | pack_uint((uint32_t) p_cal->centre_low, E_CAL_C_LOW_SHIFT,
                    CODEC_BITS_ADC)
        | pack_uint((uint32_t) p_cal->centre_high, E_CAL_C_HIGH_SHIFT,
                    CODEC_BITS_ADC)
        | pack_uint(p_cal->b_barcode_learned ? 1U : 0U,
                    E_CAL_LEARNED_SHIFT, CODEC_BITS_CAL_LEARNED)
        | pack_uint(p_cal->b_first_right ? 1U : 0U, E_CAL_FIRST_R_SHIFT,
                    CODEC_BITS_CAL_FIRST_RIGHT);
    p_words[E_W_A] |=
        pack_uint((uint32_t) p_cal->centre_threshold, E_CAL_C_THR_SHIFT,
                  CODEC_BITS_ADC)
        | pack_uint((uint32_t) p_cal->centre_full_black, E_CAL_C_FULL_SHIFT,
                    CODEC_BITS_ADC)
        | pack_uint(non_negative(p_cal->sweep_deg), E_CAL_SWEEP_SHIFT,
                    CODEC_BITS_CAL_ANGLE);
    p_words[E_W_B] |=
        pack_uint((uint32_t) p_cal->right_low, E_CAL_R_LOW_SHIFT,
                  CODEC_BITS_ADC)
        | pack_uint((uint32_t) p_cal->right_high, E_CAL_R_HIGH_SHIFT,
                    CODEC_BITS_ADC);
    p_words[E_W_C] |=
        pack_uint((uint32_t) p_cal->right_threshold, E_CAL_R_THR_SHIFT,
                  CODEC_BITS_ADC)
        | pack_uint((uint32_t) p_cal->barcode_low, E_CAL_B_LOW_SHIFT,
                    CODEC_BITS_ADC);
    p_words[E_W_D] |=
        pack_uint((uint32_t) p_cal->barcode_high, E_CAL_B_HIGH_SHIFT,
                  CODEC_BITS_ADC)
        | pack_uint(p_cal->b4_samples, E_CAL_SAMPLES_SHIFT,
                    CODEC_BITS_CAL_SAMPLES);
    p_words[E_W_E] |=
        pack_uint((uint32_t) p_cal->barcode_threshold, E_CAL_B_THR_SHIFT,
                  CODEC_BITS_ADC)
        | pack_int(p_cal->overlap_samples, E_CAL_OVERLAP_SHIFT,
                   CODEC_BITS_CAL_OVERLAP)
        | pack_uint(p_cal->b4_floor_mg, E_CAL_FLOOR_SHIFT,
                    CODEC_BITS_CAL_FLOOR);
}


static void
unpack_calibration (uint32_t const * p_words, comms_calibration_t * p_cal)
{
    p_cal->result = (comms_cal_result_t) unpack_uint(p_words[E_W_TYPE],
                                                     E_CAL_RESULT_SHIFT,
                                                     CODEC_BITS_CAL_RESULT);
    p_cal->centre_low = (uint16_t) unpack_uint(p_words[E_W_TYPE],
                                               E_CAL_C_LOW_SHIFT,
                                               CODEC_BITS_ADC);
    p_cal->centre_high = (uint16_t) unpack_uint(p_words[E_W_TYPE],
                                                E_CAL_C_HIGH_SHIFT,
                                                CODEC_BITS_ADC);
    p_cal->b_barcode_learned = (0U != unpack_uint(p_words[E_W_TYPE],
                                                  E_CAL_LEARNED_SHIFT,
                                                CODEC_BITS_CAL_LEARNED));
    p_cal->b_first_right = (0U != unpack_uint(p_words[E_W_TYPE],
                                              E_CAL_FIRST_R_SHIFT,
                                              CODEC_BITS_CAL_FIRST_RIGHT));
    p_cal->centre_threshold = (uint16_t) unpack_uint(p_words[E_W_A],
                                                     E_CAL_C_THR_SHIFT,
                                                     CODEC_BITS_ADC);
    p_cal->centre_full_black = (uint16_t) unpack_uint(p_words[E_W_A],
                                                      E_CAL_C_FULL_SHIFT,
                                                      CODEC_BITS_ADC);
    p_cal->sweep_deg = (int32_t) unpack_uint(p_words[E_W_A],
                                             E_CAL_SWEEP_SHIFT,
                                             CODEC_BITS_CAL_ANGLE);
    p_cal->right_low = (uint16_t) unpack_uint(p_words[E_W_B],
                                              E_CAL_R_LOW_SHIFT,
                                              CODEC_BITS_ADC);
    p_cal->right_high = (uint16_t) unpack_uint(p_words[E_W_B],
                                               E_CAL_R_HIGH_SHIFT,
                                               CODEC_BITS_ADC);
    p_cal->right_threshold = (uint16_t) unpack_uint(p_words[E_W_C],
                                                    E_CAL_R_THR_SHIFT,
                                                    CODEC_BITS_ADC);
    p_cal->barcode_low = (uint16_t) unpack_uint(p_words[E_W_C],
                                                E_CAL_B_LOW_SHIFT,
                                                CODEC_BITS_ADC);
    p_cal->barcode_high = (uint16_t) unpack_uint(p_words[E_W_D],
                                                 E_CAL_B_HIGH_SHIFT,
                                                 CODEC_BITS_ADC);
    p_cal->b4_samples = unpack_uint(p_words[E_W_D], E_CAL_SAMPLES_SHIFT,
                                    CODEC_BITS_CAL_SAMPLES);
    p_cal->barcode_threshold = (uint16_t) unpack_uint(p_words[E_W_E],
                                                      E_CAL_B_THR_SHIFT,
                                                      CODEC_BITS_ADC);
    p_cal->overlap_samples = unpack_int(p_words[E_W_E], E_CAL_OVERLAP_SHIFT,
                                        CODEC_BITS_CAL_OVERLAP);
    p_cal->b4_floor_mg = unpack_uint(p_words[E_W_E], E_CAL_FLOOR_SHIFT,
                                     CODEC_BITS_CAL_FLOOR);
}


void
codec_pack_event (uint32_t * p_words, codec_event_t const * p_event)
{
    uint32_t i;

    for (i = 0U; i < CODEC_EVENT_WORDS; i++)
    {
        p_words[i] = 0U;                /* Fields the type does not use  */
    }

    p_words[E_W_TIME] = p_event->time_ms;
    p_words[E_W_TYPE] = pack_uint((uint32_t) p_event->type, E_TYPE_SHIFT,
                                  CODEC_BITS_EVENT_TYPE);

    switch ((codec_event_type_t) p_event->type)
    {
        case CODEC_EVENT_BARCODE:
            p_words[E_W_TYPE] |= pack_uint((uint32_t) p_event->code,
                                           E_CODE_SHIFT, E_CODE_BITS)
                                 | pack_uint(p_event->kind, E_NAV_SHIFT,
                                             CODEC_BITS_NAV);
            break;

        case CODEC_EVENT_HUMP:
            p_words[E_W_TYPE] |= pack_uint(non_negative(p_event->value),
                                           E_PEAK_SHIFT, CODEC_BITS_HUMP);
            break;

        case CODEC_EVENT_OBSTACLE:
            p_words[E_W_A] |=
                pack_uint(non_negative(p_event->obstacle.distance_mm),
                          E_DIST_SHIFT, CODEC_BITS_OBST_DIST)
                | pack_uint(non_negative(p_event->obstacle.angle_deg),
                            E_ANGLE_SHIFT, CODEC_BITS_ANGLE)
                | pack_uint(non_negative(p_event->obstacle.width_mm),
                            E_WIDTH_SHIFT, CODEC_BITS_WIDTH);
            p_words[E_W_B] |=
                pack_uint(non_negative(p_event->obstacle.clear_left_mm),
                          E_CLEAR_L_SHIFT, CODEC_BITS_CLEARANCE)
                | pack_uint(non_negative(p_event->obstacle.clear_right_mm),
                            E_CLEAR_R_SHIFT, CODEC_BITS_CLEARANCE)
                | pack_uint((uint32_t) p_event->obstacle.action,
                            E_ACTION_SHIFT, CODEC_BITS_ACTION);
            break;

        case CODEC_EVENT_MOTION:
            p_words[E_W_TYPE] |= pack_uint(p_event->kind, E_CLASS_SHIFT,
                                           CODEC_BITS_MOTION_CLASS);
            break;

        case CODEC_EVENT_CALIBRATION:
            pack_calibration(p_words, &p_event->calibration);
            break;

        case CODEC_EVENT_RUN_BEGIN:
        case CODEC_EVENT_RUN_END:
        case CODEC_EVENT_COUNT:
        default:
            /* The type alone is the payload. */
            break;
    }
}


bool
codec_unpack_event (uint32_t const * p_words, codec_event_t * p_event)
{
    comms_obstacle_t const    no_obstacle    = {0};
    comms_calibration_t const no_calibration = {0};
    uint32_t const type = unpack_uint(p_words[E_W_TYPE], E_TYPE_SHIFT,
                                      CODEC_BITS_EVENT_TYPE);
    bool           b_ok = false;

    p_event->time_ms      = p_words[E_W_TIME];
    p_event->type         = (uint8_t) type;
    p_event->code         = 0U;
    p_event->kind         = 0U;
    p_event->value        = 0;
    p_event->obstacle     = no_obstacle;
    p_event->calibration  = no_calibration;

    switch ((codec_event_type_t) type)
    {
        case CODEC_EVENT_BARCODE:
            p_event->code = (uint8_t) unpack_uint(p_words[E_W_TYPE],
                                                  E_CODE_SHIFT,
                                                  E_CODE_BITS);
            p_event->kind = (uint8_t) unpack_uint(p_words[E_W_TYPE],
                                                  E_NAV_SHIFT,
                                                  CODEC_BITS_NAV);
            b_ok          = true;
            break;

        case CODEC_EVENT_HUMP:
            p_event->value = (int32_t) unpack_uint(p_words[E_W_TYPE],
                                                   E_PEAK_SHIFT,
                                                   CODEC_BITS_HUMP);
            b_ok           = true;
            break;

        case CODEC_EVENT_OBSTACLE:
            p_event->obstacle.distance_mm = (int32_t) unpack_uint(
                p_words[E_W_A], E_DIST_SHIFT, CODEC_BITS_OBST_DIST);
            p_event->obstacle.angle_deg = (int32_t) unpack_uint(
                p_words[E_W_A], E_ANGLE_SHIFT, CODEC_BITS_ANGLE);
            p_event->obstacle.width_mm = (int32_t) unpack_uint(
                p_words[E_W_A], E_WIDTH_SHIFT, CODEC_BITS_WIDTH);
            p_event->obstacle.clear_left_mm = (int32_t) unpack_uint(
                p_words[E_W_B], E_CLEAR_L_SHIFT, CODEC_BITS_CLEARANCE);
            p_event->obstacle.clear_right_mm = (int32_t) unpack_uint(
                p_words[E_W_B], E_CLEAR_R_SHIFT, CODEC_BITS_CLEARANCE);
            p_event->obstacle.action = (comms_avoid_t) unpack_uint(
                p_words[E_W_B], E_ACTION_SHIFT, CODEC_BITS_ACTION);
            b_ok = true;
            break;

        case CODEC_EVENT_MOTION:
            p_event->kind = (uint8_t) unpack_uint(p_words[E_W_TYPE],
                                                  E_CLASS_SHIFT,
                                                  CODEC_BITS_MOTION_CLASS);
            b_ok          = true;
            break;

        case CODEC_EVENT_RUN_BEGIN:
        case CODEC_EVENT_RUN_END:
            b_ok = true;
            break;

        case CODEC_EVENT_CALIBRATION:
            unpack_calibration(p_words, &p_event->calibration);
            b_ok = true;
            break;

        case CODEC_EVENT_COUNT:
        default:
            b_ok = false;       /* Unknown type: the caller drops it.   */
            break;
    }

    return b_ok;
}

/*** end of file ***/
