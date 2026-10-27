/** @file comms_codec.c
 *
 * @brief Binary / JSON message encoder (see comms_codec.h).
 *
 * Only the format selected by COMMS_JSON is compiled in.
 */

#include <stdint.h>
#include <stdbool.h>
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

/*** end of file ***/
