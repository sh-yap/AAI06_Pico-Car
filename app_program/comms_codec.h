/** @file comms_codec.h
 *
 * @brief Message encoder for the comms module: packed binary (default,
 *        CR1 minimum-bit representation) or JSON (fallback / debugging).
 *
 * The format is chosen at build time: COMMS_FORMAT=binary (default) or
 * COMMS_FORMAT=json, which sets COMMS_JSON.  Only the chosen format is
 * compiled in, so the binary firmware carries no JSON code or key text.
 *
 * Each message is described once, with each field's JSON key and binary
 * bit width.  Wrap keys in CODEC_KEY() and name tables in CODEC_NAMES()
 * so that a binary build drops the text:
 *
 *     codec_begin(&w, p_msg, "robot/heartbeat", QOS_0, false, now);
 *     codec_put_uint(&w, CODEC_KEY("seq"), seq & 0xFFFFU, 16U);
 *     codec_put_name(&w, CODEC_KEY("state"), CODEC_NAMES(g_state_names),
 *                    COMMS_STATE_COUNT, state, 4U);
 *     codec_end(&w);
 *
 * Binary: 2-bit version (2), 32-bit time, then each field in its own width,
 *         least significant bit first (docs/BINARY_FORMAT.md).
 * JSON:   {"v":1,"t":<ms>,"seq":12,"state":"READY"}
 *
 * Values outside a field's range are clamped to the nearest end in BOTH
 * formats, so a decoded binary message equals its JSON twin.
 *
 * Pure logic: no RTOS, no hardware, host-testable (tests/comms).
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "comms_link.h" before this header.
 */

#ifndef COMMS_CODEC_H
#define COMMS_CODEC_H

#ifndef COMMS_JSON
#define COMMS_JSON             (0)      /* 0 = binary (default), 1 = JSON. */
#endif

#if COMMS_JSON
#define CODEC_KEY(key)         (key)
#define CODEC_NAMES(names)     (names)
#else
#define CODEC_KEY(key)         ((char const *) 0)   /* Text not compiled in. */
#define CODEC_NAMES(names)     ((char const * const *) 0)
#endif

#define CODEC_JSON_VERSION     (1U)
#define CODEC_VERSION          (2U)     /* Binary format version.          */
#define CODEC_VERSION_BITS     (2U)
#define CODEC_TIME_BITS        (32U)
#define CODEC_HEADER_BITS      (CODEC_VERSION_BITS + CODEC_TIME_BITS)

/* One message being written.  Fill with codec_begin(); never by hand. */
typedef struct
{
    comms_link_msg_t * p_msg;
    uint32_t           bits;       /* Binary: bits written so far.       */
} codec_writer_t;

/* Start a message: topic, delivery options, version and time. */
void codec_begin(codec_writer_t * p_w, comms_link_msg_t * p_msg,
                 char const * p_topic, uint8_t qos, bool b_retain,
                 uint32_t time_ms);

/* Unsigned field, 1..32 bits; clamped to 0 .. 2^bits - 1. */
void codec_put_uint(codec_writer_t * p_w, char const * p_key,
                    uint32_t value, uint32_t bits);

/* Signed field, 2..32 bits, two's complement; clamped to its range. */
void codec_put_int(codec_writer_t * p_w, char const * p_key,
                   int32_t value, uint32_t bits);

/* Enum field: binary writes the index (clamped); JSON writes
 * p_names[index], or "UNKNOWN" if out of range. */
void codec_put_name(codec_writer_t * p_w, char const * p_key,
                    char const * const * p_names, uint32_t count,
                    uint32_t index, uint32_t bits);

/* Field that is a small code in binary and text in JSON (barcode). */
void codec_put_symbol(codec_writer_t * p_w, char const * p_key,
                      char const * p_text, uint32_t code, uint32_t bits);

/* Finish the message (JSON: closing brace; binary: nothing to add). */
void codec_end(codec_writer_t * p_w);

/* A plain-text message in either format (robot/info). */
void codec_text(comms_link_msg_t * p_msg, char const * p_topic, uint8_t qos,
                bool b_retain, char const * p_text);

#endif /* COMMS_CODEC_H */

/*** end of file ***/
