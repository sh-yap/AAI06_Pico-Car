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
 * The packed records at the end of this header are the same fields held in
 * fixed 32-bit words, for data that is queued or shared between tasks.
 *
 * Pure logic: no RTOS, no hardware, host-testable (tests/comms).
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h>,
 * "comms_link.h" and "comms.h" before this header.
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

/* ------------------------------------------------------------------------ *
 * Packed records: minimum-bit storage for queued and shared data
 *
 * A packed record is a fixed number of 32-bit words.  Bit 0 is the LSB, a
 * field never crosses a word boundary, and the layout is written with
 * explicit masks and shifts, never with C bit-fields (BARR-C 5.5.b).  A
 * stored field is normally the message width, so packing clamps to the
 * field ends like the encoder.  Fields the consumer reads back and
 * subtracts (encoder counts, travelled distance) keep a full 32-bit word
 * and are narrowed only on the wire, at send time.  Signed fields are two's
 * complement in their own width and are sign-extended on unpack; negative
 * "invalid" values are stored as 0, as the encoder sends them
 * (docs/DATA_REPRESENTATION.md sections 7, 8 and 11).
 *
 * Each record has one writer task; the words are copied whole (a single
 * aligned store each), under the caller's seqlock or message buffer.
 * ------------------------------------------------------------------------ */

/* Field widths in bits, shared by the messages and the packed records so
 * the two can never drift apart. */
#define CODEC_BITS_SPEED              (10U)   /* signed                */
#define CODEC_BITS_DRIVE              (11U)   /* signed, permille      */
#define CODEC_BITS_ENCODER            (19U)   /* signed, counts        */
#define CODEC_BITS_DISTANCE           (16U)   /* signed, mm            */
#define CODEC_BITS_LINE_POS           (3U)
#define CODEC_BITS_LINE_ERROR         (8U)    /* signed                */
#define CODEC_BITS_SENSORS            (2U)
#define CODEC_BITS_ADC                (12U)   /* RP2040 ADC reading    */
#define CODEC_BITS_MOTION_CLASS       (3U)
#define CODEC_BITS_TILT               (11U)   /* signed, milli-g       */
#define CODEC_BITS_HUMP               (10U)   /* 0.1 mm                */
#define CODEC_BITS_TURN               (11U)   /* signed, deg/s         */
#define CODEC_BITS_BARCODE            (6U)    /* 0 none, 1..43, 44 bad */
#define CODEC_BITS_NAV                (3U)
#define CODEC_BITS_OBST_DIST          (12U)   /* mm                    */
#define CODEC_BITS_ANGLE              (8U)    /* deg                   */
#define CODEC_BITS_WIDTH              (10U)   /* mm                    */
#define CODEC_BITS_CLEARANCE          (12U)   /* mm                    */
#define CODEC_BITS_ACTION             (3U)
#define CODEC_BITS_EDGE_RESULT        (2U)    /* comms_edge_result_t   */
#define CODEC_BITS_SIDE               (1U)    /* comms_side_t          */
#define CODEC_BITS_EDGE               (10U)   /* mm                    */
#define CODEC_BITS_EST_WIDTH          (11U)   /* mm                    */
#define CODEC_BITS_LEG                (11U)   /* mm                    */
#define CODEC_BITS_TURN_DEG           (6U)    /* whole degrees         */
#define CODEC_BITS_RANGE              (3U)    /* comms_range_t         */
#define CODEC_BITS_SONAR_STATE        (3U)    /* comms_sonar_state_t   */
#define CODEC_BITS_SILENT             (8U)    /* wrapping count        */
#define CODEC_BITS_CAL_RESULT         (3U)
#define CODEC_BITS_CAL_OVERLAP        (9U)    /* signed, -256 = none   */
#define CODEC_BITS_CAL_LEARNED        (1U)
#define CODEC_BITS_CAL_FIRST_RIGHT    (1U)
#define CODEC_BITS_CAL_FLOOR          (9U)    /* milli-g               */
#define CODEC_BITS_CAL_SAMPLES        (15U)
#define CODEC_BITS_CAL_ANGLE          (8U)    /* whole degrees         */
#define CODEC_BITS_EVENT_TYPE         (3U)    /* queued element only   */

/* Record sizes in words (old storage in the comment). */
#define CODEC_MOTION_WORDS            (6U)    /* 24 B, was 44 B        */
#define CODEC_LINE_WORDS              (2U)    /*  8 B, was 16 B        */
#define CODEC_TERRAIN_WORDS           (2U)    /*  8 B, was 16 B        */
#define CODEC_SONAR_WORDS             (1U)    /*  4 B, new             */
#define CODEC_EVENT_WORDS             (7U)    /* 28 B, was 76 B        */

/* Event codes: the queued element's type field (7 types, 3 bits). */
typedef enum
{
    CODEC_EVENT_BARCODE = 0,
    CODEC_EVENT_HUMP,
    CODEC_EVENT_OBSTACLE,
    CODEC_EVENT_MOTION,
    CODEC_EVENT_RUN_BEGIN,
    CODEC_EVENT_RUN_END,
    CODEC_EVENT_CALIBRATION,
    CODEC_EVENT_COUNT
} codec_event_type_t;

/* CODEC_EVENT_OBSTACLE carries one of two records, chosen by the event's
 * kind field: the scan profile (robot/event/obstacle) or the width
 * estimate from the edge watch (robot/event/obstacle_width). */
#define CODEC_OBSTACLE_PROFILE        (0U)
#define CODEC_OBSTACLE_WIDTH          (1U)

/* One queued event, unpacked (the queue element itself is 7 words). */
typedef struct
{
    uint32_t            time_ms;
    uint8_t             type;      /* codec_event_type_t               */
    uint8_t             code;      /* barcode character                */
    uint8_t             kind;      /* nav command / motion class /
                                      CODEC_OBSTACLE_*                 */
    int32_t             value;     /* hump peak height, 0.1 mm         */
    comms_obstacle_t    obstacle;
    comms_obstacle_width_t obstacle_width;
    comms_calibration_t calibration;
} codec_event_t;

/* Motion telemetry record: 6 words, robot/telemetry/motion fields. */
void codec_pack_motion(uint32_t * p_words, comms_motion_t const * p_motion);
void codec_unpack_motion(uint32_t const * p_words, comms_motion_t * p_motion);

/* Line telemetry record: 2 words, robot/telemetry/line fields. */
void codec_pack_line(uint32_t * p_words, comms_line_t const * p_line);
void codec_unpack_line(uint32_t const * p_words, comms_line_t * p_line);

/* Terrain telemetry record: 2 words, robot/telemetry/terrain fields. */
void codec_pack_terrain(uint32_t * p_words,
                        comms_terrain_t const * p_terrain);
void codec_unpack_terrain(uint32_t const * p_words,
                          comms_terrain_t * p_terrain);

/* Ultrasonic telemetry record: 1 word, robot/telemetry/obstacle fields. */
void codec_pack_sonar(uint32_t * p_words, comms_sonar_t const * p_sonar);
void codec_unpack_sonar(uint32_t const * p_words, comms_sonar_t * p_sonar);

/* Queued event element: 7 words.  Unpack returns false for an unknown event
 * type: the caller drops the element (the out-of-range rule: reject, never
 * guess). */
void codec_pack_event(uint32_t * p_words, codec_event_t const * p_event);
bool codec_unpack_event(uint32_t const * p_words, codec_event_t * p_event);

#endif /* COMMS_CODEC_H */

/*** end of file ***/
