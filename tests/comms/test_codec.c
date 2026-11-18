/** @file test_codec.c
 *
 * @brief Host unit tests for the message encoder (comms_codec.c).
 *
 * Built twice by the Makefile: binary (default) and JSON (-DCOMMS_JSON=1),
 * so both formats are always tested.
 *
 * Binary output is checked two ways: exact bytes worked out by hand, and a
 * round trip through an independent bit reader for every field width.  The
 * golden messages are locked; the laptop decoder test
 * (tools/test_robot_decode.ps1) decodes the same bytes.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "comms.h"
#include "comms_link.h"
#include "comms_codec.h"

static uint32_t g_checks;
static uint32_t g_failures;

#define CHECK(cond)                                                         \
    do                                                                      \
    {                                                                       \
        g_checks++;                                                         \
        if (!(cond))                                                        \
        {                                                                   \
            g_failures++;                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                   \
    } while (0)

/* Message with guard bytes after it, to catch any overrun. */
typedef struct
{
    comms_link_msg_t msg;
    uint8_t          guard[16];
} guarded_msg_t;

static guarded_msg_t g_m;

#if COMMS_JSON
/* Binary builds never reference name tables (CODEC_NAMES drops them). */
static char const * const g_names[] = { "ZERO", "ONE", "TWO" };
#endif

static void
fresh (void)
{
    memset(&g_m, 0xA5, sizeof(g_m));
}


static bool
guard_intact (void)
{
    uint32_t i;
    bool     b_ok = true;

    for (i = 0U; i < sizeof(g_m.guard); i++)
    {
        b_ok = b_ok && (0xA5U == g_m.guard[i]);
    }

    return b_ok;
}


/* Format-independent: plain-text messages and buffer safety. */
static void
test_text (void)
{
    char           text[400];
    codec_writer_t w;
    uint32_t       i;

    printf("plain text message + overflow safety\n");

    fresh();
    codec_text(&g_m.msg, "robot/info", 1U, true, "Oct 27 2026 10:00:00");
    CHECK(0 == strcmp("robot/info", g_m.msg.topic));
    CHECK(0 == strcmp("Oct 27 2026 10:00:00", g_m.msg.payload));
    CHECK(20U == g_m.msg.payload_len);
    CHECK(1U == g_m.msg.b_retain);

    memset(text, 'x', sizeof(text) - 1U);
    text[sizeof(text) - 1U] = '\0';
    fresh();
    codec_text(&g_m.msg, text, 0U, false, text);
    CHECK((COMMS_LINK_PAYLOAD_MAX - 1U) == g_m.msg.payload_len);
    CHECK('\0' == g_m.msg.payload[COMMS_LINK_PAYLOAD_MAX - 1U]);
    CHECK((COMMS_LINK_TOPIC_MAX - 1U) == strlen(g_m.msg.topic));
    CHECK(guard_intact());

    /* Far more fields than fit: stops safely at the buffer end. */
    fresh();
    codec_begin(&w, &g_m.msg, "t", 0U, false, 0U);

    for (i = 0U; i < 100U; i++)
    {
        codec_put_uint(&w, CODEC_KEY("k"), 0xFFFFFFFFU, 32U);
    }

    codec_end(&w);
    CHECK(COMMS_LINK_PAYLOAD_MAX >= g_m.msg.payload_len);
    CHECK(guard_intact());
}

#if COMMS_JSON
/* ------------------------------------------------------------------------ *
 * JSON build
 * ------------------------------------------------------------------------ */

static void
test_json (void)
{
    codec_writer_t w;

    printf("JSON text\n");

    fresh();
    codec_begin(&w, &g_m.msg, "robot/heartbeat", 0U, false, 1234U);
    codec_put_uint(&w, CODEC_KEY("seq"), 12U, 16U);
    codec_put_int(&w, CODEC_KEY("neg"), -40, 10U);
    codec_put_name(&w, CODEC_KEY("state"), CODEC_NAMES(g_names), 3U, 1U, 4U);
    codec_put_symbol(&w, CODEC_KEY("code"), "A", 1U, 3U);
    codec_end(&w);
    CHECK(0 == strcmp("{\"v\":1,\"t\":1234,\"seq\":12,\"neg\":-40,"
                      "\"state\":\"ONE\",\"code\":\"A\"}", g_m.msg.payload));
    CHECK(strlen(g_m.msg.payload) == g_m.msg.payload_len);
    CHECK(guard_intact());

    /* Clamped exactly like binary; unknown enum index is named. */
    fresh();
    codec_begin(&w, &g_m.msg, "x", 0U, false, 0U);
    codec_put_uint(&w, CODEC_KEY("u"), 5000U, 12U);
    codec_put_int(&w, CODEC_KEY("hi"), 600, 10U);
    codec_put_int(&w, CODEC_KEY("lo"), -600, 10U);
    codec_put_int(&w, CODEC_KEY("min"), INT32_MIN, 32U);
    codec_put_uint(&w, CODEC_KEY("max"), 0xFFFFFFFFU, 32U);
    codec_put_name(&w, CODEC_KEY("bad"), CODEC_NAMES(g_names), 3U, 9U, 4U);
    codec_end(&w);
    CHECK(0 == strcmp("{\"v\":1,\"t\":0,\"u\":4095,\"hi\":511,\"lo\":-512,"
                      "\"min\":-2147483648,\"max\":4294967295,"
                      "\"bad\":\"UNKNOWN\"}", g_m.msg.payload));
}

#else
/* ------------------------------------------------------------------------ *
 * Binary build
 * ------------------------------------------------------------------------ */

static uint8_t
byte_at (uint32_t i)
{
    return (uint8_t) g_m.msg.payload[i];
}


/* Independent reader: LSB-first bit stream, as the laptop decodes it. */
static uint32_t
read_bits (uint32_t * p_pos, uint32_t bits)
{
    uint32_t value = 0U;
    uint32_t i;

    for (i = 0U; i < bits; i++)
    {
        uint32_t const bit = (byte_at(*p_pos / 8U) >> (*p_pos % 8U)) & 1U;

        value |= bit << i;
        (*p_pos)++;
    }

    return value;
}


static int32_t
read_signed (uint32_t * p_pos, uint32_t bits)
{
    uint32_t const raw = read_bits(p_pos, bits);
    int64_t        v   = (int64_t) raw;

    if (0U != (raw & (1UL << (bits - 1U))))
    {
        v -= (int64_t) 1 << bits;          /* Sign-extend. */
    }

    return (int32_t) v;
}


static void
to_hex (char * p_out)
{
    uint32_t i;

    for (i = 0U; i < g_m.msg.payload_len; i++)
    {
        sprintf(&p_out[i * 2U], "%02x", byte_at(i));
    }

    p_out[g_m.msg.payload_len * 2U] = '\0';
}


static void
test_exact_bytes (void)
{
    codec_writer_t w;

    printf("binary exact bytes\n");

    /* Header only: version 2 in bits 0-1 (0x02), time 1 in bit 2 (0x04)
     * -> 0x06; 34 bits round up to 5 bytes. */
    fresh();
    codec_begin(&w, &g_m.msg, "t", 1U, true, 1U);
    codec_end(&w);
    CHECK(5U == g_m.msg.payload_len);
    CHECK(0x06U == byte_at(0));
    CHECK(0x00U == byte_at(1));
    CHECK(0x00U == byte_at(4));
    CHECK(1U == g_m.msg.qos);
    CHECK(1U == g_m.msg.b_retain);

    /* Time 0xFFFFFFFF fills bits 2-33: 0xFE, 0xFF x3, 0x03. */
    fresh();
    codec_begin(&w, &g_m.msg, "t", 0U, false, 0xFFFFFFFFU);
    CHECK(0xFEU == byte_at(0));
    CHECK(0xFFU == byte_at(3));
    CHECK(0x03U == byte_at(4));

    /* -1 in 4 bits after the header: bits 34-37 -> byte 4 = 0x3C. */
    fresh();
    codec_begin(&w, &g_m.msg, "t", 0U, false, 0U);
    codec_put_int(&w, CODEC_KEY("x"), -1, 4U);
    CHECK(5U == g_m.msg.payload_len);
    CHECK(0x02U == byte_at(0));
    CHECK(0x3CU == byte_at(4));

    /* 34 + 6 = 40 bits fill 5 bytes exactly; one more bit starts a clean
     * 6th byte.  Names and symbols are written as their code. */
    fresh();
    codec_begin(&w, &g_m.msg, "t", 0U, false, 0U);
    codec_put_name(&w, CODEC_KEY("n"), CODEC_NAMES(g_names), 3U, 0U, 3U);
    codec_put_symbol(&w, CODEC_KEY("s"), "A", 0U, 3U);
    CHECK(5U == g_m.msg.payload_len);
    codec_put_uint(&w, CODEC_KEY("b"), 1U, 1U);
    CHECK(6U == g_m.msg.payload_len);
    CHECK(0x01U == byte_at(5));
}


static void
test_round_trip (void)
{
    codec_writer_t w;
    uint32_t       pos;
    uint32_t       bits;

    printf("binary round trip, every width\n");

    for (bits = 2U; bits < 32U; bits++)
    {
        int32_t const max = (int32_t) ((1UL << (bits - 1U)) - 1U);
        int32_t const min = -max - 1;

        fresh();
        codec_begin(&w, &g_m.msg, "t", 0U, false, 0xDEADBEEFU);
        codec_put_uint(&w, CODEC_KEY("a"), 1U, 1U);     /* Misalign. */
        codec_put_int(&w, CODEC_KEY("b"), max, bits);
        codec_put_int(&w, CODEC_KEY("c"), min, bits);
        codec_put_int(&w, CODEC_KEY("d"), -1, bits);
        codec_put_uint(&w, CODEC_KEY("e"), 0xFFFFFFFFU, bits);  /* Clamped. */

        pos = 0U;
        CHECK(CODEC_VERSION == read_bits(&pos, CODEC_VERSION_BITS));
        CHECK(0xDEADBEEFU == read_bits(&pos, CODEC_TIME_BITS));
        CHECK(1U == read_bits(&pos, 1U));
        CHECK(max == read_signed(&pos, bits));
        CHECK(min == read_signed(&pos, bits));
        CHECK(-1 == read_signed(&pos, bits));
        CHECK(((1UL << bits) - 1U) == read_bits(&pos, bits));
        CHECK(((pos + 7U) / 8U) == g_m.msg.payload_len);
        CHECK(guard_intact());
    }

    /* 32-bit fields: full range. */
    fresh();
    codec_begin(&w, &g_m.msg, "t", 0U, false, 0U);
    codec_put_int(&w, CODEC_KEY("a"), INT32_MIN, 32U);
    codec_put_uint(&w, CODEC_KEY("b"), 0xFFFFFFFFU, 32U);
    pos = CODEC_HEADER_BITS;
    CHECK(0x80000000U == read_bits(&pos, 32U));
    CHECK(0xFFFFFFFFU == read_bits(&pos, 32U));

    /* Out of range clamps, never wraps; unknown name index clamps. */
    fresh();
    codec_begin(&w, &g_m.msg, "t", 0U, false, 0U);
    codec_put_int(&w, CODEC_KEY("a"), 600, 10U);
    codec_put_int(&w, CODEC_KEY("b"), -600, 10U);
    codec_put_name(&w, CODEC_KEY("c"), CODEC_NAMES(g_names), 3U, 9U, 3U);
    codec_put_uint(&w, CODEC_KEY("d"), 5000U, 12U);
    pos = CODEC_HEADER_BITS;
    CHECK(511 == read_signed(&pos, 10U));
    CHECK(-512 == read_signed(&pos, 10U));
    CHECK(7U == read_bits(&pos, 3U));
    CHECK(4095U == read_bits(&pos, 12U));
}


/* Golden messages: locked here, decoded by the laptop test. */
static void
test_golden (void)
{
    codec_writer_t w;
    char           hex[COMMS_LINK_PAYLOAD_MAX * 2U + 1U];

    printf("golden messages (for tools/test_robot_decode.ps1)\n");

    /* robot/telemetry/motion */
    fresh();
    codec_begin(&w, &g_m.msg, "robot/telemetry/motion", 0U, false, 207050U);
    codec_put_int(&w, CODEC_KEY("spd_l_mm_s"), 201, 10U);
    codec_put_int(&w, CODEC_KEY("spd_r_mm_s"), -199, 10U);
    codec_put_int(&w, CODEC_KEY("enc_l"), 40, 19U);
    codec_put_int(&w, CODEC_KEY("enc_r"), -40, 19U);
    codec_put_int(&w, CODEC_KEY("dist_mm"), 20, 16U);
    codec_put_int(&w, CODEC_KEY("tgt_l_mm_s"), 250, 10U);
    codec_put_int(&w, CODEC_KEY("tgt_r_mm_s"), -120, 10U);
    codec_put_int(&w, CODEC_KEY("slew_l_mm_s"), 200, 10U);
    codec_put_int(&w, CODEC_KEY("slew_r_mm_s"), -100, 10U);
    codec_put_int(&w, CODEC_KEY("pwm_l"), 190, 11U);
    codec_put_int(&w, CODEC_KEY("pwm_r"), -145, 11U);
    codec_end(&w);
    to_hex(hex);
    printf("  motion    %s\n", hex);
    CHECK(22U == g_m.msg.payload_len);
    /* Locked: a change here is a format change (docs/BINARY_FORMAT.md). */
    CHECK(0 == strcmp("2aa30c002493330a00b0ff4f01a00fe2c870ee8bb703", hex));

    /* robot/telemetry/line: position 1, error -5, centre on the line,
     * raw centre 3100 and right 1800. */
    fresh();
    codec_begin(&w, &g_m.msg, "robot/telemetry/line", 0U, false, 207050U);
    codec_put_name(&w, CODEC_KEY("pos"), CODEC_NAMES(g_names), 3U, 1U, 3U);
    codec_put_int(&w, CODEC_KEY("err"), -5, 8U);
    codec_put_uint(&w, CODEC_KEY("sensors"), 1U, 2U);
    codec_put_uint(&w, CODEC_KEY("centre_adc"), 3100U, 12U);
    codec_put_uint(&w, CODEC_KEY("right_adc"), 1800U, 12U);
    codec_end(&w);
    to_hex(hex);
    printf("  line      %s\n", hex);
    CHECK(9U == g_m.msg.payload_len);
    CHECK(0 == strcmp("2aa30c00643f0e4638", hex));

    /* robot/heartbeat: seq 199, READY, fault 0x204, ev_drop 300 -> 255 */
    fresh();
    codec_begin(&w, &g_m.msg, "robot/heartbeat", 0U, false, 5710U);
    codec_put_uint(&w, CODEC_KEY("seq"), 199U, 16U);
    codec_put_name(&w, CODEC_KEY("state"), CODEC_NAMES(g_names), 3U, 1U, 4U);
    codec_put_uint(&w, CODEC_KEY("fault"), 0x204U, 10U);
    codec_put_uint(&w, CODEC_KEY("ev_drop"), 300U, 8U);
    codec_end(&w);
    to_hex(hex);
    printf("  heartbeat %s\n", hex);
    CHECK(9U == g_m.msg.payload_len);
    CHECK(0 == strcmp("3a5900001c030481ff", hex));

    /* robot/event/calibration: index 4 of 5 (sensor gap), a 6 sample gap. */
    fresh();
    codec_begin(&w, &g_m.msg, "robot/event/calibration", 1U, false, 8000U);
    codec_put_uint(&w, CODEC_KEY("result"), 4U, 3U);
    codec_put_uint(&w, CODEC_KEY("centre_low"), 400U, 12U);
    codec_put_uint(&w, CODEC_KEY("centre_high"), 3100U, 12U);
    codec_put_uint(&w, CODEC_KEY("centre_thr"), 1750U, 12U);
    codec_put_uint(&w, CODEC_KEY("centre_full"), 2760U, 12U);
    codec_put_uint(&w, CODEC_KEY("right_low"), 500U, 12U);
    codec_put_uint(&w, CODEC_KEY("right_high"), 3000U, 12U);
    codec_put_uint(&w, CODEC_KEY("right_thr"), 1750U, 12U);
    codec_put_uint(&w, CODEC_KEY("barcode_low"), 300U, 12U);
    codec_put_uint(&w, CODEC_KEY("barcode_high"), 3200U, 12U);
    codec_put_uint(&w, CODEC_KEY("barcode_thr"), 1750U, 12U);
    codec_put_uint(&w, CODEC_KEY("barcode_learned"), 1U, 1U);
    codec_put_uint(&w, CODEC_KEY("first_right"), 1U, 1U);
    codec_put_int(&w, CODEC_KEY("overlap"), -6, 9U);
    codec_put_uint(&w, CODEC_KEY("b4_floor_mg"), 0U, 9U);
    codec_put_uint(&w, CODEC_KEY("b4_samples"), 0U, 15U);
    codec_put_uint(&w, CODEC_KEY("sweep_deg"), 30U, 8U);
    codec_end(&w);
    to_hex(hex);
    printf("  calibration %s\n", hex);
    CHECK(25U == g_m.msg.payload_len);
    CHECK(0 == strcmp("027d0000103238d8da90953e70d7da580290ad6dfd0000001e", hex));
}

#endif /* COMMS_JSON */

/* ------------------------------------------------------------------------ *
 * Packed records (comms_codec.c): the words are the same in both builds, so
 * this runs for the binary and the JSON one.  Every field is checked at
 * zero, at its maximum, at its minimum where it is signed, and with an
 * out-of-range value that must clamp to the wire range.
 * ------------------------------------------------------------------------ */

static void
test_pack_motion (void)
{
    comms_motion_t in;
    comms_motion_t out;
    uint32_t       words[CODEC_MOTION_WORDS];
    int32_t        run_start_mm;

    printf("packed motion record, %u words\n",
           (unsigned) CODEC_MOTION_WORDS);
    CHECK(24U == (CODEC_MOTION_WORDS * sizeof(uint32_t)));

    /* Zero. */
    memset(&in, 0, sizeof(in));
    codec_pack_motion(words, &in);
    memset(&out, 0xFF, sizeof(out));
    codec_unpack_motion(words, &out);
    CHECK(0 == out.speed_left_mm_s);
    CHECK(0 == out.speed_right_mm_s);
    CHECK(0 == out.encoder_left);
    CHECK(0 == out.encoder_right);
    CHECK(0 == out.distance_mm);
    CHECK(0 == out.target_left_mm_s);
    CHECK(0 == out.target_right_mm_s);
    CHECK(0 == out.slewed_left_mm_s);
    CHECK(0 == out.slewed_right_mm_s);
    CHECK(0 == out.drive_left_permille);
    CHECK(0 == out.drive_right_permille);

    /* Maximum of every field (10, 19, 16 and 11 bits). */
    memset(&in, 0, sizeof(in));
    in.speed_left_mm_s      = 511;
    in.speed_right_mm_s     = 511;
    in.encoder_left         = 262143;
    in.encoder_right        = 262143;
    in.distance_mm          = 32767;
    in.target_left_mm_s     = 511;
    in.target_right_mm_s    = 511;
    in.slewed_left_mm_s     = 511;
    in.slewed_right_mm_s    = 511;
    in.drive_left_permille  = 1023;
    in.drive_right_permille = 1023;
    codec_pack_motion(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_motion(words, &out);
    CHECK(511 == out.speed_left_mm_s);
    CHECK(511 == out.speed_right_mm_s);
    CHECK(262143 == out.encoder_left);
    CHECK(262143 == out.encoder_right);
    CHECK(32767 == out.distance_mm);
    CHECK(511 == out.target_left_mm_s);
    CHECK(511 == out.target_right_mm_s);
    CHECK(511 == out.slewed_left_mm_s);
    CHECK(511 == out.slewed_right_mm_s);
    CHECK(1023 == out.drive_left_permille);
    CHECK(1023 == out.drive_right_permille);

    /* Minimum of every field: the sign extends out of the field. */
    memset(&in, 0, sizeof(in));
    in.speed_left_mm_s      = -512;
    in.speed_right_mm_s     = -512;
    in.encoder_left         = -262144;
    in.encoder_right        = -262144;
    in.distance_mm          = -32768;
    in.target_left_mm_s     = -512;
    in.target_right_mm_s    = -512;
    in.slewed_left_mm_s     = -512;
    in.slewed_right_mm_s    = -512;
    in.drive_left_permille  = -1024;
    in.drive_right_permille = -1024;
    codec_pack_motion(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_motion(words, &out);
    CHECK(-512 == out.speed_left_mm_s);
    CHECK(-512 == out.speed_right_mm_s);
    CHECK(-262144 == out.encoder_left);
    CHECK(-262144 == out.encoder_right);
    CHECK(-32768 == out.distance_mm);
    CHECK(-512 == out.target_left_mm_s);
    CHECK(-512 == out.target_right_mm_s);
    CHECK(-512 == out.slewed_left_mm_s);
    CHECK(-512 == out.slewed_right_mm_s);
    CHECK(-1024 == out.drive_left_permille);
    CHECK(-1024 == out.drive_right_permille);

    /* Encoder counts and travelled distance are stored full width because
     * the consumer reads them back and subtracts (run summary, wheel
     * difference); only the wire field clamps. */
    memset(&in, 0, sizeof(in));
    in.speed_left_mm_s      = 5000;
    in.speed_right_mm_s     = -5000;
    in.encoder_left         = 999999999;
    in.encoder_right        = -999999999;
    in.distance_mm          = 40000;
    in.target_left_mm_s     = 5000;
    in.target_right_mm_s    = -5000;
    in.slewed_left_mm_s     = 5000;
    in.slewed_right_mm_s    = -5000;
    in.drive_left_permille  = 5000;
    in.drive_right_permille = -5000;
    codec_pack_motion(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_motion(words, &out);
    CHECK(511 == out.speed_left_mm_s);
    CHECK(-512 == out.speed_right_mm_s);
    CHECK(999999999 == out.encoder_left);
    CHECK(-999999999 == out.encoder_right);
    CHECK(40000 == out.distance_mm);
    CHECK(511 == out.target_left_mm_s);
    CHECK(-512 == out.target_right_mm_s);
    CHECK(511 == out.slewed_left_mm_s);
    CHECK(-512 == out.slewed_right_mm_s);
    CHECK(1023 == out.drive_left_permille);
    CHECK(-1024 == out.drive_right_permille);

    /* A run that starts at 32000 mm and ends at 33000 mm must report
     * 1000 mm, not a clamped 767. */
    memset(&in, 0, sizeof(in));
    in.distance_mm = 32000;
    codec_pack_motion(words, &in);
    codec_unpack_motion(words, &out);
    run_start_mm = out.distance_mm;
    CHECK(32000 == run_start_mm);
    memset(&in, 0, sizeof(in));
    in.distance_mm = 33000;
    codec_pack_motion(words, &in);
    codec_unpack_motion(words, &out);
    CHECK(33000 == out.distance_mm);
    CHECK(1000 == (out.distance_mm - run_start_mm));

    /* Counts either side of the old 24-bit field and the int32 extremes. */
    memset(&in, 0, sizeof(in));
    in.encoder_left  = 8388608 + 100;
    in.encoder_right = -(8388608 + 100);
    codec_pack_motion(words, &in);
    codec_unpack_motion(words, &out);
    CHECK((8388608 + 100) == out.encoder_left);
    CHECK(-(8388608 + 100) == out.encoder_right);
    in.encoder_left  = INT32_MAX;
    in.encoder_right = INT32_MIN;
    codec_pack_motion(words, &in);
    codec_unpack_motion(words, &out);
    CHECK(INT32_MAX == out.encoder_left);
    CHECK(INT32_MIN == out.encoder_right);
    in.distance_mm = INT32_MAX;
    codec_pack_motion(words, &in);
    codec_unpack_motion(words, &out);
    CHECK(INT32_MAX == out.distance_mm);
    in.distance_mm = INT32_MIN;
    codec_pack_motion(words, &in);
    codec_unpack_motion(words, &out);
    CHECK(INT32_MIN == out.distance_mm);
}


static void
test_pack_line (void)
{
    comms_line_t in;
    comms_line_t out;
    uint32_t     words[CODEC_LINE_WORDS];

    printf("packed line record, %u words\n", (unsigned) CODEC_LINE_WORDS);
    CHECK(8U == (CODEC_LINE_WORDS * sizeof(uint32_t)));

    /* Zero. */
    memset(&in, 0, sizeof(in));
    codec_pack_line(words, &in);
    memset(&out, 0xFF, sizeof(out));
    codec_unpack_line(words, &out);
    CHECK(COMMS_LINE_LEFT == out.position);
    CHECK(0 == out.error);
    CHECK(0 == out.sensors);
    CHECK(0 == out.centre_adc);
    CHECK(0 == out.right_adc);

    /* Maximum (3, 8 signed, 2 and 12 bits). */
    memset(&in, 0, sizeof(in));
    in.position   = (comms_line_pos_t) 7;
    in.error      = 127;
    in.sensors    = 3U;
    in.centre_adc = 4095U;
    in.right_adc  = 4095U;
    codec_pack_line(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_line(words, &out);
    CHECK(7 == out.position);
    CHECK(127 == out.error);
    CHECK(3 == out.sensors);
    CHECK(4095 == out.centre_adc);
    CHECK(4095 == out.right_adc);

    /* Minimum: the signed error reaches -128 in its 8-bit field. */
    memset(&in, 0, sizeof(in));
    in.error = -128;
    codec_pack_line(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_line(words, &out);
    CHECK(-128 == out.error);

    /* Out of range: clamped. */
    memset(&in, 0, sizeof(in));
    in.position   = (comms_line_pos_t) 9;
    in.error      = 200;
    in.sensors    = 5U;
    in.centre_adc = 5000U;
    in.right_adc  = 5000U;
    codec_pack_line(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_line(words, &out);
    CHECK(7 == out.position);
    CHECK(127 == out.error);
    CHECK(3 == out.sensors);
    CHECK(4095 == out.centre_adc);
    CHECK(4095 == out.right_adc);

    in.error = -200;
    codec_pack_line(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_line(words, &out);
    CHECK(-128 == out.error);
}


static void
test_pack_terrain (void)
{
    comms_terrain_t in;
    comms_terrain_t out;
    uint32_t        words[CODEC_TERRAIN_WORDS];

    printf("packed terrain record, %u words\n",
           (unsigned) CODEC_TERRAIN_WORDS);
    CHECK(8U == (CODEC_TERRAIN_WORDS * sizeof(uint32_t)));

    /* Zero. */
    memset(&in, 0, sizeof(in));
    codec_pack_terrain(words, &in);
    memset(&out, 0xFF, sizeof(out));
    codec_unpack_terrain(words, &out);
    CHECK(COMMS_MOTION_QUIET == out.motion);
    CHECK(0 == out.tilt_mg);
    CHECK(0 == out.hump_dmm);
    CHECK(0 == out.turn_dps);

    /* Maximum (3, 11 signed, 10 and 11 signed bits). */
    memset(&in, 0, sizeof(in));
    in.motion   = (comms_motion_class_t) 7;
    in.tilt_mg  = 1023;
    in.hump_dmm = 1023;
    in.turn_dps = 1023;
    codec_pack_terrain(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_terrain(words, &out);
    CHECK(7 == out.motion);
    CHECK(1023 == out.tilt_mg);
    CHECK(1023 == out.hump_dmm);
    CHECK(1023 == out.turn_dps);

    /* Minimum. */
    memset(&in, 0, sizeof(in));
    in.tilt_mg  = -1024;
    in.turn_dps = -1024;
    codec_pack_terrain(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_terrain(words, &out);
    CHECK(-1024 == out.tilt_mg);
    CHECK(-1024 == out.turn_dps);

    /* Out of range: clamped (the hump height is unsigned on the wire). */
    memset(&in, 0, sizeof(in));
    in.motion   = (comms_motion_class_t) 9;
    in.tilt_mg  = 5000;
    in.hump_dmm = -5;
    in.turn_dps = -5000;
    codec_pack_terrain(words, &in);
    memset(&out, 0, sizeof(out));
    codec_unpack_terrain(words, &out);
    CHECK(7 == out.motion);
    CHECK(1023 == out.tilt_mg);
    CHECK(0 == out.hump_dmm);
    CHECK(-1024 == out.turn_dps);
}


/* Pack, unpack and return whether the type was known. */
static bool
event_round_trip (codec_event_t * p_in, codec_event_t * p_out)
{
    uint32_t words[CODEC_EVENT_WORDS];
    bool     b_ok;

    codec_pack_event(words, p_in);
    memset(p_out, 0, sizeof(*p_out));
    b_ok = codec_unpack_event(words, p_out);

    return b_ok;
}


static void
test_pack_event (void)
{
    codec_event_t in;
    codec_event_t out;

    printf("packed event element, %u words\n",
           (unsigned) CODEC_EVENT_WORDS);
    CHECK(28U == (CODEC_EVENT_WORDS * sizeof(uint32_t)));
    CHECK(7U == CODEC_EVENT_COUNT);
    /* Any Code 39 character, not just the course set (18 Nov). */
    CHECK(6U == CODEC_BITS_BARCODE);

    /* A run marker carries only its type; the time is all 32 bits. */
    memset(&in, 0, sizeof(in));
    in.time_ms = 0xFFFFFFFFU;
    in.type    = (uint8_t) CODEC_EVENT_RUN_BEGIN;
    CHECK(event_round_trip(&in, &out));
    CHECK(CODEC_EVENT_RUN_BEGIN == out.type);
    CHECK(0xFFFFFFFFU == out.time_ms);

    /* Barcode: a character, a nav code, then the clamps. */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_BARCODE;
    in.code = 'Z';
    in.kind = (uint8_t) COMMS_NAV_UTURN;
    CHECK(event_round_trip(&in, &out));
    CHECK(CODEC_EVENT_BARCODE == out.type);
    CHECK('Z' == out.code);
    CHECK(COMMS_NAV_UTURN == out.kind);

    in.code = 0xFFU;
    in.kind = 9U;                       /* 3 bits: clamps to 7 */
    CHECK(event_round_trip(&in, &out));
    CHECK(0xFF == out.code);
    CHECK(7 == out.kind);

    /* Hump: zero, maximum, out of range and the negative "invalid" one. */
    memset(&in, 0, sizeof(in));
    in.type  = (uint8_t) CODEC_EVENT_HUMP;
    in.value = 800;
    CHECK(event_round_trip(&in, &out));
    CHECK(800 == out.value);

    in.value = 5000;
    CHECK(event_round_trip(&in, &out));
    CHECK(1023 == out.value);

    in.value = -5;
    CHECK(event_round_trip(&in, &out));
    CHECK(0 == out.value);

    /* Motion class: 5, then out of range. */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_MOTION;
    in.kind = (uint8_t) COMMS_MOTION_TURNING;
    CHECK(event_round_trip(&in, &out));
    CHECK(COMMS_MOTION_TURNING == out.kind);

    in.kind = 9U;
    CHECK(event_round_trip(&in, &out));
    CHECK(7 == out.kind);

    /* Obstacle: zero. */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_OBSTACLE;
    CHECK(event_round_trip(&in, &out));
    CHECK(CODEC_EVENT_OBSTACLE == out.type);
    CHECK(0 == out.obstacle.distance_mm);
    CHECK(0 == out.obstacle.angle_deg);
    CHECK(0 == out.obstacle.width_mm);
    CHECK(0 == out.obstacle.clear_left_mm);
    CHECK(0 == out.obstacle.clear_right_mm);
    CHECK(COMMS_AVOID_STOP == out.obstacle.action);

    /* Obstacle: every field at its maximum (12, 8, 10, 12, 12, 3 bits). */
    in.obstacle.distance_mm    = 4095;
    in.obstacle.angle_deg      = 255;
    in.obstacle.width_mm       = 1023;
    in.obstacle.clear_left_mm  = 4095;
    in.obstacle.clear_right_mm = 4095;
    in.obstacle.action         = COMMS_AVOID_REVERSE_RETRY;
    CHECK(event_round_trip(&in, &out));
    CHECK(4095 == out.obstacle.distance_mm);
    CHECK(255 == out.obstacle.angle_deg);
    CHECK(1023 == out.obstacle.width_mm);
    CHECK(4095 == out.obstacle.clear_left_mm);
    CHECK(4095 == out.obstacle.clear_right_mm);
    CHECK(COMMS_AVOID_REVERSE_RETRY == out.obstacle.action);

    /* Obstacle: out of range clamps, a negative "invalid" becomes 0. */
    in.obstacle.distance_mm    = -1;
    in.obstacle.angle_deg      = 5000;
    in.obstacle.width_mm       = 5000;
    in.obstacle.clear_left_mm  = -1;
    in.obstacle.clear_right_mm = 5000;
    in.obstacle.action         = (comms_avoid_t) 9;
    CHECK(event_round_trip(&in, &out));
    CHECK(0 == out.obstacle.distance_mm);
    CHECK(255 == out.obstacle.angle_deg);
    CHECK(1023 == out.obstacle.width_mm);
    CHECK(0 == out.obstacle.clear_left_mm);
    CHECK(4095 == out.obstacle.clear_right_mm);
    CHECK(7 == out.obstacle.action);

    /* Calibration: zero. */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_CALIBRATION;
    CHECK(event_round_trip(&in, &out));
    CHECK(CODEC_EVENT_CALIBRATION == out.type);
    CHECK(COMMS_CAL_OK == out.calibration.result);
    CHECK(0 == out.calibration.centre_low);
    CHECK(0 == out.calibration.centre_high);
    CHECK(0 == out.calibration.centre_threshold);
    CHECK(0 == out.calibration.centre_full_black);
    CHECK(0 == out.calibration.right_low);
    CHECK(0 == out.calibration.right_high);
    CHECK(0 == out.calibration.right_threshold);
    CHECK(0 == out.calibration.barcode_low);
    CHECK(0 == out.calibration.barcode_high);
    CHECK(0 == out.calibration.barcode_threshold);
    CHECK(!out.calibration.b_barcode_learned);
    CHECK(!out.calibration.b_first_right);
    CHECK(0 == out.calibration.overlap_samples);
    CHECK(0 == out.calibration.b4_floor_mg);
    CHECK(0 == out.calibration.b4_samples);
    CHECK(0 == out.calibration.sweep_deg);

    /* Calibration: every field at its maximum. */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_CALIBRATION;
    in.calibration.result             = COMMS_CAL_SENSOR_GAP;
    in.calibration.centre_low         = 4095U;
    in.calibration.centre_high        = 4095U;
    in.calibration.centre_threshold   = 4095U;
    in.calibration.centre_full_black  = 4095U;
    in.calibration.right_low          = 4095U;
    in.calibration.right_high         = 4095U;
    in.calibration.right_threshold    = 4095U;
    in.calibration.barcode_low        = 4095U;
    in.calibration.barcode_high       = 4095U;
    in.calibration.barcode_threshold  = 4095U;
    in.calibration.b_barcode_learned  = true;
    in.calibration.b_first_right      = true;
    in.calibration.overlap_samples    = 255;
    in.calibration.b4_floor_mg        = 511U;
    in.calibration.b4_samples         = 32767U;
    in.calibration.sweep_deg          = 255;
    CHECK(event_round_trip(&in, &out));
    CHECK(COMMS_CAL_SENSOR_GAP == out.calibration.result);
    CHECK(4095 == out.calibration.centre_low);
    CHECK(4095 == out.calibration.centre_high);
    CHECK(4095 == out.calibration.centre_threshold);
    CHECK(4095 == out.calibration.centre_full_black);
    CHECK(4095 == out.calibration.right_low);
    CHECK(4095 == out.calibration.right_high);
    CHECK(4095 == out.calibration.right_threshold);
    CHECK(4095 == out.calibration.barcode_low);
    CHECK(4095 == out.calibration.barcode_high);
    CHECK(4095 == out.calibration.barcode_threshold);
    CHECK(out.calibration.b_barcode_learned);
    CHECK(out.calibration.b_first_right);
    CHECK(255 == out.calibration.overlap_samples);
    CHECK(511 == out.calibration.b4_floor_mg);
    CHECK(32767 == out.calibration.b4_samples);
    CHECK(255 == out.calibration.sweep_deg);

    /* Calibration: the signed overlap reaches its minimum (-256, which is
     * also "not measured"). */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_CALIBRATION;
    in.calibration.overlap_samples = COMMS_CAL_OVERLAP_NOT_MEASURED;
    CHECK(event_round_trip(&in, &out));
    CHECK(COMMS_CAL_OVERLAP_NOT_MEASURED ==
          out.calibration.overlap_samples);

    /* Calibration: out of range clamps. */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_CALIBRATION;
    in.calibration.result            = (comms_cal_result_t) 9;
    in.calibration.centre_low        = 5000U;
    in.calibration.overlap_samples   = -400;
    in.calibration.b4_floor_mg       = 600U;
    in.calibration.b4_samples        = 40000U;
    in.calibration.sweep_deg         = -5;
    CHECK(event_round_trip(&in, &out));
    CHECK(7 == out.calibration.result);
    CHECK(4095 == out.calibration.centre_low);
    CHECK(COMMS_CAL_OVERLAP_NOT_MEASURED ==
          out.calibration.overlap_samples);
    CHECK(511 == out.calibration.b4_floor_mg);
    CHECK(32767 == out.calibration.b4_samples);
    CHECK(0 == out.calibration.sweep_deg);

    in.calibration.sweep_deg = 500;
    CHECK(event_round_trip(&in, &out));
    CHECK(255 == out.calibration.sweep_deg);

    /* Unknown event type: rejected, never guessed; the caller drops it. */
    memset(&in, 0, sizeof(in));
    in.type = (uint8_t) CODEC_EVENT_COUNT;
    CHECK(!event_round_trip(&in, &out));
}

/* ------------------------------------------------------------------------ */

int
main (void)
{
    printf("format: %s\n", COMMS_JSON ? "JSON" : "binary");
    test_text();
#if COMMS_JSON
    test_json();
#else
    test_exact_bytes();
    test_round_trip();
    test_golden();
#endif
    test_pack_motion();
    test_pack_line();
    test_pack_terrain();
    test_pack_event();

    printf("\n%u checks, %u failed\n", (unsigned) g_checks,
           (unsigned) g_failures);

    return (0U == g_failures) ? 0 : 1;
}

/*** end of file ***/
