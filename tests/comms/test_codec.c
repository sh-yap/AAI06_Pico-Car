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
    codec_end(&w);
    to_hex(hex);
    printf("  motion    %s\n", hex);
    CHECK(14U == g_m.msg.payload_len);
    /* Locked: a change here is a format change (docs/BINARY_FORMAT.md). */
    CHECK(0 == strcmp("2aa30c002493330a00b0ff4f0100", hex));

    /* robot/heartbeat: seq 199, READY, fault 0x104, ev_drop 300 -> 255 */
    fresh();
    codec_begin(&w, &g_m.msg, "robot/heartbeat", 0U, false, 5710U);
    codec_put_uint(&w, CODEC_KEY("seq"), 199U, 16U);
    codec_put_name(&w, CODEC_KEY("state"), CODEC_NAMES(g_names), 3U, 1U, 4U);
    codec_put_uint(&w, CODEC_KEY("fault"), 0x104U, 9U);
    codec_put_uint(&w, CODEC_KEY("ev_drop"), 300U, 8U);
    codec_end(&w);
    to_hex(hex);
    printf("  heartbeat %s\n", hex);
    CHECK(9U == g_m.msg.payload_len);
    CHECK(0 == strcmp("3a5900001c0304c17f", hex));
}

#endif /* COMMS_JSON */

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

    printf("\n%u checks, %u failed\n", (unsigned) g_checks,
           (unsigned) g_failures);

    return (0U == g_failures) ? 0 : 1;
}

/*** end of file ***/
