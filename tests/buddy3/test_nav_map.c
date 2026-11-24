/** @file test_nav_map.c
 *
 * @brief Host tests for app_program/buddy3_line_barcode/nav_map.c: the barcode character ->
 *        movement mapping the PC sends over MQTT before the demo.
 *
 * Run with:  make -C tests/buddy3 test
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nav_map.h"

static uint32_t failures = 0U;
static uint32_t checks = 0U;


static void
check(char const * p_name, bool ok)
{
    checks++;

    if (!ok)
    {
        failures++;
        (void)printf("FAIL %s\n", p_name);
    }
}


static bool
parse(char const * p_text, uint32_t * p_word)
{
    return nav_map_parse((uint8_t const *)p_text, (uint32_t)strlen(p_text),
                         p_word);
}


/* Parsed word has these characters for LEFT, RIGHT, FORWARD, UTURN. */
static bool
parses_to(char const * p_text, char const * p_chars)
{
    uint32_t word = 0U;

    return parse(p_text, &word)
           && (nav_map_char(word, NAV_LEFT) == p_chars[0])
           && (nav_map_char(word, NAV_RIGHT) == p_chars[1])
           && (nav_map_char(word, NAV_STRAIGHT) == p_chars[2])
           && (nav_map_char(word, NAV_UTURN) == p_chars[3]);
}


static bool
rejected(char const * p_text)
{
    uint32_t word = 0x12345678U;

    return !parse(p_text, &word) && (word == 0x12345678U);
}


static void
test_is_command(void)
{
    check("'map A=LEFT' is a map command",
          nav_map_is_command((uint8_t const *)"map A=LEFT", 10U));
    check("'MAP' alone is a map command",
          nav_map_is_command((uint8_t const *)"MAP", 3U));
    check("'mapping' is not", !nav_map_is_command((uint8_t const *)"mapping", 7U));
    check("'start' is not", !nav_map_is_command((uint8_t const *)"start", 5U));
    check("'ma' is not", !nav_map_is_command((uint8_t const *)"ma", 2U));
}


static void
test_accepted_forms(void)
{
    check("canonical form",
          parses_to("map A=LEFT F=RIGHT Z=FORWARD X=UTURN", "AFZX"));
    check("any order",
          parses_to("map X=UTURN Z=FORWARD A=LEFT F=RIGHT", "AFZX"));
    check("MOVE=CHAR",
          parses_to("map LEFT=A RIGHT=F FORWARD=Z UTURN=X", "AFZX"));
    check("lower case, STRAIGHT, U-TURN",
          parses_to("map a=left f=right z=straight x=u-turn", "AFZX"));
    check("U_TURN",
          parses_to("map A=LEFT F=RIGHT Z=FORWARD X=U_TURN", "AFZX"));
    check("commas, colons, spaces around the delimiter",
          parses_to("map A : LEFT, F:RIGHT ; Z= FORWARD,X =UTURN", "AFZX"));
    check("JSON object after map",
          parses_to("map {\"A\":\"LEFT\",\"F\":\"RIGHT\",\"Z\":\"FORWARD\",\"X\":\"UTURN\"}",
                    "AFZX"));
    check("without the 'map' keyword",
          parses_to("A=LEFT F=RIGHT Z=FORWARD X=UTURN", "AFZX"));
    check("digits and symbols",
          parses_to("map 1=LEFT -=RIGHT $=FORWARD %=UTURN", "1-$%"));
    check("trailing newline",
          parses_to("map A=LEFT F=RIGHT Z=FORWARD X=UTURN\r\n", "AFZX"));
}


static void
test_rejected_forms(void)
{
    check("empty", rejected("map"));
    check("three pairs", rejected("map A=LEFT F=RIGHT Z=FORWARD"));
    check("five pairs", rejected("map A=LEFT F=RIGHT Z=FORWARD X=UTURN Y=LEFT"));
    check("movement twice", rejected("map A=LEFT F=LEFT Z=FORWARD X=UTURN"));
    check("character twice", rejected("map A=LEFT A=RIGHT Z=FORWARD X=UTURN"));
    check("unknown movement", rejected("map A=LEFT F=RIGHT Z=BACK X=UTURN"));
    check("two-letter character", rejected("map AB=LEFT F=RIGHT Z=FORWARD X=UTURN"));
    check("not Code 39", rejected("map #=LEFT F=RIGHT Z=FORWARD X=UTURN"));
    check("missing delimiter", rejected("map A LEFT F=RIGHT Z=FORWARD X=UTURN"));
    check("missing value", rejected("map A= F=RIGHT Z=FORWARD X=UTURN"));
    check("char=char", rejected("map A=B F=RIGHT Z=FORWARD X=UTURN"));
    check("over-long word", rejected("map A=LEFTLEFTLEFT F=RIGHT Z=FORWARD X=UTURN"));
}


static void
test_lookup_and_format(void)
{
    uint32_t word = 0U;
    char text[NAV_MAP_TEXT_MAX];

    nav_map_set(NAV_MAP_NONE);
    check("nothing mapped at first", nav_map_lookup('A') == NAV_NONE);
    (void)nav_map_format(nav_map_get(), text, (uint32_t)sizeof(text));
    check("no mapping formats as 'none'", strcmp(text, "none") == 0);

    (void)parse("map Q=LEFT 7=RIGHT K=FORWARD /=UTURN", &word);
    nav_map_set(word);
    check("Q -> LEFT", nav_map_lookup('Q') == NAV_LEFT);
    check("7 -> RIGHT", nav_map_lookup('7') == NAV_RIGHT);
    check("K -> FORWARD", nav_map_lookup('K') == NAV_STRAIGHT);
    check("/ -> UTURN", nav_map_lookup('/') == NAV_UTURN);
    check("unmapped A -> NONE", nav_map_lookup('A') == NAV_NONE);
    check("'\\0' -> NONE", nav_map_lookup('\0') == NAV_NONE);

    (void)nav_map_format(nav_map_get(), text, (uint32_t)sizeof(text));
    check("format", strcmp(text, "Q=LEFT 7=RIGHT K=FORWARD /=UTURN") == 0);

    (void)nav_map_format(nav_map_get(), text, 5U);
    check("format truncates safely", strcmp(text, "Q=LE") == 0);

    /* The formatted text parses back to the same mapping. */
    {
        uint32_t again = 0U;
        (void)nav_map_format(word, text, (uint32_t)sizeof(text));
        check("format round-trips", parse(text, &again) && (again == word));
    }

    nav_map_set(NAV_MAP_NONE);
}


int
main(void)
{
    test_is_command();
    test_accepted_forms();
    test_rejected_forms();
    test_lookup_and_format();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return (failures == 0U) ? 0 : 1;
}
