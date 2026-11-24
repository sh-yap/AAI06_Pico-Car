/*
 * nav_map.c
 *
 * Barcode character -> movement mapping received over MQTT (see
 * nav_map.h for the command format).
 */

#include "nav_map.h"

#include <stddef.h>


/* Longest movement name accepted ("STRAIGHT"). */
#define NAV_MAP_WORD_MAX     8U

#define BYTE_BITS            8U
#define BYTE_MASK            UINT32_C(0xFF)


/* Active mapping: byte n = character for NAV_LEFT + n, 0 = none. */
static volatile uint32_t g_map_word = NAV_MAP_NONE;

/* Text names in nav_map_format(), NAV_LEFT .. NAV_UTURN. */
static char const * const g_move_names[NAV_MAP_MOVES] =
{
    "LEFT", "RIGHT", "FORWARD", "UTURN"
};

/* Names accepted in a command, with their movement. */
typedef struct
{
    char const *  p_name;
    nav_command_t move;
} move_name_t;

static move_name_t const g_accepted_names[] =
{
    { "LEFT",     NAV_LEFT     },
    { "RIGHT",    NAV_RIGHT    },
    { "FORWARD",  NAV_STRAIGHT },
    { "STRAIGHT", NAV_STRAIGHT },
    { "UTURN",    NAV_UTURN    },
    { "U-TURN",   NAV_UTURN    },
    { "U_TURN",   NAV_UTURN    },
};


static char
to_upper (char c)
{
    return ((c >= 'a') && (c <= 'z')) ? (char) (c - ('a' - 'A')) : c;
}


static bool
is_space (uint8_t c)
{
    return (' ' == c) || ('\t' == c) || ('\r' == c) || ('\n' == c);
}


/* Between pairs: whitespace, list punctuation, JSON braces and quotes. */
static bool
is_separator (uint8_t c)
{
    return is_space(c) || (',' == c) || (';' == c) || ('{' == c)
           || ('}' == c) || ('"' == c) || ('\'' == c);
}


static bool
is_pair_delim (uint8_t c)
{
    return ('=' == c) || (':' == c);
}


/* A Code 39 data character the barcode can carry (space excluded: it
 * cannot be written in a pair). */
static bool
is_code39_char (char c)
{
    return ((c >= '0') && (c <= '9')) || ((c >= 'A') && (c <= 'Z'))
           || ('-' == c) || ('.' == c) || ('$' == c) || ('/' == c)
           || ('+' == c) || ('%' == c);
}


static bool
word_is (char const * p_word, uint32_t len, char const * p_name)
{
    uint32_t i = 0U;

    while ((i < len) && ('\0' != p_name[i]) && (p_word[i] == p_name[i]))
    {
        i++;
    }

    return (i == len) && ('\0' == p_name[i]);
}


/* Movement named by an upper-case word, or NAV_NONE. */
static nav_command_t
move_of (char const * p_word, uint32_t len)
{
    nav_command_t move = NAV_NONE;
    uint32_t      i;

    for (i = 0U; i < (sizeof g_accepted_names / sizeof g_accepted_names[0]);
         i++)
    {
        if (word_is(p_word, len, g_accepted_names[i].p_name))
        {
            move = g_accepted_names[i].move;
            break;
        }
    }

    return move;
}


/*
 * Read one word at *p_pos into p_word (upper case) up to a separator, a
 * pair delimiter or the end.  Returns its length, 0 if empty or longer
 * than NAV_MAP_WORD_MAX.
 */
static uint32_t
read_word (uint8_t const * p_data, uint32_t len, uint32_t * p_pos,
           char p_word[NAV_MAP_WORD_MAX])
{
    uint32_t n = 0U;

    while ((*p_pos < len) && !is_separator(p_data[*p_pos])
           && !is_pair_delim(p_data[*p_pos]))
    {
        if (n >= NAV_MAP_WORD_MAX)
        {
            return 0U;
        }

        p_word[n] = to_upper((char) p_data[*p_pos]);
        n++;
        (*p_pos)++;
    }

    return n;
}


/* Skip whitespace and quotes (not commas: those end a pair). */
static void
skip_blank (uint8_t const * p_data, uint32_t len, uint32_t * p_pos)
{
    while ((*p_pos < len)
           && (is_space(p_data[*p_pos]) || ('"' == p_data[*p_pos])
               || ('\'' == p_data[*p_pos])))
    {
        (*p_pos)++;
    }
}


bool
nav_map_is_command (uint8_t const * p_data, uint32_t len)
{
    return (NULL != p_data) && (len >= 3U)
           && ('M' == to_upper((char) p_data[0]))
           && ('A' == to_upper((char) p_data[1]))
           && ('P' == to_upper((char) p_data[2]))
           && ((3U == len) || is_space(p_data[3]));
}


bool
nav_map_parse (uint8_t const * p_data, uint32_t len, uint32_t * p_word)
{
    char          first[NAV_MAP_WORD_MAX];
    char          second[NAV_MAP_WORD_MAX];
    uint32_t      first_len;
    uint32_t      second_len;
    uint32_t      pos   = 0U;
    uint32_t      pairs = 0U;
    uint32_t      word  = NAV_MAP_NONE;
    uint32_t      slot;
    uint32_t      n;
    nav_command_t move;
    char          character;

    if ((NULL == p_data) || (NULL == p_word))
    {
        return false;
    }

    if (nav_map_is_command(p_data, len))
    {
        pos = 3U;
    }

    for (;;)
    {
        while ((pos < len) && is_separator(p_data[pos]))
        {
            pos++;
        }

        if (pos >= len)
        {
            break;
        }

        /* CHAR=MOVE or MOVE=CHAR. */
        first_len = read_word(p_data, len, &pos, first);
        skip_blank(p_data, len, &pos);

        if ((0U == first_len) || (pos >= len) || !is_pair_delim(p_data[pos]))
        {
            return false;
        }

        pos++;
        skip_blank(p_data, len, &pos);
        second_len = read_word(p_data, len, &pos, second);

        move = move_of(second, second_len);

        if ((1U == first_len) && (NAV_NONE != move))
        {
            character = first[0];
        }
        else
        {
            move = move_of(first, first_len);

            if ((1U != second_len) || (NAV_NONE == move))
            {
                return false;
            }

            character = second[0];
        }

        if (!is_code39_char(character) || (pairs >= NAV_MAP_MOVES))
        {
            return false;
        }

        slot = (uint32_t) move - (uint32_t) NAV_LEFT;

        /* Each movement once, each character once. */
        if (0U != ((word >> (slot * BYTE_BITS)) & BYTE_MASK))
        {
            return false;
        }

        for (n = 0U; n < NAV_MAP_MOVES; n++)
        {
            if ((uint32_t) (uint8_t) character
                == ((word >> (n * BYTE_BITS)) & BYTE_MASK))
            {
                return false;
            }
        }

        word |= ((uint32_t) (uint8_t) character) << (slot * BYTE_BITS);
        pairs++;
    }

    if (NAV_MAP_MOVES != pairs)
    {
        return false;
    }

    *p_word = word;
    return true;
}


void
nav_map_set (uint32_t word)
{
    g_map_word = word;
}


uint32_t
nav_map_get (void)
{
    return g_map_word;
}


char
nav_map_char (uint32_t word, nav_command_t move)
{
    char character = '\0';

    if ((move >= NAV_LEFT) && (move <= NAV_UTURN))
    {
        character = (char) ((word
                             >> (((uint32_t) move - (uint32_t) NAV_LEFT)
                                 * BYTE_BITS))
                            & BYTE_MASK);
    }

    return character;
}


nav_command_t
nav_map_lookup (char character)
{
    uint32_t const word = g_map_word;       /* One read: one mapping. */
    nav_command_t  move = NAV_NONE;
    uint32_t       n;

    if ('\0' != character)
    {
        for (n = 0U; n < NAV_MAP_MOVES; n++)
        {
            if ((uint32_t) (uint8_t) character
                == ((word >> (n * BYTE_BITS)) & BYTE_MASK))
            {
                move = (nav_command_t) ((uint32_t) NAV_LEFT + n);
                break;
            }
        }
    }

    return move;
}


/* Append p_text at *p_len, never past size - 1. */
static void
append (char * p_buf, uint32_t size, uint32_t * p_len, char const * p_text)
{
    while (('\0' != *p_text) && ((*p_len + 1U) < size))
    {
        p_buf[*p_len] = *p_text;
        (*p_len)++;
        p_text++;
    }

    p_buf[*p_len] = '\0';
}


uint32_t
nav_map_format (uint32_t word, char * p_buf, uint32_t size)
{
    uint32_t len = 0U;
    uint32_t n;
    char     pair[3];

    if ((NULL == p_buf) || (0U == size))
    {
        return 0U;
    }

    p_buf[0] = '\0';

    if (NAV_MAP_NONE == word)
    {
        append(p_buf, size, &len, "none");
        return len;
    }

    for (n = 0U; n < NAV_MAP_MOVES; n++)
    {
        pair[0] = (char) ((word >> (n * BYTE_BITS)) & BYTE_MASK);
        pair[1] = '=';
        pair[2] = '\0';

        if (0U != n)
        {
            append(p_buf, size, &len, " ");
        }

        append(p_buf, size, &len, pair);
        append(p_buf, size, &len, g_move_names[n]);
    }

    return len;
}
