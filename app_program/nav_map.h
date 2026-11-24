#ifndef NAV_MAP_H_
#define NAV_MAP_H_

/*
 * nav_map.h
 *
 * Barcode character -> movement mapping, received over MQTT.
 *
 * The instructor names the four characters for LEFT, RIGHT, FORWARD
 * (NAV_STRAIGHT) and U-TURN just before the demo, so nothing here is
 * fixed at build time.  The PC sends one command on robot/command:
 *
 *     map A=LEFT F=RIGHT Z=FORWARD X=UTURN
 *
 * Accepted forms (case-insensitive, any order):
 *   - pairs CHAR=MOVE or MOVE=CHAR, '=' or ':' between them;
 *   - pairs separated by spaces, commas or semicolons;
 *   - quotes and braces are ignored, so {"A":"LEFT","F":"RIGHT",...}
 *     after "map " works too;
 *   - movement names LEFT, RIGHT, FORWARD or STRAIGHT, UTURN or U-TURN
 *     or U_TURN;
 *   - CHAR is one Code 39 data character: 0-9, A-Z (lower case is
 *     taken as upper case), - . $ / + %.
 * Exactly four pairs, each movement once, four different characters.
 * Anything else is rejected and the previous mapping stays.
 *
 * Until a mapping arrives every character is unmapped: barcodes are
 * still decoded and reported, but no turn is made.  The mapping lives in
 * RAM; send the command retained so a car that reboots gets it again.
 *
 * Storage is one 32-bit word, byte n = character for movement
 * NAV_LEFT + n (0 = no mapping).  An aligned word is read and written in
 * one access, so the comms task can replace it while Buddy 3 reads it
 * without a lock.  Plain C with no kernel calls, so it can be host-tested.
 */

#include <stdbool.h>
#include <stdint.h>

#include "navigation.h"

/* Movements in the mapping: NAV_LEFT .. NAV_UTURN. */
#define NAV_MAP_MOVES        4U

/* No mapping received. */
#define NAV_MAP_NONE         (UINT32_C(0))

/* Longest text from nav_map_format(), with its terminator:
 * "A=LEFT F=RIGHT Z=FORWARD X=UTURN". */
#define NAV_MAP_TEXT_MAX     40U

/* True if the payload is a map command: "map" (any case), then a space
 * or the end.  Trailing whitespace is ignored by the caller's parse. */
bool nav_map_is_command(uint8_t const * p_data, uint32_t len);

/*
 * Parse a map command payload (with or without the leading "map") into
 * a mapping word.  False, with *p_word untouched, if it is not exactly
 * four valid pairs.
 */
bool nav_map_parse(uint8_t const * p_data, uint32_t len, uint32_t * p_word);

/* Replace the active mapping (NAV_MAP_NONE clears it). */
void nav_map_set(uint32_t word);

/* The active mapping word. */
uint32_t nav_map_get(void);

/* Movement for a decoded character; NAV_NONE if it is not mapped. */
nav_command_t nav_map_lookup(char character);

/* Character mapped to a movement in word, or '\0'. */
char nav_map_char(uint32_t word, nav_command_t move);

/*
 * Text form of a mapping word, e.g. "A=LEFT F=RIGHT Z=FORWARD X=UTURN",
 * or "none".  Always terminated when size > 0; returns the length.
 */
uint32_t nav_map_format(uint32_t word, char * p_buf, uint32_t size);

#endif /* NAV_MAP_H_ */
