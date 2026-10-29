#ifndef BUDDY3_STATUS_H_
#define BUDDY3_STATUS_H_

#include <stdbool.h>
#include <stdint.h>

#include "line_follow.h"
#include "barcode.h"
#include "navigation.h"


/*
 * Buddy 3 packed status word (Tribunal 1 minimum-bit format).
 *
 *   bits  0-2   line state      3 bits  0 LEFT .. 4 JUNCTION, 5-7 reserved
 *   bits  3-10  line error      8 bits  signed, -127 left .. +127 right
 *   bits 11-12  line sensors    2 bits  LINE_SENSOR_* bits, 1 = black
 *   bits 13-15  barcode         3 bits  0 NONE .. 4 INVALID, 5-7 reserved
 *   bits 16-18  nav command     3 bits  0 NONE .. 4 UTURN, 5-7 reserved
 *   bits 19-31  reserved        always 0
 *
 * Written only by the Buddy 3 task. Only the stored/sent word is
 * packed; ADC readings, timestamps and encoder positions keep their
 * full width inside Buddy 3.
 */
typedef uint32_t buddy3_packed_t;

#define B3_LINE_STATE_SHIFT     0U
#define B3_LINE_STATE_MASK      0x7U
#define B3_LINE_ERROR_SHIFT     3U
#define B3_LINE_ERROR_MASK      0xFFU
#define B3_LINE_SENSORS_SHIFT   11U
#define B3_LINE_SENSORS_MASK    0x3U
#define B3_BARCODE_SHIFT        13U
#define B3_BARCODE_MASK         0x7U
#define B3_NAV_SHIFT            16U
#define B3_NAV_MASK             0x7U
#define B3_RESERVED_MASK        0xFFF80000UL


/* Unpacked view of the status word, used for logic and debugging. */
typedef struct
{
    line_state_t line_state;
    int8_t line_error;         /* -127 left .. +127 right */
    uint8_t line_sensors;      /* LINE_SENSOR_* bits, 1 = black */
    barcode_t barcode;         /* BARCODE_NONE unless one was just decoded */
    nav_command_t nav_command;
} buddy3_status_t;


/* Pack a status into the 19-bit status word. */
buddy3_packed_t buddy3_status_pack(buddy3_status_t const *p_status);

/*
 * Unpack a status word. Returns false (and leaves *p_status unchanged)
 * if any field holds a reserved code or a reserved bit is set.
 */
bool buddy3_status_unpack(buddy3_packed_t packed, buddy3_status_t *p_status);

#endif /* BUDDY3_STATUS_H_ */
