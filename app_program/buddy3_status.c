/*
 * buddy3_status.c
 *
 * Buddy 3 - packed status word
 *
 * Packs line state, line error, line sensors, barcode and
 * navigation command into one 32-bit word using masks and
 * shifts, and unpacks it with reserved-code validation.
 * Plain C with no kernel calls, so it can be host-tested.
 */

#include "buddy3_status.h"


buddy3_packed_t buddy3_status_pack(buddy3_status_t const *p_status)
{
    buddy3_packed_t packed = 0U;

    packed |= ((uint32_t)p_status->line_state & B3_LINE_STATE_MASK)
              << B3_LINE_STATE_SHIFT;

    /* Two's complement byte: -127..+127 -> 0x81..0x7F. */
    packed |= ((uint32_t)(uint8_t)p_status->line_error & B3_LINE_ERROR_MASK)
              << B3_LINE_ERROR_SHIFT;

    packed |= ((uint32_t)p_status->line_sensors & B3_LINE_SENSORS_MASK)
              << B3_LINE_SENSORS_SHIFT;

    packed |= ((uint32_t)p_status->barcode & B3_BARCODE_MASK)
              << B3_BARCODE_SHIFT;

    packed |= ((uint32_t)p_status->nav_command & B3_NAV_MASK)
              << B3_NAV_SHIFT;

    packed |= ((uint32_t)p_status->cal_state & B3_CAL_MASK)
              << B3_CAL_SHIFT;

    return packed;
}


bool buddy3_status_unpack(buddy3_packed_t packed, buddy3_status_t *p_status)
{
    uint32_t line_state;
    uint32_t error_byte;
    uint32_t barcode;
    uint32_t nav;
    uint32_t cal;
    int32_t line_error;

    if ((packed & B3_RESERVED_MASK) != 0U)
    {
        return false;
    }

    line_state = (packed >> B3_LINE_STATE_SHIFT) & B3_LINE_STATE_MASK;
    error_byte = (packed >> B3_LINE_ERROR_SHIFT) & B3_LINE_ERROR_MASK;
    barcode    = (packed >> B3_BARCODE_SHIFT) & B3_BARCODE_MASK;
    nav        = (packed >> B3_NAV_SHIFT) & B3_NAV_MASK;
    cal        = (packed >> B3_CAL_SHIFT) & B3_CAL_MASK;

    /* Reserved codes: anything past the last defined enum value. */
    if ((line_state > (uint32_t)LINE_JUNCTION) ||
        (barcode > (uint32_t)BARCODE_INVALID) ||
        (nav > (uint32_t)NAV_UTURN) ||
        (cal > (uint32_t)LINE_CAL_FAIL_GAP))
    {
        return false;
    }

    /* Sign-extend the 8-bit error without relying on a narrowing cast. */
    line_error = (int32_t)error_byte;
    if (line_error > 127)
    {
        line_error -= 256;
    }

    /* -128 is outside the agreed -127..+127 range. */
    if (line_error < -LINE_ERROR_MAX)
    {
        return false;
    }

    p_status->line_state   = (line_state_t)line_state;
    p_status->line_error   = (int8_t)line_error;
    p_status->line_sensors =
        (uint8_t)((packed >> B3_LINE_SENSORS_SHIFT) & B3_LINE_SENSORS_MASK);
    p_status->barcode      = (barcode_t)barcode;
    p_status->nav_command  = (nav_command_t)nav;
    p_status->cal_state    = (line_cal_state_t)cal;

    return true;
}
