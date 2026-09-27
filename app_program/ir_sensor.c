/*
 * ir_sensor.c
 *
 * Buddy 3 - IR sensor interface
 *
 * Reads the three IR sensors through the micro T-Kernel
 * ADC device driver instead of accessing the Pico SDK
 * ADC hardware directly.
 */

#include "ir_sensor.h"

#include <tk/tkernel.h>
#include <tm/tmonitor.h>


/*
 * IR sensor GPIO assignments.
 *
 * Left sensor   AO -> GP28 / ADC2
 * Centre sensor AO -> GP27 / ADC1
 * Right sensor  AO -> GP26 / ADC0
 */
#define IR_LEFT_PIN        28U
#define IR_CENTRE_PIN      27U
#define IR_RIGHT_PIN       26U

#define IR_LEFT_ADC        2U
#define IR_CENTRE_ADC      1U
#define IR_RIGHT_ADC       0U

/* ADC unit 0 is registered as "adca" by the T-Kernel device layer. */
#define IR_ADC_DEVNAME    ((CONST UB *)"adca")

/* Device descriptor from tk_opn_dev(). Negative = not open. */
LOCAL ID ir_adc_dd = -1;


void ir_sensor_init(void)
{
    ir_adc_dd = tk_opn_dev(IR_ADC_DEVNAME, TD_READ);

    if (ir_adc_dd < E_OK)
    {
        tm_printf(
            (UB *)"[ir_sensor] Failed to open ADC device, er=%d\n",
            ir_adc_dd
        );
    }
}


/* Synchronous read of one ADC channel. */
static uint16_t ir_read_channel(W channel)
{
    uint16_t value = 0U;
    SZ actual_size = 0;
    ER ercd;

    if (ir_adc_dd < E_OK)
    {
        return 0U;
    }

    ercd = tk_srea_dev(
        ir_adc_dd,
        channel,
        &value,
        sizeof(value),
        &actual_size
    );

    if ((ercd < E_OK) ||
        (actual_size != (SZ)sizeof(value)))
    {
        return 0U;
    }

    return value;
}


ir_readings_t ir_sensor_read(void)
{
    ir_readings_t readings;

    readings.left   = ir_read_channel(IR_LEFT_CH);
    readings.centre = ir_read_channel(IR_CENTRE_CH);
    readings.right  = ir_read_channel(IR_RIGHT_CH);

    return readings;
}


/* black-vs-white ADC direction still needs calibration. */
bool ir_sensor_is_black(uint16_t reading, uint16_t threshold)
{
    return (reading >= threshold);
}