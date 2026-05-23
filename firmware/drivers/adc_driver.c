/* ADC Driver — Wheel Speed Hall Sensor (ADC1 CH0, PA0)
 * Resolution: 12-bit (0–4095)
 * Vref: 3.3V
 * Conversion time: ~1µs at 84MHz APB2 with /4 prescaler */
#include "telemetry.h"

uint16_t ADC_ReadChannel(uint8_t channel) {
    /* HAL_ADC_Start(&hadc1);
     * HAL_ADC_PollForConversion(&hadc1, 10);
     * return HAL_ADC_GetValue(&hadc1); */
    (void)channel;
    return 2048; /* stub — mid-scale = ~50% speed */
}
