#include "bsp_adc.h"

float get_adc(ADC_HandleTypeDef *hadc)
{
    uint32_t adc;
    HAL_ADC_Start(hadc);
    adc=HAL_ADC_GetValue(hadc);
    return adc*3.3f/4027.0f;
}