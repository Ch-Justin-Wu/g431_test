#include "bsp_adc.h"

struct adc adc;

float get_adc(ADC_HandleTypeDef *hadc)
{
    uint32_t adc;
    HAL_ADC_Start(hadc);
    adc=HAL_ADC_GetValue(hadc);
    return adc*3.3f/4096.0f;
}

float my_get_adc(ADC_HandleTypeDef *hadc)
{
    uint32_t adc = 0;
    HAL_ADC_Start(hadc);
    adc = HAL_ADC_GetValue(hadc);
    return adc / 4096.f * 3.3f;
}
