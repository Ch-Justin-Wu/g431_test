#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "main.h"

struct adc
{
    float v1;
    float v2;
    float v_mcp;
};

extern struct adc adc;

float get_adc(ADC_HandleTypeDef *pin);


#endif // __BSP_ADC_H