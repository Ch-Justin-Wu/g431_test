#include "led.h"

void led_disp(uint8_t ds_led)
{ // 所有LED熄灭
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
    //
    HAL_GPIO_WritePin(GPIOC, ds_led<<8, GPIO_PIN_RESET);
    //打开锁存器
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}