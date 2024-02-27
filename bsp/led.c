#include "led.h"

void led_disp(uint8_t ds_led)
{ // 所有LED熄灭
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
    //输出数据
    HAL_GPIO_WritePin(GPIOC, ds_led<<8, GPIO_PIN_RESET);
    //使能锁存器
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    //延时
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}