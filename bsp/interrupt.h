#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "main.h"
#include "stdbool.h"

struct keys
{
    uint8_t judge_status; // 判断进行到哪一步
    bool key_status;      // 按键按下为0，松开为1
    bool sigle_flag;      // 短按下为1
    bool long_flag;       // 长按下为1 (700ms)
    bool double_flag;     // 双击标志
    uint16_t last_press_time; // 上一次按键的时间
    uint16_t hold_time;   // 按键按下的时间
};

// 枚举按键判断状态
// 0: 按键松开
// 1: 按键按下 消抖
// 2: 按键按下并保持一段时间
enum enum_judge_status
{
    KEY_UP = 0,
    KEY_DOWN = 1,
    KEY_HOLD = 2
};

enum enum_key_status
{
    DOWN = 0,
    UP = 1
};

extern struct keys key[4];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif // __INTERRUPT_H