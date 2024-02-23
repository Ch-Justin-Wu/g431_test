#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "main.h"
#include "stdbool.h"

struct keys
{
    uint8_t judge_status; // �жϽ��е���һ��
    bool key_status;      // ��������Ϊ0���ɿ�Ϊ1
    bool sigle_flag;      // �̰���Ϊ1
    bool long_flag;       // ������Ϊ1 (700ms)
    bool double_flag;     // ˫����־
    uint16_t last_press_time; // ��һ�ΰ�����ʱ��
    uint16_t hold_time;   // �������µ�ʱ��
};

// ö�ٰ����ж�״̬
// 0: �����ɿ�
// 1: �������� ����
// 2: �������²�����һ��ʱ��
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