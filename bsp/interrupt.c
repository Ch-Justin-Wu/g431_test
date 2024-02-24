#include "interrupt.h"

struct keys key[4] = {0, 0, 0, 0};
struct pwm_capture pwm_capture[2] = {0, 0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM6)
    {
        // 读取四个按键的状态
        key[0].key_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        key[1].key_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
        key[2].key_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
        key[3].key_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

        // 对每个按键进行消抖操作
        for (int i = 0; i < 4; i++)
        {
            switch (key[i].judge_status)
            {
            case KEY_UP: // 按键未被按下
            {
                if (key[i].key_status == DOWN) // 如果按键被按下
                {
                    key[i].judge_status = KEY_DOWN; // 更改状态为按键已被按下
                }
            }
            break;
            case KEY_DOWN: // 按键已被按下
            {
                if (key[i].key_status == DOWN) // 如果按键仍然被按下
                {

                    key[i].judge_status = KEY_HOLD; // 更改状态为按键被持续按下
                }
                else // 如果按键已经释放
                {
                    key[i].judge_status = KEY_UP; // 更改状态为按键未被按下
                }
            }
            break;
            case KEY_HOLD: // 按键被持续按下
            {
                if (key[i].key_status == UP) // 如果按键已经释放
                {
                    key[i].judge_status = KEY_UP; // 更改状态为按键未被按下
                    if (key[i].hold_time < 70)    // 如果按键按下的时间小于700ms
                    {
                        key[i].sigle_flag = 1; // 设置短按标志
                    }
                    key[i].hold_time = 0; // 重置按键按下的时间
                }
                else // 如果按键仍然被按下
                {
                    key[i].hold_time++;         // 按键按下的时间加一
                    if (key[i].hold_time >= 70) // 如果按键按下的时间超过700ms
                    {
                        key[i].long_flag = 1; // 设置长按标志
                    }
                }
            }
            break;

            default:
                break;
            }
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        pwm_capture[0].ccr1_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        __HAL_TIM_SetCounter(htim, 0);
        pwm_capture[0].frq=80000000/80/pwm_capture[0].ccr1_val;
        HAL_TIM_IC_Start(htim, TIM_CHANNEL_1);
    }
    if (htim->Instance == TIM3)
    {
        pwm_capture[1].ccr1_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        __HAL_TIM_SetCounter(htim, 0);
        pwm_capture[1].frq=80000000/80/pwm_capture[1].ccr1_val;
        HAL_TIM_IC_Start(htim, TIM_CHANNEL_1);
    }
}