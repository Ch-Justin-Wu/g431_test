#include "interrupt.h"

struct keys key[4] = {0, 0, 0, 0};
struct pwm_capture pwm_capture[2] = {0, 0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM6)
    {
        // ��ȡ�ĸ�������״̬
        key[0].key_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        key[1].key_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
        key[2].key_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
        key[3].key_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

        // ��ÿ������������������
        for (int i = 0; i < 4; i++)
        {
            switch (key[i].judge_status)
            {
            case KEY_UP: // ����δ������
            {
                if (key[i].key_status == DOWN) // �������������
                {
                    key[i].judge_status = KEY_DOWN; // ����״̬Ϊ�����ѱ�����
                }
            }
            break;
            case KEY_DOWN: // �����ѱ�����
            {
                if (key[i].key_status == DOWN) // ���������Ȼ������
                {

                    key[i].judge_status = KEY_HOLD; // ����״̬Ϊ��������������
                }
                else // ��������Ѿ��ͷ�
                {
                    key[i].judge_status = KEY_UP; // ����״̬Ϊ����δ������
                }
            }
            break;
            case KEY_HOLD: // ��������������
            {
                if (key[i].key_status == UP) // ��������Ѿ��ͷ�
                {
                    key[i].judge_status = KEY_UP; // ����״̬Ϊ����δ������
                    if (key[i].hold_time < 70)    // ����������µ�ʱ��С��700ms
                    {
                        key[i].sigle_flag = 1; // ���ö̰���־
                    }
                    key[i].hold_time = 0; // ���ð������µ�ʱ��
                }
                else // ���������Ȼ������
                {
                    key[i].hold_time++;         // �������µ�ʱ���һ
                    if (key[i].hold_time >= 70) // ����������µ�ʱ�䳬��700ms
                    {
                        key[i].long_flag = 1; // ���ó�����־
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