/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "lcd.h"
#include "stdio.h"
#include "interrupt.h"
#include "bsp_adc.h"
#include "i2c_hal.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RTC_TimeTypeDef s_time;
RTC_DateTypeDef s_date;

uint32_t i_text = 5;
char view_flag = 0;

// pwmռ�ձ�
uint8_t pa6_duty = 10;
uint8_t pa7_duty = 10;

struct car car[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void key_process();
void disp_process();
void data_tx();
void data_rx();
void rtc_process();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  led_disp(0x00); // led��ʼ��
  LCD_Init();     // lcd��Ļ��ʼ��

  // ���LCD��Ļ�ϵ�����
  LCD_Clear(Black);
  // ����LCD��Ļ�ϵı�����ɫ
  LCD_SetBackColor(Black);
  // ����LCD��Ļ�ϵ�������ɫ
  LCD_SetTextColor(White);

  // ʹ�ܶ�ʱ���ж�
  HAL_TIM_Base_Start_IT(&htim6);
  // ʹ�ܶ�ʱ��PWM���
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  // ʹ�ܶ�ʱ�����벶���ж�
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  // ʹ�ܴ��ڽ����ж�
  HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    key_process();
    disp_process();
    rtc_process();
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pa6_duty);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pa7_duty);

    // data_tx();
    if (rx_ptr != 0)
    {
      uint8_t temp = rx_ptr;
      HAL_Delay(1);
      if (temp == rx_ptr)
      {
        data_rx();
      }
    }

    //   uint8_t led_values[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    //   // ����led_values���飬��ÿһ��ֵ����led_disp������ÿ��ѭ���ӳ�500ms
    //   for (int i = 0; i < 8; i++)
    //   {
    //     led_disp(led_values[i]);
    //     HAL_Delay(500);
    //     led_disp(0x00);
    //     HAL_Delay(500);
    //   }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void rtc_process()
{
  if (HAL_RTC_GetTime(&hrtc, &s_time, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RTC_GetDate(&hrtc, &s_date, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

}
// �������ݷ���
void data_tx()
{
  char temp[20];
  sprintf(temp, "frq=%d\r\n", pwm_capture[0].frq);
  HAL_UART_Transmit(&huart1, (uint8_t *)temp, sizeof(temp), 50);
}
// �������ݽ���
void data_rx()
{
  //�жϽ��յ�����
  if(rx_ptr>0)
  {
    if(rx_ptr==22)
    {
      sscanf(rx_data, "%4s:%4s:%12s",car[0].type, car[0].data, car[0].time);
    }
    else
    {
      //���Error
      char temp[20];
      sprintf(temp, "Error\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t *)temp, sizeof(temp), 50);
      
    }
    rx_ptr=0;
    memset(rx_data, 0, sizeof(rx_data));
  }
}

// ���� B2��B3 ���ڲ�����ʾ������Ч��
//   1) B1:����Ϊ�������л����������л� LCD ��ʾ�����ݽ��桱�Ͳ������档
//  2) B2:ÿ�ΰ��� B2 ������PA6 �ֶ�ģʽռ�ձȲ����� 10%��ռ�ձȿɵ�����Χ
//  10% - 90%��ռ�ձȲ������ӵ� 90%���ٴΰ��� B2 ���������� 10%��
//  3) B3:ÿ�ΰ��� B3 ������PA7 �ֶ�ģʽռ�ձȲ����� 10%��ռ�ձȿɵ�����Χ
//  10% - 90%��ռ�ձȲ������ӵ� 90%���ٴΰ��� B3 ���������� 10%��

// key0�л��˵�
void key_process()
{
  // key0�л��˵�
  if (key[0].sigle_flag == 1)
  {
    view_flag++;
    if(view_flag==4)
    {
      view_flag=0;
    }
    LCD_Clear(Black);
    key[0].sigle_flag = 0;
  }

  if (view_flag == 1)
  { // key1����pa6ռ�ձ�
    if (key[1].sigle_flag == 1)
    {
      if (pa6_duty < 90)
      {
        pa6_duty += 10;
      }
      else
      {
        pa6_duty = 10;
      }
      __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pa6_duty);
      key[1].sigle_flag = 0;
    }
    // B3:ÿ�ΰ��� B3 ������PA7 �ֶ�ģʽռ�ձȲ����� 10%��ռ�ձȿɵ�����Χ
    if (key[2].sigle_flag == 1)
    {
      if (pa7_duty < 90)
      {
        pa7_duty += 10;
      }
      else
      {
        pa7_duty = 10;
      }
      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pa7_duty);
      key[2].sigle_flag = 0;
    }
  }
  if (key[3].sigle_flag == 1)
  {
    uint8_t frq_h = pwm_capture[0].frq >> 8;
    uint8_t frq_l = pwm_capture[0].frq & 0xff;
    eeprom_write(1, frq_h);
    HAL_Delay(10);
    eeprom_write(2, frq_l);
    key[3].sigle_flag = 0;
  }
  
  else
  {
    // �������������水�����󴥷�
    key[1].sigle_flag = 0;
    key[2].sigle_flag = 0;
  }
}

void disp_process()
{
 
  if (view_flag == 0)
  {

    char text[30];
    sprintf(text, "       Data    ");
    LCD_DisplayStringLine(Line0, (uint8_t *)text);
    // pwm 1
    sprintf(text, "    FRQ1:%dHz    ", pwm_capture[0].frq);
    LCD_DisplayStringLine(Line2, (uint8_t *)text);
    sprintf(text, "    DUTY1:%.2f%%    ", pwm_capture[0].duty * 100);
    LCD_DisplayStringLine(Line3, (uint8_t *)text);
    // pwm 2
    sprintf(text, "    FRQ2:%dHz    ", pwm_capture[1].frq);
    LCD_DisplayStringLine(Line4, (uint8_t *)text);
    sprintf(text, "    DUTY2:%.2f%%    ", pwm_capture[1].duty * 100);
    LCD_DisplayStringLine(Line5, (uint8_t *)text);
    // adc
    sprintf(text, "    V1:%.2fV    ", get_adc(&hadc1));
    LCD_DisplayStringLine(Line6, (uint8_t *)text);
    sprintf(text, "    V2:%.2fV    ", get_adc(&hadc2));
    LCD_DisplayStringLine(Line7, (uint8_t *)text);
    // i2c eeprom
    uint16_t eeprom_data = (eeprom_read(1) << 8) | eeprom_read(2);
    sprintf(text, "    FRQ_eep:=%d    ", eeprom_data);
    LCD_DisplayStringLine(Line8, (uint8_t *)text);
  }
  else if (view_flag == 1)
  {

    char text[30];
    sprintf(text, "       Para    ");
    LCD_DisplayStringLine(Line0, (uint8_t *)text);
    sprintf(text, "    PA6:%d%%    ", pa6_duty);
    LCD_DisplayStringLine(Line2, (uint8_t *)text);
    sprintf(text, "    PA7:%d%%    ", pa7_duty);
    LCD_DisplayStringLine(Line4, (uint8_t *)text);
  }
  else if(view_flag == 2)
  {
    char text[30];
    sprintf(text, "       Car    ");
    LCD_DisplayStringLine(Line0, (uint8_t *)text);
    sprintf(text, "   type:%s    ", car[0].type);
    LCD_DisplayStringLine(Line2, (uint8_t *)text);
    sprintf(text, "   data:%s    ", car[0].data);
    LCD_DisplayStringLine(Line4, (uint8_t *)text);
    sprintf(text, "   time:%s    ", car[0].time);
    LCD_DisplayStringLine(Line6, (uint8_t *)text);
  }
  if (view_flag == 3)
  {
    char text[30];
    sprintf(text, "       RTC    ");
    LCD_DisplayStringLine(Line0, (uint8_t *)text);
    sprintf(text, "    %02x:%02x:%02x    ", s_time.Hours, s_time.Minutes, s_time.Seconds);
    LCD_DisplayStringLine(Line2, (uint8_t *)text);
    sprintf(text, "    %02x-%02x-%02x    ", s_date.Year, s_date.Month, s_date.Date);
    LCD_DisplayStringLine(Line4, (uint8_t *)text);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
