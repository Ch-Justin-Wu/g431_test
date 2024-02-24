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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "lcd.h"
#include "stdio.h"
#include "interrupt.h"
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
uint32_t i_text = 5;
bool view_flag = 0;

// pwm占空比
uint8_t pa6_duty = 10;
uint8_t pa7_duty = 10;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void key_process();
void disp_process();
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
  /* USER CODE BEGIN 2 */
  led_disp(0x00); // led初始化
  LCD_Init();     // lcd屏幕初始化

  // 清除LCD屏幕上的内容
  LCD_Clear(Black);
  // 设置LCD屏幕上的背景颜色
  LCD_SetBackColor(Black);
  // 设置LCD屏幕上的文字颜色
  LCD_SetTextColor(White);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);//启动PWM输出
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);//启动输入捕获
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    key_process();
    disp_process();
    // int8_t text[30];

    // sprintf(text, "   CNBR:%d    ", i_text);
    // LCD_DisplayStringLine(Line9, text);

    // if (key[0].sigle_flag == 1)
    // {
    //   sprintf(text, "   KEY0 DOWN    ");
    //   LCD_DisplayStringLine(Line8, text);
    //   key[0].sigle_flag = 0;
    // }
    // if (key[0].long_flag == 1)
    // {
    //   sprintf(text, "   KEY0 LONG    ");
    //   LCD_DisplayStringLine(Line8, text);
    //   key[0].long_flag = 0;
    // }

    // if (key[1].sigle_flag == 1)
    // {
    //   sprintf(text, "   KEY1 DOWN    ");
    //   LCD_DisplayStringLine(Line8, text);
    //   key[1].sigle_flag = 0;
    // }
    // if (key[2].sigle_flag == 1)
    // {
    //   sprintf(text, "   KEY2 DOWN    ");
    //   LCD_DisplayStringLine(Line8, text);
    //   key[2].sigle_flag = 0;
    // }
    // if (key[3].sigle_flag == 1)
    // {
    //   sprintf(text, "   KEY3 DOWN    ");
    //   LCD_DisplayStringLine(Line8, text);
    //   key[3].sigle_flag = 0;
    // }

    //   uint8_t led_values[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    //   // 遍历led_values数组，将每一个值传入led_disp函数，每次循环延迟500ms
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
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
// 按键 B2、B3 仅在参数显示界面有效。
//   1) B1:定义为“界面切换”按键，切换 LCD 显示“数据界面”和参数界面。
//  2) B2:每次按下 B2 按键，PA6 手动模式占空比参数加 10%，占空比可调整范围
//  10% - 90%，占空比参数增加到 90%后，再次按下 B2 按键，返回 10%。
//  3) B3:每次按下 B3 按键，PA7 手动模式占空比参数加 10%，占空比可调整范围
//  10% - 90%，占空比参数增加到 90%后，再次按下 B3 按键，返回 10%。

// key0切换菜单
void key_process()
{
  // key0切换菜单
  if (key[0].sigle_flag == 1)
  {
    view_flag = !view_flag;
    key[0].sigle_flag = 0;
  }

  if (view_flag == 1)
  { // key1增加pa6占空比
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
    // B3:每次按下 B3 按键，PA7 手动模式占空比参数加 10%，占空比可调整范围
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
  else
  {
    // 避免在其他界面按键被误触发
    key[1].sigle_flag = 0;
    key[2].sigle_flag = 0;
  }
}

void disp_process()
{
  static int last_view_flag = -1; // 初始化为一个不可能的值
  if (last_view_flag != view_flag)
  {
    LCD_Clear(Black); // 清屏
    last_view_flag = view_flag;
  }
  if (view_flag == 0)
  {

    char text[30];
    sprintf(text, "       Data    ");
    LCD_DisplayStringLine(Line0, (uint8_t *)text);
    sprintf(text,"    FRQ1:%dHz    ", pwm_capture[0].frq);
    LCD_DisplayStringLine(Line2, (uint8_t *)text);
    sprintf(text,"    FRQ2:%dHz    ", pwm_capture[1].frq);
    LCD_DisplayStringLine(Line4, (uint8_t *)text);
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
