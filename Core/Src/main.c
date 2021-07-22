/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "control.h"
#include "mpu6050.h"
#include "retarget.h"
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
//float Sp_Kp,Sp_Ki;
//float Ba_Kp,Ba_Ki;
extern int speedL,speedR;
extern int Balance_Pwm,Velocity_Pwm;
extern int Encoder_Left, Encoder_Right;
uint8_t Blue_Rx_data;
//region OLED Interface
char OLEDString[8] = {0}; // This is just for OLED Show
//endregion
//region MPU6050 Structure
TM_MPU6050_t MPU6050;
TM_MPU6050_Interrupt_t MPU6050_Interrupts;
//endregion
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void OLED_SHOW_GY(float num)
{
    SSD1306_GotoXY(60, 50);
    snprintf(OLEDString,8, "%.3f", num);
    SSD1306_Puts(OLEDString, &Font_7x10, 1);
    SSD1306_UpdateScreen();
}
void OLED_SHOW_arg(float Ba_kp, float Ba_ki, float Sp_kp, float Sp_ki)
{
    // print Ba_kp
    SSD1306_GotoXY(60, 10); // goto 10, 10
    snprintf(OLEDString,8, "%3f", Ba_kp);
    SSD1306_Puts(OLEDString, &Font_7x10, 1);
    // print ,Ba_ki
    SSD1306_GotoXY(60, 20); // goto 10, 10
    snprintf(OLEDString,8, "%3f", Ba_ki);
    SSD1306_Puts(OLEDString, &Font_7x10, 1);
    // print Sp_kp,
    SSD1306_GotoXY(60, 30);
    snprintf(OLEDString,8, "%3f", Sp_kp);
    SSD1306_Puts(OLEDString, &Font_7x10, 1);
    // print Sp_ki
    SSD1306_GotoXY(60, 40);
    snprintf(OLEDString,8, "%3f", Sp_ki);
    SSD1306_Puts(OLEDString, &Font_7x10, 1);
    SSD1306_UpdateScreen();
}
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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //region BlueTooth Initiation | Status: Off
//RetargetInit(&huart2);
//HAL_UART_Receive_IT(&huart2,&Blue_Rx_data,1);
//printf("Connect Successfully");
  //endregion
  //region OLED Initiation
  SSD1306_Init(); // initialize the display
  // print Ba_kp
  SSD1306_GotoXY(10, 10); // goto 10, 10
  SSD1306_Puts("Ba_kp:", &Font_7x10, 1);
  // print ,Ba_ki
  SSD1306_GotoXY(10, 20); // goto 10, 10
  SSD1306_Puts("Ba_ki:", &Font_7x10, 1);
  // print Sp_kp,
  SSD1306_GotoXY(10, 30);
  SSD1306_Puts("Sp_kp:", &Font_7x10, 1);
  // print Sp_ki
  SSD1306_GotoXY(10, 40);
  SSD1306_Puts("Sp_ki:", &Font_7x10, 1);
  // print GY
  SSD1306_GotoXY(10, 50);
  SSD1306_Puts("Gy:", &Font_7x10, 1);

  //endregion
  //region MPU6050 Initiation
  if(TM_MPU6050_Init(&MPU6050,TM_MPU6050_Device_0,TM_MPU6050_Accelerometer_2G,TM_MPU6050_Gyroscope_500s)==TM_MPU6050_Result_Ok)
  {
    SSD1306_GotoXY(30, 50);
    SSD1306_Puts("OK", &Font_7x10, 1);
  }
  TM_MPU6050_SetDataRate(&MPU6050,TM_MPU6050_DataRate_100Hz);
  TM_MPU6050_EnableInterrupts(&MPU6050);
  //endregion


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //region Car | PWM Enabled | Encoder Initiation
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_ALL);
  //endregion
//Set_Pwm(200,500);
  SSD1306_UpdateScreen(); // update screen
  while (1)
  {

    //region For MPU Transfer data to OLED | Now Status: Off
//    HAL_Delay (100);
    //endregion
    OLED_SHOW_arg(MPU6050.KalmanAngleY,MPU6050.Gy,Encoder_Left,Encoder_Right );
    OLED_SHOW_GY(speedL);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    OLED_SHOW_GY(speedR); // Show the angle of Y
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//  if(htim->Instance == htim2.Instance)
//  {
////    speedR = (int16_t)__HAL_TIM_GET_COUNTER(&htim2); //读取脉冲数 ==>返回脉冲数
//    __HAL_TIM_SET_COUNTER(&htim2,0);      //计数器清零
//  }
//  if(htim->Instance == htim4.Instance)
//  {
////    speedL = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
//    __HAL_TIM_SET_COUNTER(&htim4,0);
//  }
//  OLED_SHOW_GY(speedL);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == huart2.Instance)
//    {
//        printf("%f",Blue_Rx_data);
//        HAL_UART_Receive_IT(&huart2,&Blue_Rx_data,1);
//    }
//}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
