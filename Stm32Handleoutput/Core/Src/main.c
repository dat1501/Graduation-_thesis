/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht.h"
DHT_DataTypedef DHT_DATA;
uint8_t RxData[8];
uint8_t TxData[8];
uint8_t resultUart;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  	volatile static uint8_t sensorLight = 0;
    volatile static uint32_t huminityValue = 0;
    static uint8_t UserOption;
    static uint8_t LightState;
    static uint8_t MotorState;
    static uint8_t FanState;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_Delay(1500);
//  HAL_UART_Receive_IT(&huart1, RxData, 8);
  HAL_UART_Receive_DMA(&huart2, RxData, 8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

	  /* Read Data of DHT11 sensor */
	  DHT_GetData(&DHT_DATA);

	  /* Read Data of Light Sensor*/
	  sensorLight = HAL_GPIO_ReadPin(LighSensor_GPIO_Port, LighSensor_Pin);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, sensorLight);

	  /* Read Data of earth humidity Sensor */
	  HAL_ADC_Start(&hadc1);
	  huminityValue = HAL_ADC_GetValue(&hadc1);
	  huminityValue = huminityValue/40;
	  /* Assign Data of Sensors to TxData*/
	  TxData[0] = sensorLight;
	  TxData[1] = (uint8_t)huminityValue;
	  TxData[2] = DHT_DATA.Temperature;
	  TxData[3] = DHT_DATA.Humidity;

	  HAL_Delay(500);

	  UserOption = RxData[0]; 	// assign UserOption equal User data receive via Uart
	  /* Check user option */
	  if(UserOption == 0)
	  {
		  LightState = RxData[1];	// assign LightState equal Light data receive via Uart
		  MotorState = RxData[2];	// assign MotorState equal Motor data receive via Uart
		  FanState = RxData[3];		// assign FanState equal Fan data receive via Uart
		  /* compare light state if light state on or off as soon as Turn on the light*/
		  if(LightState == 1)
		  {
			  HAL_GPIO_WritePin(Light_GPIO_Port, Light_Pin, LightState);
		  }
		  else HAL_GPIO_WritePin(Light_GPIO_Port, Light_Pin, LightState);

		  /* compare Motor state if Motor state on or off as soon as Turn on the Motor*/
		  if(MotorState == 1)
		  {
			  HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, MotorState);
		  }
		  else HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, MotorState);

		  /* compare Fan state if Motor Fan on or off as soon as Turn on the Fan*/
		  if(FanState == 1)
		  {
			  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, FanState);
		  }
		  else HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, FanState);

		  TxData[4] = LightState;
		  TxData[5] = MotorState;
		  TxData[6] = FanState;

	  }
	  else if(UserOption == 1)
	  {
		  /*To do: control output to handle Motor, light, Fan*/
		  if(sensorLight == 1)
		  {
			  HAL_GPIO_WritePin(LighSensor_GPIO_Port, LighSensor_Pin, GPIO_PIN_SET);
			  LightState = 1;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(LighSensor_GPIO_Port, LighSensor_Pin, GPIO_PIN_RESET);
			  LightState = 0;
		  }
		  /* */
		  if(DHT_DATA.Temperature >= 40 || DHT_DATA.Humidity <=70)
		  {
			  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_SET);
			  FanState = 1;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_RESET);
			  FanState = 0;
		  }

		  if(huminityValue <40)
		  {
			  HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_SET);
			  MotorState = 1;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
			  MotorState = 0;
		  }
		  TxData[4] = LightState;
		  TxData[5] = MotorState;
		  TxData[6] = FanState;
	  }
	  resultUart = HAL_UART_Transmit(&huart2, TxData, 8, 100);

	  if(resultUart == HAL_OK)
	  {
	  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == huart1.Instance)
//	{
//		HAL_UART_Receive_IT(&huart1, RxData, 8);
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
//	}
//
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
