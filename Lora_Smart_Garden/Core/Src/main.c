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
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "dht.h"
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
LoRa myLoRa;
uint8_t LoraState = 0;
static uint8_t TxData[10] = {0};
static uint8_t RxData[10] = {0};
static uint8_t RxDataUart[8];
static uint8_t TxDataUart[8];
uint8_t Rx_start = 0;
volatile static uint8_t ledData;
DHT_DataTypedef DHT_DATA;

uint8_t LocalAddress = 0x02; /* Addess of this devince (Slaver)*/
uint8_t DestinationAddress = 0x01; /* destination address is address of Master (ESP32)*/

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t ret;
//  HAL_UART_Receive_IT(&huart1, RxDataUart, 8);
  HAL_UART_Receive_DMA(&huart2, RxDataUart, 8);
  myLoRa = newLoRa();

  myLoRa.CS_port         = NSS_GPIO_Port;
  myLoRa.CS_pin          = NSS_Pin;
  myLoRa.reset_port      = RST_GPIO_Port;
  myLoRa.reset_pin       = RST_Pin;
  myLoRa.DIO0_port       = DIO0_GPIO_Port;
  myLoRa.DIO0_pin        = DIO0_Pin;
  myLoRa.hSPIx           = &hspi1;

  if(LoRa_init(&myLoRa))
  {
	  LoraState = 1;
  }
  LoRa_startReceiving(&myLoRa);
//  HAL_Delay(2000);
//  TxData[0] = LocalAddress;
//  for(index = 1; index < 10; index++)
//  {
//	  TxData[index] = (uint8_t)str[index-1];
//  }
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(LoRa_transmit(&myLoRa, TxData, 8, 1000))
	  {
	  	  HAL_Delay(150);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  	  HAL_IWDG_Refresh(&hiwdg);
	  	  Rx_start = 0;
	  };

	  ret = HAL_UART_Transmit(&huart1, TxDataUart, 8, 1000);
	  if(ret == HAL_OK)
	  {
//		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	  }
	  TxData[0] = 0x01; 			// Address receiver;
	  TxData[1] = RxDataUart[0];  	//  Sensor Light
	  TxData[2] = RxDataUart[1];  	// Sensor earth humidity
	  TxData[3] = RxDataUart[2];  	// Air Temperature
	  TxData[4] = RxDataUart[3];  	// Air humidity
	  TxData[5] = RxDataUart[4];  	// Light State
	  TxData[6] = RxDataUart[5];  	// Motor State
	  TxData[7] = RxDataUart[6];  	// Fan State
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == DIO0_Pin)
	{
		Rx_start = 1;
		LoRa_receive(&myLoRa,RxData,8);
		if(ledData != RxData[1])
		{
			ledData = RxData[1];
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RxData[1]);

//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		}
		TxDataUart[0] = RxData[1]; // user option
		TxDataUart[1] = RxData[2]; // light control
		TxDataUart[2] = RxData[3]; // motor control
		TxDataUart[3] = RxData[4]; // fan control
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	}

//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == huart1.Instance)
//	{
//		HAL_UART_Receive_IT(&huart1, RxDataUart, 4);
//		TxData[0] = 0x01; 		// Address receiver;
//		TxData[1] = RxDataUart[0];  //  Sensor Light
//		TxData[2] = RxDataUart[1];  // Sensor earth humidity
//		TxData[3] = RxDataUart[2];  // Air Temperature
//		TxData[4] = RxDataUart[3];  // Air humidity
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//	}
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
