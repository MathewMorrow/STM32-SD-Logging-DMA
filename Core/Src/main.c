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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VCP.h"
#include "blackbox.h"
#include "time.h"
#include "cpu_load.h"
#include "project_resources.h"
#include "console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define microsTimer htim2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_tx;
DMA_HandleTypeDef hdma_sdio_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
//cpuLoad_t cpuLoadDelay;
//cpuLoad_t cpuLoadBlackbox;
//cpuLoad_t cpuLoadUSB;
//cpuLoad_t cpuLoadCLI;
//
//uint32_t delayMicroseconds = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//void printCLI();
//void streamData();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USB VCP CLI */
//uint8_t sprintfBuffer[512] = {'\0'};
//
//uint32_t lastMillis = 0;
//uint32_t stopBlink = 0;
//uint32_t streamSDIO = 0;
//uint16_t writePeriod = 10;
//
///* SD card benchmarking */
//uint8_t doSDBenchmark = 0;
//
//volatile uint8_t simNewDataFlag = 0;
//
///* USB VCP CLI */
//uint8_t streamDataSet = 0;
//uint8_t streamBuffRem = 0;

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
  MX_CAN2_Init();
  MX_SDIO_SD_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);

  microsInit(&microsTimer);
  cpuLoad_Init(&cpuLoadDelay);
  cpuLoad_Init(&cpuLoadBlackbox);
  cpuLoad_Init(&cpuLoadUSB);
  cpuLoad_Init(&cpuLoadCLI);


  uint8_t sdResult;
  sdResult = blackbox_Init();

  if(sdResult == BLACKBOX_OK)
  {
	  uint8_t sdHeader[256];
	  sprintf(sdHeader, "Param1\n\rParam2\n\rParam3\n\rParam4\n\rParam5\n\r$$ end of header\n\r");
	  blackbox_SetHeader(sdHeader, strlen(sdHeader));

//	  sdResult = blackbox_CreateFile(loggingFileSize);
  }
  if(sdResult != FR_OK)
  {
//	  Error_Handler();
	  uint32_t sdErrorStartMillis = HAL_GetTick();
	  uint32_t sdErrorBlinkTime = sdErrorStartMillis;
	  while(HAL_GetTick() - sdErrorStartMillis < 5000)
	  {
		  if(HAL_GetTick() - sdErrorBlinkTime > 50)
		  {
			  sdErrorBlinkTime = HAL_GetTick();
			  HAL_GPIO_TogglePin(LED_FLT_GPIO_Port, LED_FLT_Pin);
		  }
	  }
  }


  cpuLoad_Reset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_SET);

	  // Runtime LED blinking
	  if(HAL_GetTick() - lastMillis >= 200){
		  lastMillis = HAL_GetTick();
		  if(!stopBlink) HAL_GPIO_TogglePin(LED_FLT_GPIO_Port, LED_FLT_Pin);
	  }

	  cpuLoad_Start(&cpuLoadBlackbox);
	  if(streamSDIO)
	  {
		  static uint32_t lastSDMillis = 0;
		  if(simNewDataFlag)
		  {
			  simNewDataFlag = 0;
HAL_GPIO_WritePin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin, GPIO_PIN_SET);

			  uint8_t sdBuff[512] = {'\0'};
			  static uint32_t numLogs = 0;
//			  sprintf(sdBuff, "%u,4321,5678,8765,1111,2222,3333,4444,5555,6666,7777,8888,9999,\n", getMircos());
//			  sprintf(sdBuff, "%u,4321,5678,8765,1111,2222,3333,4444,5555,6666,7777,8888,9999,5678,6789,9876,8765,6543,4567,1245,5432,7654,8796,ENDS,\n", ++numLogs);

			  /* 84uS - 100uS to use sprintf */
//			  sprintf(sdBuff, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,\n",
//					  getMircos(),
//					  (int16_t)(gyroPitchRaw*10.0f),
//					  (int16_t)(gyroPitchFilt*10.0f),
//					  (int16_t)(gyroRollRaw*10.0f),
//					  (int16_t)(gyroRollFilt*10.0f),
//					  (int16_t)(gyroYawRaw*10.0f),
//					  (int16_t)(gyroYawFilt*10.0f),
//					  (int16_t)(pitchP*1000.0f),
//					  (int16_t)(pitchI*1000.0f),
//					  (int16_t)(pitchD*1000.0f)
//					  );
//			  blackbox_BufferData(sdBuff, strlen(sdBuff));

			  /* 84uS - 25uS to use sprintf */
			  uint32_t microsNow = getMircos();
			  uint8_t frameIndex = 0;
			  blackboxFrame[frameIndex++] = 'S';
			  blackboxFrame[frameIndex++] = 'O';
			  blackboxFrame[frameIndex++] = 'F';
			  blackboxFrame[frameIndex++] = (microsNow >> 24) & 0xFF;
			  blackboxFrame[frameIndex++] = (microsNow >> 16) & 0xFF;
			  blackboxFrame[frameIndex++] = (microsNow >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = microsNow & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(gyroPitchRaw*10.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(gyroPitchRaw*10.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(gyroPitchFilt*10.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(gyroPitchFilt*10.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(gyroRollRaw*10.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(gyroRollRaw*10.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(gyroRollFilt*10.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(gyroRollFilt*10.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(gyroYawRaw*10.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(gyroYawRaw*10.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(gyroYawFilt*10.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(gyroYawFilt*10.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(pitchP*1000.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(pitchP*1000.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(pitchI*1000.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(pitchI*1000.0f)) & 0xFF;
			  blackboxFrame[frameIndex++] = (((int16_t)(pitchD*1000.0f)) >> 8) & 0xFF;
			  blackboxFrame[frameIndex++] = ((int16_t)(pitchD*1000.0f)) & 0xFF;
			  blackbox_BufferData(blackboxFrame, (frameIndex));


HAL_GPIO_WritePin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin, GPIO_PIN_RESET);
		  }
	  }
//	  HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_SET);
	  blackbox_ProcessData(); // If new dataPending / send to DMA takes 55uS otherwise <1uS
//	  HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_RESET);
	  cpuLoad_End(&cpuLoadBlackbox);

	  cpuLoad_Start(&cpuLoadDelay);
//	  if(simNewDataFlag)
//	  {
//		  simNewDataFlag = 0;
//		  uint32_t tickstart = getMircos();
//		  while((getMircos() - tickstart) < delayMicroseconds)
//		  {
//
//		  }
//	  }
	  cpuLoad_End(&cpuLoadDelay);

//	 blackboxSDBenchmark(doSDBenchmark);
	  /* USB VCP CLI */
	  cpuLoad_Start(&cpuLoadCLI);
	  if(vcp_getRxCount()){printCLI();}
	  cpuLoad_End(&cpuLoadCLI);
	  streamData();
	  cpuLoad_Process();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_RESET);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 252;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B; // cubeMX issue must default to 1 bit wide bus then init function updates to 4
  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 26000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PIN_TEST1_Pin|LED2_Pin|LED_FLT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BMP_INT1_Pin */
  GPIO_InitStruct.Pin = BMP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMP_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP_INT2_Pin */
  GPIO_InitStruct.Pin = BMP_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMP_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_TEST2_Pin */
  GPIO_InitStruct.Pin = PIN_TEST2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_TEST2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_TEST1_Pin LED2_Pin LED_FLT_Pin */
  GPIO_InitStruct.Pin = PIN_TEST1_Pin|LED2_Pin|LED_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SDDETECT_Pin */
  GPIO_InitStruct.Pin = SDDETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDDETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


int _write(int file, char *data, int len)
{
	uint8_t usbReturnStatus = CDC_Transmit_FS((uint8_t*) data, len);
	if(usbReturnStatus) return 0;
//   if(usbReturnStatus)
//   {
//	   for(int i=0; i<500; i++)
//	   {
//		   usbReturnStatus = CDC_Transmit_FS((uint8_t*) data, len);
//		   if(usbReturnStatus == 0)
//			   {
//			   return len;
//			   }
//	   }
//	   return 0;
//   }
   return len;
}

//void vcp_TransmitCplt()
//{
//	memset(sprintfBuffer, '\0', strlen(sprintfBuffer));
//}

//void printCLI(void)
//{
////	char read_Value = usart_Read(&huart1);
//	char read_Value = vcp_Read();
//
//	/* Reset all streaming variables */
//	memset(sprintfBuffer, '\0', strlen(sprintfBuffer));
//	if(read_Value == 'h')
//	{
//		sprintf(sprintfBuffer,"\n\r"
//				"a - Blink\n\r"
//				"d - SD Detect Pin\n\r"
//				"1 - Close File\n\r"
//				"2 - Sync\n\r"
//				"3 - Create File\n\r"
//				"s - SteamSDIO\n\r"
//				"f - fetFree\n\r"
//				"m - Card Info\n\r"
//				"c - Card States\n\r"
//				"v - Benchmark Stats\n\r"
//				"z - SD Benchmark\n\r"
//				"x - Write kBps\n\r"
//				"l - CPU Load\n\r"
//				"o - Delay--\n\r"
//				"p - Delay++\n\r"
//				"r - Reset CPU load\n\r"
//				"\n"
//				);
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'a')
//	{
//		if(stopBlink) stopBlink = 0;
//		else stopBlink = 1;
//	}
//	else if(read_Value == 'd')
//	{
//		sprintf(sprintfBuffer,"SD Inserted\n\r");
//		if(HAL_GPIO_ReadPin(SDDETECT_GPIO_Port, SDDETECT_Pin)){
//			sprintf(sprintfBuffer,"NO SD INSERTED\n\r");
//		}
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == '1')
//	{
//		uint8_t res = blackbox_CloseFile();
//		if(res == 0){
//			sprintf(sprintfBuffer,"Closed Successfully\n\r");
//		}
//		else{
//			sprintf(sprintfBuffer,"Close Failed %u\n\r", res);
//		}
//
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == '2')
//	{
//		HAL_GPIO_WritePin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin, GPIO_PIN_SET);
//		blackbox_Sync();
//		HAL_GPIO_WritePin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin, GPIO_PIN_RESET);
//	}
//	else if(read_Value == '3')
//	{
//		uint8_t res = blackbox_CreateFile(MEGABYTE*100);
//		if(res == 0){
//			sprintf(sprintfBuffer,"File Created\n\r");
//		}
//		else{
//			sprintf(sprintfBuffer,"File Failed %u\n\r", res);
//		}
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 's')
//	{
//		 streamSDIO = streamSDIO ^ 1;
//	}
//	else if(read_Value == 'f')
//	{
//		if(!streamDataSet)
//		{
//			streamDataSet = 1;
//			streamBuffRem = 1;
//		}
//		else
//		{
//			streamDataSet = 0;
//			streamBuffRem = 0;
//		}
//	}
//	else if(read_Value == 'm')
//	{
//		sprintf(sprintfBuffer,"\n\r"
//				"CardType %u\n\r"
//				"CardVersion %u\n\r"
//				"Class %u\n\r"
//				"RelCardAdd %u\n\r"
//				"BlockNbr %u\n\r"
//				"BlockSize %u\n\r"
//				"LogBlockNbr %u\n\r"
//				"LogBlockSize %u\n\r"
//				"\n",
//				sdGetCardType(),
//				sdGetCardVersion(),
//				sdGetCardClass(),
//				sdGetCardAdd(),
//				sdGetCardBlockNber(),
//				sdGetCardBlockSize(),
//				sdGetCardLogBlkNber(),
//				sdGetCardLogBlockSize()
//				);
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'c')
//	{
//		sprintf(sprintfBuffer,"\n\r"
//				"Ready %u\n\r"
//				"Identification %u\n\r"
//				"Standby %u\n\r"
//				"Transfer %u\n\r"
//				"Sending %u\n\r"
//				"Receiving %u\n\r"
//				"Programming %u\n\r"
//				"Disconnected %u\n\r"
//				"DMA Busy %u\n\r"
//				"Buffer Full %u\n\r"
//				"Error %u\n\r"
//				"File Full %u\n\r"
//				"\n",
//				SD_CARD_READY_COUNT,
//				SD_CARD_IDENTIFICATION_COUNT,
//				SD_CARD_STANDBY_COUNT,
//				SD_CARD_TRANSFER_COUNT,
//				SD_CARD_SENDING_COUNT,
//				SD_CARD_RECEIVING_COUNT,
//				SD_CARD_PROGRAMMING_COUNT,
//				SD_CARD_DISCONNECTED_COUNT,
//				SD_CARD_DMA_BUSY_COUNT,
//				SD_CARD_BUFFER_FULL_COUNT,
//				SD_CARD_ERROR_COUNT,
//				blackboxIsFileFull()
//				);
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'v')
//	{
//		sprintf(sprintfBuffer,"\n\r"
//				"<300us %u\n\r"
//				"<500us %u\n\r"
//				"<1ms %u\n\r"
//				"<2ms %u\n\r"
//				"<10ms %u\n\r"
//				"<200mS %u\n\r"
//				">200mS %u\n\r"
//				"Max ms %u\n\r"
//				"\n",
//				SD_CARD_LATENCY_1_300,
//				SD_CARD_LATENCY_301_500,
//				SD_CARD_LATENCY_501_1000,
//				SD_CARD_LATENCY_1001_2000,
//				SD_CARD_LATENCY_2001_10000,
//				SD_CARD_LATENCY_10001_200000,
//				SD_CARD_LATENCY_200msplus,
//				SD_CARD_LATENCY_MAX
//				);
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'z')
//	{
//		doSDBenchmark = doSDBenchmark ^ 1;
//	}
//	else if(read_Value == 'x')
//	{
//		sprintf(sprintfBuffer, "Write kBps %u\n\r", sdGetWritekBps());
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'l')
//	{
//		sprintf(sprintfBuffer,
//				"CPU Load %u\n\r"
//				"Min Loop %u\n\r"
//				"Max Loop %u\n\r",
//				cpuLoad_GetPercentLoad(),
//				cpuLoad_GetMinLoopTime(),
//				cpuLoad_GetMaxLoopTime());
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'o')
//	{
//		if(delayMicroseconds > 1) delayMicroseconds--;
//		sprintf(sprintfBuffer, "Delay %u\n\r", delayMicroseconds);
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'p')
//	{
//		if(delayMicroseconds < 100000) delayMicroseconds++;
//		sprintf(sprintfBuffer, "Delay %u\n\r", delayMicroseconds);
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else if(read_Value == 'r')
//	{
//		cpuLoad_Reset();
//		sprintf(sprintfBuffer, "CPU Load Calculation Reset\n\r");
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//	else
//	{
//		sprintf(sprintfBuffer, "Invalid Command");
//		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//	}
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	simNewDataFlag = 1;
}

//void streamData()
//{
//	static uint32_t printLastTimeMillis = 0;
//	if(streamDataSet)
//	{
//		if(HAL_GetTick() - printLastTimeMillis >= 5)
//		{
//			printLastTimeMillis = HAL_GetTick();
//			if(streamBuffRem)
//			{
//				sprintf(sprintfBuffer, "%u\n", blackboxGetBufferAvailable());
//				CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
//			}
//		}
//	}
//}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  HAL_GPIO_WritePin(LED_FLT_GPIO_Port, LED_FLT_Pin, GPIO_PIN_SET);
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
