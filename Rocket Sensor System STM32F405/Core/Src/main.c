/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Global.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for StatsPrintTask */
osThreadId_t StatsPrintTaskHandle;
uint32_t StatsPrintTaskBuffer[ 512 ];
osStaticThreadDef_t StatsPrintTaskControlBlock;
const osThreadAttr_t StatsPrintTask_attributes = {
  .name = "StatsPrintTask",
  .cb_mem = &StatsPrintTaskControlBlock,
  .cb_size = sizeof(StatsPrintTaskControlBlock),
  .stack_mem = &StatsPrintTaskBuffer[0],
  .stack_size = sizeof(StatsPrintTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DataCollection */
osThreadId_t DataCollectionHandle;
uint32_t DataCollectionBuffer[ 512 ];
osStaticThreadDef_t DataCollectionControlBlock;
const osThreadAttr_t DataCollection_attributes = {
  .name = "DataCollection",
  .cb_mem = &DataCollectionControlBlock,
  .cb_size = sizeof(DataCollectionControlBlock),
  .stack_mem = &DataCollectionBuffer[0],
  .stack_size = sizeof(DataCollectionBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RadioComms */
osThreadId_t RadioCommsHandle;
uint32_t RadioCommsBuffer[ 512 ];
osStaticThreadDef_t RadioCommsControlBlock;
const osThreadAttr_t RadioComms_attributes = {
  .name = "RadioComms",
  .cb_mem = &RadioCommsControlBlock,
  .cb_size = sizeof(RadioCommsControlBlock),
  .stack_mem = &RadioCommsBuffer[0],
  .stack_size = sizeof(RadioCommsBuffer),
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for DataWriter */
osThreadId_t DataWriterHandle;
uint32_t DataWriterBuffer[ 1024 ];
osStaticThreadDef_t DataWriterControlBlock;
const osThreadAttr_t DataWriter_attributes = {
  .name = "DataWriter",
  .cb_mem = &DataWriterControlBlock,
  .cb_size = sizeof(DataWriterControlBlock),
  .stack_mem = &DataWriterBuffer[0],
  .stack_size = sizeof(DataWriterBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for RocketApp */
osThreadId_t RocketAppHandle;
uint32_t RocketAppBuffer[ 1024 ];
osStaticThreadDef_t RocketAppControlBlock;
const osThreadAttr_t RocketApp_attributes = {
  .name = "RocketApp",
  .cb_mem = &RocketAppControlBlock,
  .cb_size = sizeof(RocketAppControlBlock),
  .stack_mem = &RocketAppBuffer[0],
  .stack_size = sizeof(RocketAppBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataCollectionQueue */
osMessageQueueId_t DataCollectionQueueHandle;
uint8_t DataCollectionQueueBuffer[ 16 * sizeof( uint64_t ) ];
osStaticMessageQDef_t DataCollectionQueueControlBlock;
const osMessageQueueAttr_t DataCollectionQueue_attributes = {
  .name = "DataCollectionQueue",
  .cb_mem = &DataCollectionQueueControlBlock,
  .cb_size = sizeof(DataCollectionQueueControlBlock),
  .mq_mem = &DataCollectionQueueBuffer,
  .mq_size = sizeof(DataCollectionQueueBuffer)
};
/* Definitions for RadioCommsQueue */
osMessageQueueId_t RadioCommsQueueHandle;
uint8_t RadioCommsQueueBuffer[ 16 * sizeof( uint64_t ) ];
osStaticMessageQDef_t RadioCommsQueueControlBlock;
const osMessageQueueAttr_t RadioCommsQueue_attributes = {
  .name = "RadioCommsQueue",
  .cb_mem = &RadioCommsQueueControlBlock,
  .cb_size = sizeof(RadioCommsQueueControlBlock),
  .mq_mem = &RadioCommsQueueBuffer,
  .mq_size = sizeof(RadioCommsQueueBuffer)
};
/* Definitions for DataWriterQueue */
osMessageQueueId_t DataWriterQueueHandle;
uint8_t DataWriterQueueBuffer[ 16 * sizeof( uint64_t ) ];
osStaticMessageQDef_t DataWriterQueueControlBlock;
const osMessageQueueAttr_t DataWriterQueue_attributes = {
  .name = "DataWriterQueue",
  .cb_mem = &DataWriterQueueControlBlock,
  .cb_size = sizeof(DataWriterQueueControlBlock),
  .mq_mem = &DataWriterQueueBuffer,
  .mq_size = sizeof(DataWriterQueueBuffer)
};
/* Definitions for RocketAppQueue */
osMessageQueueId_t RocketAppQueueHandle;
uint8_t RocketAppQueueBuffer[ 16 * sizeof( uint64_t ) ];
osStaticMessageQDef_t RocketAppQueueControlBlock;
const osMessageQueueAttr_t RocketAppQueue_attributes = {
  .name = "RocketAppQueue",
  .cb_mem = &RocketAppQueueControlBlock,
  .cb_size = sizeof(RocketAppQueueControlBlock),
  .mq_mem = &RocketAppQueueBuffer,
  .mq_size = sizeof(RocketAppQueueBuffer)
};
/* Definitions for DefaultTaskQueue */
osMessageQueueId_t DefaultTaskQueueHandle;
uint8_t DefaultTaskQueueBuffer[ 16 * sizeof( uint64_t ) ];
osStaticMessageQDef_t DefaultTaskQueueControlBlock;
const osMessageQueueAttr_t DefaultTaskQueue_attributes = {
  .name = "DefaultTaskQueue",
  .cb_mem = &DefaultTaskQueueControlBlock,
  .cb_size = sizeof(DefaultTaskQueueControlBlock),
  .mq_mem = &DefaultTaskQueueBuffer,
  .mq_size = sizeof(DefaultTaskQueueBuffer)
};
/* Definitions for delayTimer6Event */
osEventFlagsId_t delayTimer6EventHandle;
osStaticEventGroupDef_t delayTimer6EventControlBlock;
const osEventFlagsAttr_t delayTimer6Event_attributes = {
  .name = "delayTimer6Event",
  .cb_mem = &delayTimer6EventControlBlock,
  .cb_size = sizeof(delayTimer6EventControlBlock),
};
/* Definitions for taskGateEvent */
osEventFlagsId_t taskGateEventHandle;
osStaticEventGroupDef_t taskGateEventControlBlock;
const osEventFlagsAttr_t taskGateEvent_attributes = {
  .name = "taskGateEvent",
  .cb_mem = &taskGateEventControlBlock,
  .cb_size = sizeof(taskGateEventControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM13_Init(void);
void DefaultTask(void *argument);
void DataCollectionTask(void *argument);
void RadioCommsTask(void *argument);
void DataWriterStart(void *argument);
void RocketAppTask(void *argument);

/* USER CODE BEGIN PFP */
void StartDataCollectionTask();
void StartDefaultTask();
void StartRadioCommsTask();
void StartGpsCollectorTask();
void StartDataWriterTask();
void StartRocketAppTask();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// FATFS needs this function.
// We don't have a pin to detect the card so just force it to be there when running.
uint8_t BSP_SD_IsDetected(void)
{
  return SD_PRESENT;
}

void StartCpuMicroSecondTimer()
{
  htim13.Instance->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim13);
}

void StopCpuMicroSecondTimer()
{
  HAL_TIM_Base_Stop_IT(&htim13);
}

void StartStatusUpdatesTimer()
{
  HAL_TIM_Base_Start_IT(&htim14);
}

void StopStatusUpdatesTimer()
{
  HAL_TIM_Base_Stop_IT(&htim14);
}
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
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_FATFS_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  // Accel Int 1 Low Acc
  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  // Accel Int 2 Low Acc
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  initialize_debug_printf();
  PRINTLN("My PCB Rocket System version 1.0");
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of DataCollectionQueue */
  DataCollectionQueueHandle = osMessageQueueNew (16, sizeof(uint64_t), &DataCollectionQueue_attributes);

  /* creation of RadioCommsQueue */
  RadioCommsQueueHandle = osMessageQueueNew (16, sizeof(uint64_t), &RadioCommsQueue_attributes);

  /* creation of DataWriterQueue */
  DataWriterQueueHandle = osMessageQueueNew (16, sizeof(uint64_t), &DataWriterQueue_attributes);

  /* creation of RocketAppQueue */
  RocketAppQueueHandle = osMessageQueueNew (16, sizeof(uint64_t), &RocketAppQueue_attributes);

  /* creation of DefaultTaskQueue */
  DefaultTaskQueueHandle = osMessageQueueNew (16, sizeof(uint64_t), &DefaultTaskQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StatsPrintTask */
  StatsPrintTaskHandle = osThreadNew(DefaultTask, NULL, &StatsPrintTask_attributes);

  /* creation of DataCollection */
  DataCollectionHandle = osThreadNew(DataCollectionTask, NULL, &DataCollection_attributes);

  /* creation of RadioComms */
  RadioCommsHandle = osThreadNew(RadioCommsTask, NULL, &RadioComms_attributes);

  /* creation of DataWriter */
  DataWriterHandle = osThreadNew(DataWriterStart, NULL, &DataWriter_attributes);

  /* creation of RocketApp */
  RocketAppHandle = osThreadNew(RocketAppTask, NULL, &RocketApp_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of delayTimer6Event */
  delayTimer6EventHandle = osEventFlagsNew(&delayTimer6Event_attributes);

  /* creation of taskGateEvent */
  taskGateEventHandle = osEventFlagsNew(&taskGateEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  StartCpuMicroSecondTimer();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1680;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim6.Init.Prescaler = 84;
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
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */
  // On APB1 with 84MHz clock src
  // Set to 1us per counter increment
  // Interrupts every 65,535us so 65.535ms
  // Just increment another 32 bit counter every interrupt
  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 84;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 0xFFFF;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16800;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 25000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 2000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TEST_PIN0_Pin|TEST_PIN1_Pin|TEST_PIN2_Pin|TEST_PIN3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACCEL1_CS_GPIO_Port, ACCEL1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACCEL2_CS_GPIO_Port, ACCEL2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAROMETER_CS_GPIO_Port, BAROMETER_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_MCU_GPIO_Port, LED1_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED2_MCU_Pin|GPS_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DEPLOY_SWITCH_2_Pin|DEPLOY_SWITCH_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RADIO_RESET_Pin */
  GPIO_InitStruct.Pin = RADIO_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_RADIO_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_RADIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_RADIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_INT2_Pin */
  GPIO_InitStruct.Pin = RADIO_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_INT1_Pin */
  GPIO_InitStruct.Pin = RADIO_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RADIO_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TEST_PIN0_Pin TEST_PIN1_Pin TEST_PIN2_Pin TEST_PIN3_Pin */
  GPIO_InitStruct.Pin = TEST_PIN0_Pin|TEST_PIN1_Pin|TEST_PIN2_Pin|TEST_PIN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL1_CS_Pin */
  GPIO_InitStruct.Pin = ACCEL1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACCEL1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL1_INT1_Pin */
  GPIO_InitStruct.Pin = ACCEL1_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ACCEL1_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL2_CS_Pin */
  GPIO_InitStruct.Pin = ACCEL2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACCEL2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACCEL2_INT1_Pin BAROMETER_INT_Pin */
  GPIO_InitStruct.Pin = ACCEL2_INT1_Pin|BAROMETER_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL1_INT2_Pin */
  GPIO_InitStruct.Pin = ACCEL1_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ACCEL1_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAROMETER_CS_Pin */
  GPIO_InitStruct.Pin = BAROMETER_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAROMETER_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL2_INT2_INPUT_Pin */
  GPIO_InitStruct.Pin = ACCEL2_INT2_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ACCEL2_INT2_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP_SW_Pin */
  GPIO_InitStruct.Pin = DIP_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIP_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_MCU_Pin */
  GPIO_InitStruct.Pin = LED1_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_MCU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_MCU_Pin GPS_RESET_Pin */
  GPIO_InitStruct.Pin = LED2_MCU_Pin|GPS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_INT_Pin */
  GPIO_InitStruct.Pin = GPS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPS_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_PPS_INT_Pin */
  GPIO_InitStruct.Pin = GPS_PPS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPS_PPS_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEPLOY_SWITCH_2_Pin DEPLOY_SWITCH_1_Pin */
  GPIO_InitStruct.Pin = DEPLOY_SWITCH_2_Pin|DEPLOY_SWITCH_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_DefaultTask */
/**
  * @brief  Function implementing the StatsPrintTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DefaultTask */
void DefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  StartDefaultTask();
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DataCollectionTask */
/**
 * @brief Function implementing the DataCollection thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DataCollectionTask */
void DataCollectionTask(void *argument)
{
  /* USER CODE BEGIN DataCollectionTask */
  /* Infinite loop */
  StartDataCollectionTask();
  /* USER CODE END DataCollectionTask */
}

/* USER CODE BEGIN Header_RadioCommsTask */
/**
 * @brief Function implementing the RadioComms thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RadioCommsTask */
void RadioCommsTask(void *argument)
{
  /* USER CODE BEGIN RadioCommsTask */
  /* Infinite loop */
  StartRadioCommsTask();
  /* USER CODE END RadioCommsTask */
}

/* USER CODE BEGIN Header_DataWriterStart */
/**
 * @brief Function implementing the DataWriter thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DataWriterStart */
void DataWriterStart(void *argument)
{
  /* USER CODE BEGIN DataWriterStart */
  /* Infinite loop */
  StartDataWriterTask();
  /* USER CODE END DataWriterStart */
}

/* USER CODE BEGIN Header_RocketAppTask */
/**
* @brief Function implementing the RocketApp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RocketAppTask */
void RocketAppTask(void *argument)
{
  /* USER CODE BEGIN RocketAppTask */
  StartRocketAppTask();
  /* USER CODE END RocketAppTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM14)
  {
    ToggleLedOne();
  }
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
