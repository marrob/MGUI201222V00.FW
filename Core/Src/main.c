/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "mx25.h"
#include <stdio.h>
#include <queue.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_BUFFER_SIZE    40
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*** MCP3208 ***/
#define MCP320X_CH0          0
#define MCP320X_CH1          1
#define MCP320X_CH2          2
#define MCP320X_CH3          3
#define MCP320X_CON_SINGLE_END  (1<<3)

/*** SDRAM ***/
/* SDRAM refresh counter (100Mhz SD clock)    */
#define SDRAM_REFRESH_COUNT                      ((uint32_t)0x0603)
#define SDRAM_TIMEOUT                            ((uint32_t)0xFFFF)
#define SDRAM_DEVICE_ADDR                        ((uint32_t)0xD0000000)
/* SDRAM device size in MBytes */
#define SDRAM_DEVICE_SIZE                        ((uint32_t)0x800000)
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
#define SDRAM_BUFFER_SIZE                        ((uint32_t)0x1000)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c2;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_usart1_rx;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UsbRx_Task */
osThreadId_t UsbRx_TaskHandle;
const osThreadAttr_t UsbRx_Task_attributes = {
  .name = "UsbRx_Task",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LiveLed_Task */
osThreadId_t LiveLed_TaskHandle;
const osThreadAttr_t LiveLed_Task_attributes = {
  .name = "LiveLed_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RS485Rx_Task */
osThreadId_t RS485Rx_TaskHandle;
const osThreadAttr_t RS485Rx_Task_attributes = {
  .name = "RS485Rx_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RS485Tx_Task */
osThreadId_t RS485Tx_TaskHandle;
const osThreadAttr_t RS485Tx_Task_attributes = {
  .name = "RS485Tx_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USBUartRxQueue */
osMessageQueueId_t USBUartRxQueueHandle;
const osMessageQueueAttr_t USBUartRxQueue_attributes = {
  .name = "USBUartRxQueue"
};
/* USER CODE BEGIN PV */


Device_t Device;
__IO unsigned long RTOSRunTimeStatTick;

static char USB_UART_RxBuffer[UART_BUFFER_SIZE] __attribute__ ((aligned (32)));
static char RS485_UART_RxBuffer[UART_BUFFER_SIZE] __attribute__ ((aligned (32)));

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART7_Init(void);
void StartDefaultTask(void *argument);
void UsbRxTask(void *argument);
void LiveLedTask(void *argument);
void RS485RxTask(void *argument);
void RS485TxTask(void *argument);

/* USER CODE BEGIN PFP */
/*** Live LED***/
void LiveLedOff(void);
void LiveLedOn(void);

/*** LCD ***/
void LCD_Enable(void);

void ConsoleWrite(char *str);
void UsbParser(char *request);
void UsbUartTx(char *str);

/*** RS485 ***/
void RS485DirTx(void);
void RS485DirRx(void);
void RS485Parser(char *response);
void RS485UartTx(char *str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SayHelloWorld(uint8_t p)
{
  printf("Hello World");
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_DMA_Init();
  MX_LTDC_Init();
  MX_CRC_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_UART7_Init();
  MX_TouchGFX_Init();
  /* USER CODE BEGIN 2 */

  /*** Display ***/
  LCD_Enable();

  /*** Flash ***/
  MX25_Init(&hqspi);
  MX25_EnableMemoryMappedMode(&hqspi);
  HAL_NVIC_DisableIRQ(QUADSPI_IRQn);


  DisplayLightInit(&htim1);

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
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of USBUartRxQueue */
  USBUartRxQueueHandle = osMessageQueueNew (16, 80, &USBUartRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UsbRx_Task */
  UsbRx_TaskHandle = osThreadNew(UsbRxTask, NULL, &UsbRx_Task_attributes);

  /* creation of LiveLed_Task */
  LiveLed_TaskHandle = osThreadNew(LiveLedTask, NULL, &LiveLed_Task_attributes);

  /* creation of RS485Rx_Task */
  RS485Rx_TaskHandle = osThreadNew(RS485RxTask, NULL, &RS485Rx_Task_attributes);

  /* creation of RS485Tx_Task */
  RS485Tx_TaskHandle = osThreadNew(RS485TxTask, NULL, &RS485Tx_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //ConsoleWrite("FreeRTOS osKernelStart()");
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
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
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

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
  hi2c2.Init.Timing = 0x00C0EAFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 19;
  hltdc.Init.VerticalSync = 2;
  hltdc.Init.AccumulatedHBP = 159;
  hltdc.Init.AccumulatedVBP = 22;
  hltdc.Init.AccumulatedActiveW = 1183;
  hltdc.Init.AccumulatedActiveH = 622;
  hltdc.Init.TotalWidth = 1343;
  hltdc.Init.TotalHeigh = 634;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 1024;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 600;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 0xFF;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 1024;
  pLayerCfg.ImageHeight = 600;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 100;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 25;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */
  HAL_UART_Receive_DMA (&huart7, (uint8_t*)RS485_UART_RxBuffer, UART_BUFFER_SIZE);
  /* USER CODE END UART7_Init 2 */

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
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_DMA (&huart1, (uint8_t*)USB_UART_RxBuffer, UART_BUFFER_SIZE);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  __IO uint32_t tmpmrd = 0;
  FMC_SDRAM_CommandTypeDef Command;

  /* Step 1: Configure a clock configuration enable command */
  Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */
  Command.CommandMode            = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 4: Configure an Auto Refresh command */
  Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 8;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                     SDRAM_MODEREG_CAS_LATENCY_3           |\
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(&hsdram1, SDRAM_REFRESH_COUNT);

  //Deactivate speculative/cache access to first FMC Bank to save FMC bandwidth
  FMC_Bank1->BTCR[0] = 0x000030D2;
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_EN_GPIO_Port, DO_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS_485_DIR_GPIO_Port, RS_485_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AI_CS_Pin|AI_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LIVE_LED_Pin|PER_LD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TS_RST_GPIO_Port, TS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin|PER_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, PSP_EN_Pin|PER_CLR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DO_EN_Pin */
  GPIO_InitStruct.Pin = DO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_485_DIR_Pin */
  GPIO_InitStruct.Pin = RS_485_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS_485_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AI_CS_Pin AI_MOSI_Pin */
  GPIO_InitStruct.Pin = AI_CS_Pin|AI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AI_MISO_Pin */
  GPIO_InitStruct.Pin = AI_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AI_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIVE_LED_Pin PER_LD_Pin DISP_EN_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin|PER_LD_Pin|DISP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_RST_Pin */
  GPIO_InitStruct.Pin = TS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TS_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_INT_Pin */
  GPIO_InitStruct.Pin = TS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TS_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CLK_Pin PER_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI_CLK_Pin|PER_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PER_MISO_Pin */
  GPIO_InitStruct.Pin = PER_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PER_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PSP_EN_Pin PER_CLR_Pin */
  GPIO_InitStruct.Pin = PSP_EN_Pin|PER_CLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* printf -------------------------------------------------------------------*/
/*
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  return len;
}
*/

void ConsoleWrite(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

/* DCDC ----------------------------------------------------------------------*/
void PSP_Enable(){
  HAL_GPIO_WritePin(PSP_EN_GPIO_Port, PSP_EN_Pin, GPIO_PIN_SET);
}

/* Display---------------------------------------------------------------------*/
void LCD_Enable(){
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_SET);
}
void LCD_Disable(){
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_RESET);
}

/* Flash ---------------------------------------------------------------------*/
int Read (uint32_t address, uint32_t size, uint8_t* buffer)
{
  int i = 0;
  for (i=0; i < size;i++)
  {
    *(uint8_t*)buffer++ = *(uint8_t*)address;
    address ++;
  }
  return 1;
}


/* Temperature  --------------------------------------------------------------*/
void SwSPI_TransmittReceive(uint8_t *tx, uint8_t *rx, int length)
{
  for(uint8_t j=0; j < length; j++)
  {
    uint8_t rx_mask = 0x80;
    uint8_t tx_mask = 0x80;
    for(uint8_t i = 0; i<8;i++)
    {
      HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET);
      if(tx[j] & tx_mask)
        HAL_GPIO_WritePin(AI_MOSI_GPIO_Port, AI_MOSI_Pin, GPIO_PIN_SET);
      else
        HAL_GPIO_WritePin(AI_MOSI_GPIO_Port, AI_MOSI_Pin, GPIO_PIN_RESET);
      tx_mask>>=1;
      if(HAL_GPIO_ReadPin(AI_MISO_GPIO_Port, AI_MISO_Pin) == GPIO_PIN_SET)
        rx[j] |= rx_mask;
      else
        rx[j] &= ~rx_mask;
      rx_mask>>=1;
      HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET);
  }
}
  #define MACP320X_ARRAY_SIZE 3
  uint8_t result[MACP320X_ARRAY_SIZE];
  uint8_t param[MACP320X_ARRAY_SIZE];
/*
 *
 *config:
 *   MSB                          LSB
 *   0  | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
 *   X  | X | X | X |S/D|D2 |D1 |D0 |
 */
uint16_t MCP320x_GetValue(uint8_t config)
{
  uint16_t value = 0;
  param[0] = 0x01;
  param[1] = config << 0x04;
  param[2] = 0;
  HAL_GPIO_WritePin(AI_CS_GPIO_Port, AI_CS_Pin, GPIO_PIN_RESET);
  SwSPI_TransmittReceive(param, result, MACP320X_ARRAY_SIZE);
  HAL_GPIO_WritePin(AI_CS_GPIO_Port, AI_CS_Pin, GPIO_PIN_SET);
  value = (result[1]&0x03)<<8;
  value |= result[2];
  return value;
}

double GetTemperature(uint8_t channel)
{
  uint16_t adc = MCP320x_GetValue(MCP320X_CON_SINGLE_END | channel /* MCP320X_CH0*/);
  double volts = adc * 2.5/1024;
  double temp = (-2.3654*volts*volts) + (-78.154*volts) + 153.857;
  return temp;
}

/* DIO -----------------------------------------------------------------------*/
void DIO_Init(void){
  /*** DO Clear ***/
  HAL_GPIO_WritePin(PER_CLR_GPIO_Port, PER_CLR_Pin, GPIO_PIN_RESET);
  DelayUs(1);
  HAL_GPIO_WritePin(PER_CLR_GPIO_Port, PER_CLR_Pin, GPIO_PIN_SET);
  DelayUs(1);
}

void DIO_Clock(void)
{
  HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET);
  DelayUs(5);
  HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET);
  DelayUs(5);
}

uint16_t GetInputs(void)
{
  return Device.Inputs;
}

uint16_t ReadInputs(void)
{
  uint16_t retval = 0;
  uint16_t mask = 0x8000;

  /*** Load ***/
  HAL_GPIO_WritePin(PER_LD_GPIO_Port, PER_LD_Pin, GPIO_PIN_RESET);
  DIO_Clock();
  HAL_GPIO_WritePin(PER_LD_GPIO_Port, PER_LD_Pin, GPIO_PIN_SET);

  for(uint8_t j = 0; j < 16; j++)
  {
    if(HAL_GPIO_ReadPin(PER_MISO_GPIO_Port, PER_MISO_Pin) == GPIO_PIN_SET)
      retval|= mask;
    else
      retval&=~mask;
    mask >>=1;
    DIO_Clock();
  }
  return retval;
}

void SetOutputs(uint8_t data)
{
  Device.Outputs = data;

  uint8_t mask = 0x80;
  /*** Write ***/
  for(uint8_t i=0; i<8; i++)
  {
    if(data & mask)
      HAL_GPIO_WritePin(PER_MOSI_GPIO_Port, PER_MOSI_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(PER_MOSI_GPIO_Port, PER_MOSI_Pin, GPIO_PIN_RESET);

    mask>>=1;
    DIO_Clock();
  }

  /*** Update ***/
//  HAL_GPIO_WritePin(PER_WR_GPIO_Port, PER_WR_Pin, GPIO_PIN_SET);
  DelayUs(1);
//  HAL_GPIO_WritePin(PER_WR_GPIO_Port, PER_WR_Pin, GPIO_PIN_RESET);
}

uint8_t GetOutputs(void)
{
  return Device.Outputs;
}

/* FreeRTOS ------------------------------------------------------------------*/
void configureTimerForRunTimeStats(void)
{
  //HAL_TIM_Base_Start_IT(&htim2);
}

unsigned long getRunTimeCounterValue(void)
{
  return RTOSRunTimeStatTick;
}

void UsbUartTx(char *str)
{
  static char temp[80];
  sprintf(temp, "%s\n",str);
  HAL_UART_Transmit(&huart1, (uint8_t*)temp, strlen(temp), 100);
}


/* Display--------------------------------------------------------------------*/
void SetDisplayOn()
{
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_SET);
}
void SetDisplayOff()
{
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_RESET);
}

uint8_t GetDisply(void)
{
  return HAL_GPIO_ReadPin(DISP_EN_GPIO_Port, DISP_EN_Pin) == GPIO_PIN_SET;
}


void UsbParser(char *request)
{
  char response[UART_BUFFER_SIZE];
  char cmd[20];
  char arg1[10];
  char arg2[10];
  uint8_t params = 0;
  if(strlen(USB_UART_RxBuffer) !=0)
  {
    params = sscanf(request, "%s %s %s", cmd, arg1, arg2);
    if(params == 1)
    {/*** parameterless commands ***/
      if(!strcmp(cmd, "*OPC?"))
      {
        strcpy(response, "*OPC");
      }
      else if(!strcmp(cmd, "*RDY?"))
      {
        strcpy(response, "*RDY");
      }
      else if(!strcmp(cmd, "*WHOIS?"))
      {
        strcpy(response, DEVICE_NAME);
      }
      else if(!strcmp(cmd, "*VER?"))
      {
        strcpy(response, DEVICE_FW);
      }
      else if(!strcmp(cmd, "*UID?"))
      {
        sprintf(response, "%4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
      }
      else if(!strcmp(cmd,"UPTIME?"))
      {
        sprintf(response, "%lld", Device.Diag.UpTimeSec);
      }
      else if(!strcmp(cmd, "DIS:LIG?"))
      {
        sprintf(response, "%d", DisplayLightGet());
      }
//        else if(!strcmp(cmd, "LED:LIG?"))
//        {
//          sprintf(response, "%d", GetPowerLedLight());
//        }
      else if(!strcmp(cmd, "DIS?"))
      {
        sprintf(response, "%d",GetDisply()?1:0);
      }
      else if(!strcmp(cmd, "DIS:ON"))
      {
        SetDisplayOn();
        strcpy(response, "RDY");
      }
      else if(!strcmp(cmd, "DIS:OFF"))
      {
        SetDisplayOff();
        strcpy(response, "RDY");
      }
//        else if(!strcmp(cmd, "PSP:ON"))
//        {
//          SetPowerSupply(1);
//          strcpy(response, "RDY");
//        }
//        else if(!strcmp(cmd, "PSP:OFF"))
//        {
//          SetPowerSupply(0);
//          strcpy(response, "RDY");
//        }
//        else if(!strcmp(cmd, "PSP?"))
//        {
//          sprintf(response, "%d",GetPowerSupply()?1:0);
//        }
      else if(!strcmp(cmd, "DIG:INP:U16?"))
      {
        sprintf(response, "%04X",GetInputs());
      }
      else if(!strcmp(cmd, "DIG:OUT:U8?"))
      {
        sprintf(response, "%02X",GetOutputs());
      }
//        else if(!strcmp(cmd, "TEM:ARR?"))
//        {
//          sprintf(response, "%0.3f; %0.3f; %0.3f; %0.3f",
//              Device.Temperature[0],
//              Device.Temperature[1],
//              Device.Temperature[2],
//              Device.Temperature[3]);
//        }
      else
      {
        strcpy(response, "!UNKNOWN");
      }
    }

    if(params == 2)
    {/*** commands with parameters ***/
      if(!strcmp(cmd, "DIS:LIG"))
      {
        DisplayLightSet(strtol(arg1, NULL, 0));
        strcpy(response, "RDY");
      }
//        else if(!strcmp(cmd, "LED:LIG"))
//        {
//          PowerLedSetMaxLight(strtol(arg1, NULL, 0));
//          strcpy(response, "RDY");
//        }
//        else if(!strcmp(cmd, "LED:PER"))
//        {
//          PowerLedSetUserPeriod(strtol(arg1, NULL, 0));
//          strcpy(response, "RDY");
//        }
//        else if(!strcmp(cmd, "LED:DIM"))
//        {
//          PowerLedSetDimming(strtol(arg1, NULL, 0));
//          strcpy(response, "RDY");
//        }
      else if(!strcmp(cmd, "DIG:OUT:SET:U8"))
      {
        uint8_t value = strtol(arg1, NULL, 16);
        SetOutputs(value);
        strcpy(response, "RDY");
      }
      else
      {
        strcpy(response, "!UNKNOWN");
      }
    }
    UsbUartTx(response);
  }
}

/* RS485----------------------------------------------------------------------*/
void RS485DirTx(void)
{
  HAL_GPIO_WritePin(RS_485_DIR_GPIO_Port, RS_485_DIR_Pin, GPIO_PIN_SET);
}

void RS485DirRx(void)
{
  HAL_GPIO_WritePin(RS_485_DIR_GPIO_Port, RS_485_DIR_Pin, GPIO_PIN_RESET);
}

void RS485UartTx(char *str)
{
  static char temp[80];

  RS485DirTx();
  DelayMs(1);
  sprintf(temp, "%s\n",str);
  HAL_UART_Transmit(&huart7, (uint8_t*)temp, strlen(temp), 100);
  RS485DirRx();
}

void RS485Parser(char *response)
{
#define RS485_CMD_SIZE  20
#define RS485_ARG1_SIZE 10
#define RS485_ARG2_SIZE 10

  char cmd[20];
  char arg1[10];
  char arg2[10];
  uint8_t params = 0;

  memset(cmd,0xCC, RS485_CMD_SIZE);
  memset(arg1,0xCC, RS485_ARG1_SIZE);
  memset(arg2,0xCC, RS485_ARG2_SIZE);

  if(strlen(response) !=0)
  {
    Device.Diag.RS485ResponseCnt++;
    params = sscanf(response, "%s %s %s", cmd, arg1, arg2);
    if(params == 1)
    {
      if(!strcmp(cmd, "RDY"))
      {
        Device.Diag.RS485RdyCnt++;
      }
      else
      {
        Device.Diag.RS485UnknownCnt++;
      }
    }

    if(params == 2)
    {
      if(!strcmp(cmd, "*OPC"))
      {

      }
      else if(!strcmp(cmd, "*WHOIS"))
      {

      }
      else if(!strcmp(cmd, "*VER"))
      {

      }
      else if(!strcmp(cmd, "*UID"))
      {

      }
      else if(!strcmp(cmd,"UPTIME"))
      {
         Device.Karuna.UpTimeSec = strtol(arg1, NULL, 0);
      }
      else if(!strcmp(cmd, "STATUS"))
      {
        Device.Karuna.Status = strtol(arg1, NULL, 16);
      }
      else if(!strcmp(cmd, "OUTS"))
      {
        Device.Karuna.Outputs = strtol(arg1, NULL, 16);
      }
      else
      {
        Device.Diag.RS485UnknownCnt++;
      }
    }
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  MX_TouchGFX_Process();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UsbRxTask */
/**
* @brief Function implementing the UsbRx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UsbRxTask */
void UsbRxTask(void *argument)
{
  /* USER CODE BEGIN UsbRxTask */
  /* Infinite loop */
  static uint32_t timestamp;
  static uint8_t startFlag;
  for(;;)
  {
    Device.Diag.UsbUartTaskCounter++;
    if(strlen(USB_UART_RxBuffer)!=0)
    {
      if(!startFlag)
      {
        timestamp = HAL_GetTick();
        startFlag = 1;
      }
      for(uint8_t i=0; i < UART_BUFFER_SIZE; i++)
      {
        if(USB_UART_RxBuffer[i]=='\n')
        {
          USB_UART_RxBuffer[i] = 0;
          startFlag = 0;
          HAL_UART_DMAStop(&huart1);
          UsbParser(USB_UART_RxBuffer);
          memset(USB_UART_RxBuffer, 0x00, UART_BUFFER_SIZE);
          HAL_UART_Receive_DMA(&huart1, (uint8_t*) USB_UART_RxBuffer, UART_BUFFER_SIZE);
          Device.Diag.UsbUartRxCommandsCounter ++;
        }
      }
      if(startFlag)
      {
        if(HAL_GetTick() - timestamp > 500)
        {
          if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
          {
            Device.Diag.UsbUartOverrunErrorCounter++;
            __HAL_UART_CLEAR_FLAG(&huart1,UART_CLEAR_OREF);
          }
          if(__HAL_UART_GET_FLAG(&huart1, USART_ISR_NE))
          {
            Device.Diag.UsbUartNoiseErrorCounter++;
            __HAL_UART_CLEAR_FLAG(&huart1,USART_ISR_NE);
          }
          if(__HAL_UART_GET_FLAG(&huart1, USART_ISR_FE))
          {
            Device.Diag.UsbUartFrameErrorCounter++;
            __HAL_UART_CLEAR_FLAG(&huart1,USART_ISR_FE);
          }
          startFlag = 0;
          HAL_UART_DMAStop(&huart1);
          memset(USB_UART_RxBuffer, 0x00, UART_BUFFER_SIZE);
          HAL_UART_Receive_DMA(&huart1, (uint8_t*) USB_UART_RxBuffer, UART_BUFFER_SIZE);
          Device.Diag.UsbUartTimeoutCounter ++;
        }
      }
    }
    osDelay(10);
  }
  /* USER CODE END UsbRxTask */
}

/* USER CODE BEGIN Header_LiveLedTask */
/**
* @brief Function implementing the LiveLed_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LiveLedTask */
void LiveLedTask(void *argument)
{
  /* USER CODE BEGIN LiveLedTask */
  /* Infinite loop */
  uint32_t timestamp = 0;
  uint8_t flag = 0;
  for(;;)
  {
    if(HAL_GetTick() - timestamp > 500)
    {
      timestamp = HAL_GetTick();
      if(flag)
      {
        flag = 0;
        LiveLedOn();
      }
      else
      {
        flag = 1;
        LiveLedOff();
        Device.Diag.UpTimeSec++;
      }
    }
  }
  /* USER CODE END LiveLedTask */
}

/* USER CODE BEGIN Header_RS485RxTask */
/**
* @brief Function implementing the RS485Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RS485RxTask */
void RS485RxTask(void *argument)
{
  /* USER CODE BEGIN RS485RxTask */
  /* Infinite loop */
  static uint32_t timestamp;
  static uint8_t startFlag;
  for(;;)
  {
    Device.Diag.BusUartTaskCounter++;
    if(strlen(RS485_UART_RxBuffer)!=0)
    {
      if(!startFlag)
      {
        timestamp = HAL_GetTick();
        startFlag = 1;
      }
      for(uint8_t i=0; i < UART_BUFFER_SIZE; i++)
      {
        if(RS485_UART_RxBuffer[i]=='\n')
        {
          RS485_UART_RxBuffer[i] = 0;
          startFlag = 0;
          HAL_UART_DMAStop(&huart7);
          RS485Parser(RS485_UART_RxBuffer);
          memset(RS485_UART_RxBuffer, 0x00, UART_BUFFER_SIZE);
          HAL_UART_Receive_DMA(&huart7, (uint8_t*) RS485_UART_RxBuffer, UART_BUFFER_SIZE);
          Device.Diag.BusUartRxCommandsCounter ++;
        }
      }
      if(startFlag)
      {
        if(HAL_GetTick() - timestamp > 500)
        {
          if(__HAL_UART_GET_FLAG(&huart7, UART_FLAG_ORE))
          {
            Device.Diag.BusUartOverrunErrorCounter++;
            __HAL_UART_CLEAR_FLAG(&huart7,UART_CLEAR_OREF);
          }
          if(__HAL_UART_GET_FLAG(&huart7, USART_ISR_NE))
          {
            Device.Diag.BusUartNoiseErrorCounter++;
            __HAL_UART_CLEAR_FLAG(&huart7,USART_ISR_NE);
          }
          if(__HAL_UART_GET_FLAG(&huart7, USART_ISR_FE))
          {
            Device.Diag.BusUartFrameErrorCounter++;
            __HAL_UART_CLEAR_FLAG(&huart7,USART_ISR_FE);
          }
          startFlag = 0;
          HAL_UART_DMAStop(&huart7);
          memset(RS485_UART_RxBuffer, 0x00, UART_BUFFER_SIZE);
          HAL_UART_Receive_DMA(&huart7, (uint8_t*) RS485_UART_RxBuffer, UART_BUFFER_SIZE);
          Device.Diag.BusUartTimeoutCounter ++;
        }
      }
    }
    osDelay(10);
  }
  /* USER CODE END RS485RxTask */
}

/* USER CODE BEGIN Header_RS485TxTask */
/**
* @brief Function implementing the RS485Tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RS485TxTask */
void RS485TxTask(void *argument)
{
  /* USER CODE BEGIN RS485TxTask */
  /* Infinite loop */
  uint8_t cmdId = 0;
  RS485UartTx("OUTS?");
  char temp[20];
  for(;;)
  {
    switch (cmdId)
    {
      case 0: RS485UartTx("UPTIME?");break;
      case 1: RS485UartTx("STATUS?");break;
      case 3:{
        sprintf(temp,"OUTS %02X", Device.Outputs);
        RS485UartTx(temp);
        break;
      };
    }

    if(cmdId == 3)
    {
      cmdId = 0;
    }
    else
    {
      cmdId++;
    }
    Device.Diag.RS485RequestCnt++;
    osDelay(100);
  }
  /* USER CODE END RS485TxTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
