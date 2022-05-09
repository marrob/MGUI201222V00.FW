/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vt100.h"
#include "common.h"
#include "DisplayLight.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct _DiagTypeDef
{
  uint32_t HostUartRxCommandsCounter;
  uint32_t HostUartTimeoutCounter;
  uint32_t HostUartTxCommandsCounter;
  uint32_t UartTaskCounter;
  uint32_t PowerOnOffTaskCounter;
  uint32_t PowerLedTaskCounter;
  uint64_t UpTimeSec;
  uint32_t HostUartOverrunErrorCounter;
  uint32_t HostUartNoiseErrorCounter;
  uint32_t HostUartFrameErrorCounter;
}Diag_t;

typedef struct _AppTypeDef
{
  double Temperature[4];
  uint8_t Outputs;
  uint16_t Inputs;
  Diag_t Diag;
}Device_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define DEVICE_NAME             "MGUI201222VA1-KARUNA"
#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_FW               "1.0.0.1"
#define DEVICE_FW_SIZE          sizeof(DEVICE_FW)
#define DEVICE_PCB              "00"
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_MNF              "KONVOLUCIO"
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)




/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern void init_app_cpp_domain(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DO_EN_Pin GPIO_PIN_8
#define DO_EN_GPIO_Port GPIOI
#define AI_CS_Pin GPIO_PIN_0
#define AI_CS_GPIO_Port GPIOA
#define AI_MOSI_Pin GPIO_PIN_1
#define AI_MOSI_GPIO_Port GPIOA
#define AI_MISO_Pin GPIO_PIN_6
#define AI_MISO_GPIO_Port GPIOA
#define LIVE_LED_Pin GPIO_PIN_4
#define LIVE_LED_GPIO_Port GPIOC
#define TS_RST_Pin GPIO_PIN_11
#define TS_RST_GPIO_Port GPIOH
#define TS_INT_Pin GPIO_PIN_12
#define TS_INT_GPIO_Port GPIOH
#define SPI_CLK_Pin GPIO_PIN_13
#define SPI_CLK_GPIO_Port GPIOB
#define PER_MISO_Pin GPIO_PIN_14
#define PER_MISO_GPIO_Port GPIOB
#define PER_MOSI_Pin GPIO_PIN_15
#define PER_MOSI_GPIO_Port GPIOB
#define PSP_EN_Pin GPIO_PIN_2
#define PSP_EN_GPIO_Port GPIOG
#define PER_CLR_Pin GPIO_PIN_3
#define PER_CLR_GPIO_Port GPIOG
#define PER_LD_Pin GPIO_PIN_6
#define PER_LD_GPIO_Port GPIOC
#define DISP_EN_Pin GPIO_PIN_7
#define DISP_EN_GPIO_Port GPIOC
#define BTN_PWM_Pin GPIO_PIN_8
#define BTN_PWM_GPIO_Port GPIOA
#define DISP_PWM_Pin GPIO_PIN_11
#define DISP_PWM_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
  void ConsoleWrite(char *str);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
