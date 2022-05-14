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
#include "PowerLed.h"
#include "GuiItf.h"
#include "Peri.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct _DiagTypeDef
{
  uint32_t BusUartRxCommandsCounter;
  uint32_t UsbUartRxCommandsCounter;

  uint32_t UsbUartTimeoutCounter;
  uint32_t BusUartTimeoutCounter;

  uint32_t BusUartTxCommandsCounter;

  uint32_t UsbUartTaskCounter;
  uint32_t BusUartTaskCounter;

  uint32_t UsbUartOverrunErrorCounter;
  uint32_t RS485OverrunErrorCnt;

  uint32_t UsbUartNoiseErrorCounter;
  uint32_t RS485NoiseErrorCnt;

  uint32_t UsbUartFrameErrorCounter;
  uint32_t RS485FrameErrorCnt;

  uint32_t RS485RequestCnt;
  uint32_t RS485UnknownCnt;
  uint32_t RS485ResponseCnt;
  uint32_t RS485RdyCnt;


  uint32_t PowerLedTaskCounter;
  uint64_t UpTimeSec;


}Diag_t;



typedef struct _AppTypeDef
{
  Diag_t Diag;

  struct
  {
    uint8_t Status;
    uint8_t Outputs;
    uint32_t UpTimeSec;
  }Karuna;

  struct
  {
    double AnalogInputs[4];
    double Temperatures[4];
    uint8_t Outputs;
    uint16_t Inputs;
  }Peri;

}Device_t;

extern Device_t Device;
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
#define RS_485_DIR_Pin GPIO_PIN_8
#define RS_485_DIR_GPIO_Port GPIOF
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
#define DIO_WR_Pin GPIO_PIN_12
#define DIO_WR_GPIO_Port GPIOB
#define SPI_CLK_Pin GPIO_PIN_13
#define SPI_CLK_GPIO_Port GPIOB
#define PER_MISO_Pin GPIO_PIN_14
#define PER_MISO_GPIO_Port GPIOB
#define PER_MOSI_Pin GPIO_PIN_15
#define PER_MOSI_GPIO_Port GPIOB
#define PSP_EN_Pin GPIO_PIN_2
#define PSP_EN_GPIO_Port GPIOG
#define DIO_CS_Pin GPIO_PIN_3
#define DIO_CS_GPIO_Port GPIOG
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
