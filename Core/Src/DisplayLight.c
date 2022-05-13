/*
 * DisplayLight.c
 *
 *  Created on: Jan 26, 2022
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "DisplayLight.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DisplayLight_t _displayLight;
static TIM_HandleTypeDef *_htim;
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void DisplayLightInit(TIM_HandleTypeDef *htim)
{
  _htim = htim;
  HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_4);
  DisplayLightSet(10);
}

uint8_t DisplayLightSet(uint8_t percent)
{
  _displayLight.LightPercent = percent;
  percent = 100 - percent;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(_htim);
  uint32_t ccr = (arr * percent) / 100;
  __HAL_TIM_SET_COMPARE(_htim,TIM_CHANNEL_4, ccr);

  return DISPLAY_LIGHT_OK;
}

uint8_t DisplayLightGet(void)
{
  return  _displayLight.LightPercent;
}


void DisplayEnable(){
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_SET);
}
void DisplayDisable(){
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_RESET);
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
