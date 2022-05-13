/*
 * DisplayLight.h
 *
 *  Created on: Jan 26, 2022
 *      Author: Margit Robert
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APPLICATION_USER_CORE_INC_DISPLAYLIGHT_H_
#define APPLICATION_USER_CORE_INC_DISPLAYLIGHT_H_
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

typedef struct _DisplayLight
{
  uint8_t LightPercent;
} DisplayLight_t;

/* Exported macro ------------------------------------------------------------*/
#define DISPLAY_LIGHT_OK  0
/* Exported functions ------------------------------------------------------- */

void DisplayLightInit(TIM_HandleTypeDef *htim);
uint8_t DisplayLightSet(uint8_t percent);
uint8_t DisplayLightGet(void);
void DisplayEnable(void);
void DisplayDisable(void);


#endif /* APPLICATION_USER_CORE_INC_DISPLAYLIGHT_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
