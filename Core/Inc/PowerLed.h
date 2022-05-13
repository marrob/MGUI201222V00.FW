/*
 * PowerLed.h
 *
 *  Created on: Jan 26, 2022
 *      Author: Margit Robert
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APPLICATION_USER_CORE_POWERLED_H_
#define APPLICATION_USER_CORE_POWERLED_H_
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum _PowerLed_t
{
  PWR_LED_OFF,          //0
  PWR_LED_ON,           //1
  PWR_LED_WAIT_FOR,     //2
  PWR_LED_SHOW_CODE,    //3
  PWR_LED_USER_FALSH,   //4
  PWR_LED_COMPLETE      //5

}PwrLedState_t;

typedef struct _PowerLed
{
  uint8_t LightMaxPercent;
  PwrLedState_t State;
  uint8_t Code;
  uint32_t UserPeriodTimeMs;
  uint8_t Dimming;
}PowerLed_t;

/* Exported constants --------------------------------------------------------*/
#define PWRLED_OK   0
#define PWRLED_FAIL 1

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void PowerLedInit(TIM_HandleTypeDef *htim);
void PowerSetLedLight(uint8_t percent);
uint8_t PowerLedShowCode(uint8_t code);
uint8_t PowerLedIsReady(void);
uint8_t PowerLedSetState(uint8_t state);
uint8_t PowerLedSetMaxLight(uint8_t percent);
uint8_t PowerLedSetDimming(uint8_t onfoff);

#endif /* APPLICATION_USER_CORE_POWERLED_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
