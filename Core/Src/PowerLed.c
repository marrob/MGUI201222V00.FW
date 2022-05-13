/*
 * PowerLed.c
 *
 *  Created on: Jan 20, 2022
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PowerLed_t _powerLed;
static TIM_HandleTypeDef *_htim;
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN Header_PowerLedTask */
/**
* @brief Function implementing the PowerLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PowerLedTask */
void PowerLedTask(void *argument)
{
  /* USER CODE BEGIN PowerLedTask */
  /* Infinite loop */
  for(;;)
  {
    Device.Diag.PowerLedTaskCounter++;

    switch(_powerLed.State)
    {
      case PWR_LED_ON:
      {
        PowerSetLedLight(_powerLed.LightMaxPercent);
        _powerLed.State = PWR_LED_COMPLETE;
        break;
      }
      case PWR_LED_OFF:
      {
        PowerSetLedLight(0);
        _powerLed.State = PWR_LED_COMPLETE;
        break;
      }
      case PWR_LED_WAIT_FOR:
      {
        //PowerSetLedLight(0);
        for(int i = 0; i<_powerLed.LightMaxPercent; i++)
        {
          PowerSetLedLight(i);
          vTaskDelay(10/portTICK_PERIOD_MS);
        }
        osDelay(250);
        for(int i = _powerLed.LightMaxPercent; i; i--)
        {
          PowerSetLedLight(i);
          vTaskDelay(10/portTICK_PERIOD_MS);
        }
        //PowerSetLedLight(0);
        vTaskDelay(250/portTICK_PERIOD_MS);
        break;
      }

      case PWR_LED_SHOW_CODE:
      {
        PowerSetLedLight(0);

        for(uint8_t j=0; j < 3; j++)
        {
          vTaskDelay(500/portTICK_PERIOD_MS);
          for(uint8_t i=0; i < _powerLed.Code ; i++ )
          {
            PowerSetLedLight(_powerLed.LightMaxPercent);
            vTaskDelay(250/portTICK_PERIOD_MS);
            PowerSetLedLight(0);
            vTaskDelay(200/portTICK_PERIOD_MS);
            //DeviceDbgLog("i:%d, j:%d", i, j);
          }
        }
        _powerLed.State = PWR_LED_COMPLETE;
        break;
      }

      case PWR_LED_USER_FALSH:
      {
        if(_powerLed.Dimming)
        {
          //PowerSetLedLight(0);
          for(int i = 0; i<_powerLed.LightMaxPercent; i++)
          {
            PowerSetLedLight(i);
            vTaskDelay(10/portTICK_PERIOD_MS);
          }
          //vTaskDelay(_powerLed.UserPeriodTimeMs/portTICK_PERIOD_MS);
          for(int i = _powerLed.LightMaxPercent; i; i--)
          {
            PowerSetLedLight(i);
            vTaskDelay(10/portTICK_PERIOD_MS);
          }
          //PowerSetLedLight(0);
          vTaskDelay(_powerLed.UserPeriodTimeMs/portTICK_PERIOD_MS);
        }
        else
        {
          PowerSetLedLight(_powerLed.LightMaxPercent);
          vTaskDelay(_powerLed.UserPeriodTimeMs/portTICK_PERIOD_MS);
          PowerSetLedLight(0);
          vTaskDelay(_powerLed.UserPeriodTimeMs/portTICK_PERIOD_MS);
        }
        break;
      }
      case PWR_LED_COMPLETE:
      {
        break;
      }
    }

  }
  /* USER CODE END PowerLedTask */
}

void PowerLedInit(TIM_HandleTypeDef *htim)
{
  _htim = htim;
  HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_1);
  PowerLedSetState(PWR_LED_OFF);
  PowerLedSetMaxLight(80);
  PowerSetLedLight(0);
}


void PowerSetLedLight(uint8_t percent)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(_htim);
  uint32_t ccr = (arr * percent) / 100;
  __HAL_TIM_SET_COMPARE(_htim,TIM_CHANNEL_1, ccr);
}

uint8_t GetPowerLedLight(void)
{
  return _powerLed.LightMaxPercent;
}

uint8_t PowerLedShowCode(uint8_t code)
{
  _powerLed.State = PWR_LED_SHOW_CODE;
  _powerLed.Code = code;
  return PWRLED_OK;
}

uint8_t PowerLedIsReady()
{
  return  _powerLed.State == PWR_LED_COMPLETE;
}

uint8_t PowerLedSetState(uint8_t state)
{
  _powerLed.State = state;
  return PWRLED_OK;
}

uint8_t PowerLedSetUserPeriod(uint32_t period)
{
  _powerLed.UserPeriodTimeMs = period;
  return PWRLED_OK;
}

uint8_t PowerLedSetDimming(uint8_t onfoff)
{
  _powerLed.Dimming = onfoff;
  return PWRLED_OK;
}

uint8_t PowerLedSetMaxLight(uint8_t percent)
{
  _powerLed.LightMaxPercent = percent;
  return PWRLED_OK;

}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
