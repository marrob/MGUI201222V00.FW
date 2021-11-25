/**
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gt911_tgfx.h"
#include <stdio.h>
#include <string.h>
#include "main.h"

/* Private typedef -----------------------------------------------------------*/


TS_DrvTypeDef gt911_ts_drv =
{
  GT911_Init,
  gt911_TS_DetectTouch,
  gt911_TS_GetXY,
};


uint8_t GT911_DetectTouch(I2C_HandleTypeDef *i2c, uint8_t *isDetect, uint8_t *currActiveTouchIdx);
uint8_t GT911_PollTouch(uint16_t i2cAddress, uint8_t *isDetect, GT911_TouchCoordinateTypeDef touches[]);

void GT911_Init(uint16_t i2cAddress)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  //INT pin dir OUT & set LOW
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = TS_INT_Pin;
  HAL_GPIO_Init(TS_INT_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(TS_INT_GPIO_Port, TS_INT_Pin, GPIO_PIN_RESET);

  //RST pin set HIGH
  HAL_GPIO_WritePin(TS_RST_GPIO_Port, TS_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(TS_RST_GPIO_Port, TS_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(TS_RST_GPIO_Port, TS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(200);

  //INT pin dir INPUT & set PULLUP & interrupt
  GPIO_InitStruct.Pin = TS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TS_INT_GPIO_Port, &GPIO_InitStruct);

  uint8_t id_buffer[3] = {0,0,0};
  TS_IO_Read(i2cAddress, GT911_REG_ID, id_buffer, sizeof(id_buffer));
  printf("TouchPad_ID:%c%c%c\r\n",id_buffer[0],id_buffer[1],id_buffer[2]);

  uint8_t configVer =0;
  TS_IO_Read(i2cAddress, GT911_REG_CFG_ADDR, &configVer, sizeof(configVer));
  printf("TouchPad_Config_Version:%2x\r\n",configVer);

  uint16_t fwVer=0;
  TS_IO_Read(i2cAddress, GT911_REG_FW_VER, &fwVer, sizeof(fwVer));
  printf("FirmwareVersion:%2x\r\n",fwVer);


}

GT911_TouchCoordinateTypeDef Touches[5];
uint32_t detectCounter = 0;
uint32_t touchZero = 0;
void gt911_TS_GetXY(uint16_t i2cAddress, uint16_t *X, uint16_t *Y)
{
  *Y = Touches[0].X;
  *X = Touches[0].Y;

}

uint8_t gt911_TS_DetectTouch(uint16_t i2cAddress)
{
  uint8_t touchCount = 0;
  uint8_t clrarbyte = 0;
  uint8_t ts_status = 0;

  TS_IO_Read(i2cAddress, GT911_READ_COORD_ADDR, &ts_status, sizeof(ts_status));
  if((ts_status & 0x80) == 0)
  {
    touchCount = 0;
    TS_IO_Write(i2cAddress, GT911_READ_COORD_ADDR, &clrarbyte, sizeof(clrarbyte));
  }
  else
  {
    detectCounter++;
    touchCount = ts_status & 0x0F;
    if(touchCount == 0)
    {
      touchZero ++;
    }
    if(touchCount > 0)
    {
      uint8_t touchIndex = touchCount - 1;
      TS_IO_Read(i2cAddress, GT911_POINT1X_COORD_LOW + touchIndex * 8 ,  (uint8_t*) &Touches[touchIndex], sizeof(GT911_TouchCoordinateTypeDef));
    }
    TS_IO_Write(i2cAddress, GT911_READ_COORD_ADDR, &clrarbyte, sizeof(clrarbyte));
  }
  //HAL_Delay(10);

  return(touchCount);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
