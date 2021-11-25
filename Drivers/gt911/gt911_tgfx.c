/* Includes ------------------------------------------------------------------*/
#include "gt911_tgfx.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "vt100.h"

/* Private typedef -----------------------------------------------------------*/
TS_DrvTypeDef GT911_ts_drv =
{
  GT911_Init,
  GT911_DetectTouch,
  GT911_GetXY,
};


static uint8_t GT911_ReadRegister(I2C_HandleTypeDef *i2c, uint16_t reg, uint8_t *value, size_t length);
static uint8_t GT911_WriteRegister(I2C_HandleTypeDef *i2c, uint16_t reg, uint8_t *value, size_t length);


void GT911_Init(I2C_HandleTypeDef *i2c)
{
    printf(VT100_ATTR_RESET);

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
    GT911_ReadRegister(i2c, GT911_REG_ID, id_buffer, sizeof(id_buffer));
    printf("Touch:ID:%c%c%c\r\n",id_buffer[0],id_buffer[1],id_buffer[2]);

    uint8_t configVer =0;
    GT911_ReadRegister(i2c, GT911_REG_CFG_ADDR, &configVer, sizeof(configVer));
    printf("Touch:Config_Version:%2x\r\n",configVer);

    uint16_t fwVer=0;
    GT911_ReadRegister(i2c, GT911_REG_FW_VER, (uint8_t*)&fwVer, sizeof(fwVer));
    printf("Touch:FirmwareVersion:%2x\r\n",fwVer);


}

GT911_TouchCoordinateTypeDef Touches[5];
uint32_t detectCounter = 0;
uint32_t touchZero = 0;
void GT911_GetXY(uint16_t *X, uint16_t *Y)
{
    *Y = Touches[0].X;
    *X = Touches[0].Y;

}

uint8_t GT911_DetectTouch(I2C_HandleTypeDef *i2c)
{
    uint8_t touchCount = 0;
    uint8_t clrarbyte = 0;
    uint8_t ts_status = 0;

    GT911_ReadRegister(i2c, GT911_READ_COORD_ADDR, &ts_status, sizeof(ts_status));
    if((ts_status & 0x80) == 0)
    {
        touchCount = 0;
        GT911_WriteRegister(i2c, GT911_READ_COORD_ADDR, &clrarbyte, sizeof(clrarbyte));
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
          GT911_ReadRegister(i2c, GT911_POINT1X_COORD_LOW + touchIndex * 8 ,  (uint8_t*) &Touches[touchIndex], sizeof(GT911_TouchCoordinateTypeDef));
        }
        GT911_WriteRegister(i2c, GT911_READ_COORD_ADDR, &clrarbyte, sizeof(clrarbyte));
    }
    //HAL_Delay(10);
    return(touchCount);
}

uint8_t GT911_WriteRegister(I2C_HandleTypeDef *i2c, uint16_t reg, uint8_t *value, size_t length)
{
    uint8_t status = 0;
    status = HAL_I2C_Mem_Write(i2c, GT911_I2C_DEV_ADDR, reg, sizeof(reg), value, length, GT911_I2C_TIMEOUT_MS);
    if(status != HAL_OK )
    {
        printf(VT100_ATTR_RED);
        printf("HAL_I2C_Mem_Write %x\r\n", status);
        printf(VT100_ATTR_RESET);
    }
    return status;
}

uint8_t GT911_ReadRegister(I2C_HandleTypeDef *i2c, uint16_t reg, uint8_t *value, size_t length)
{
    uint8_t status = 0;
    HAL_I2C_Mem_Read(i2c, GT911_I2C_DEV_ADDR, reg, sizeof(reg), value, length, GT911_I2C_TIMEOUT_MS);
    if(status != HAL_OK )
    {
        printf(VT100_ATTR_RED);
        printf("HAL_I2C_Mem_Read %x\r\n", status);
        printf(VT100_ATTR_RESET);
    }
    return status;
}

