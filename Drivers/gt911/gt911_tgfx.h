/**
 ******************************************************************************
 * @file    ft6x06.h
 * @author  MCD Application Team
  * @version V1.0.0
  * @date    03-August-2015
 * @brief   This file contains all the functions prototypes for the
 *          ft6x06.c IO expander driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GT811_H
#define __GT811_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "ts.h"

/* Macros --------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define GT911_I2C_DEV_ADDR       0xBA
#define GT911_I2C_TIMEOUT_MS     100
#define GT911_RT_COMMAND          0x8040
#define GT911_REG_ID              0x8140
#define GT911_REG_CFG_ADDR        0x8047
#define GT911_READ_COORD_ADDR     0x814E
#define GT911_STATUS_MODE_ADDR    0x81A8
#define GT911_REG_FW_VER          0x8144
#define GT911_POINT1X_COORD_LOW   0x8150

typedef struct
{
  uint8_t i2cInitialized;
  uint8_t currActiveTouchNb;
  uint8_t currActiveTouchIdx;
  uint8_t touchCount;
} gt911_handle_TypeDef;

typedef struct
{
  uint16_t X;
	uint16_t Y;
	uint16_t Point;
  uint8_t NC;
	uint8_t TrackId;
} GT911_TouchCoordinateTypeDef;


void GT911_Init(I2C_HandleTypeDef *i2c);
void GT911_GetXY(uint16_t *X, uint16_t *Y);
uint8_t GT911_DetectTouch(I2C_HandleTypeDef *i2c);


extern TS_DrvTypeDef GT911_ts_drv;

#ifdef __cplusplus
}
#endif
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
