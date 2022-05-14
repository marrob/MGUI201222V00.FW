/*
 * GuiItf.c
 *
 *  Created on: 2022. máj. 14.
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "GuiItf.h"

/* Private user code ---------------------------------------------------------*/
uint8_t GuiItfGetKarunaStatus(void)
{
	return Device.Karuna.Status;
}

void GuiItfKarunaControl(uint8_t p_Output)
{
  Device.Karuna.Outputs = p_Output;
}

uint32_t GuiItfGetKarunaUptimeCnt(void)
{
  return Device.Karuna.UpTimeSec;
}
