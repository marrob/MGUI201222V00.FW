/*
 * GuiItf.h
 *
 *  Created on: 2022. máj. 14.
 *      Author: Margit Robert
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ANALOG_INPUTS_H_
#define _ANALOG_INPUTS_H_

#define AI_CH0    0
#define AI_CH1    1
#define AI_CH2    2
#define AI_CH3    3


/*** MCP3208 ***/
#define MCP320X_CH0          0
#define MCP320X_CH1          1
#define MCP320X_CH2          2
#define MCP320X_CH3          3
#define MCP320X_CON_SINGLE_END  (1<<3)


/* Exported functions prototypes ---------------------------------------------*/
void PeriInit(void);
double PeriGetTemperature(uint8_t channel);
uint16_t PeriGetInputs(void);
void PeriSetOutputs(uint8_t data);

#endif /* _ANALOG_INPUTS_H_ */
