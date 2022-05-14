/*
 * GuiItf.h
 *
 *  Created on: 2022. máj. 14.
 *      Author: Margit Robert
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ANALOG_INPUTS_H_
#define _ANALOG_INPUTS_H_

/*** MCP3208 ***/
#define MCP320X_CH0          0
#define MCP320X_CH1          1
#define MCP320X_CH2          2
#define MCP320X_CH3          3
#define MCP320X_CON_SINGLE_END  (1<<3)


typedef struct _Peri_t
{
  double Temperature[4];
  uint8_t Outputs;
  uint16_t Inputs;
}Peri_t;


/* Exported functions prototypes ---------------------------------------------*/
void PeriInit(void);
double PeriGetTemperature(uint8_t channel);
uint16_t PeriGetInputs(void);
void PeriSetOutputs(uint8_t data);
uint8_t PeriGetOutputs(void);

#endif /* _ANALOG_INPUTS_H_ */
