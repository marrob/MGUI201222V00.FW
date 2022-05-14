/*
 * GuiItf.h
 *
 *  Created on: 2022. máj. 14.
 *      Author: Margit Robert
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _GUIITF_H_
#define _GUIITF_H_


/* Exported functions prototypes ---------------------------------------------*/
uint8_t GuiItfGetKarunaStatus(void);
void GuiItfKarunaControl(uint8_t p_Output);
uint32_t GuiItfGetKarunaUptimeCnt(void);



#endif /* _GUIITF_H_ */
