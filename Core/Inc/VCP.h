/*
 * VCP.h
 *
 *  Created on: Nov 12, 2022
 *      Author: Mathew
 */

#ifndef INC_VCP_H_
#define INC_VCP_H_

#include "stm32f4xx_hal.h"

#define VCP_BUFFER_LENGTH 64

extern uint8_t vcpBuffer[VCP_BUFFER_LENGTH];



uint8_t vcp_peek(void);

uint8_t vcp_Read();

void vcp_RxDataHandler(uint8_t* _Buf, uint32_t *_Len);

void vcp_DefaultFramingErrorHandler();

void vcp_DefaultOverrunErrorHandler();

//void vcp_DefaultErrorHandler(){};

__weak void vcp_TransmitCplt();

uint8_t vcp_getRxCount();

#endif /* INC_VCP_H_ */
