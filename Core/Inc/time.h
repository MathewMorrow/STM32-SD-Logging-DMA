/*
 * time.h
 *
 *  Created on: Feb 6, 2023
 *      Author: Mathew
 */

#ifndef INC_TIME_H_
#define INC_TIME_H_

#include "stm32f4xx_hal.h"


void microsInit(TIM_HandleTypeDef *_microsTimer);

uint32_t getMircos();

void delayMicros(uint32_t micros);

#endif /* INC_TIME_H_ */
