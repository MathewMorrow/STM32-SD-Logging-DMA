/*
 * console.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Mathew
 */

#ifndef INC_CONSOLE_H_
#define INC_CONSOLE_H_

#include "stm32f4xx_hal.h"

void printCLI();
void streamData();

extern uint8_t streamDataSet;
extern uint8_t streamBuffRem;
extern uint8_t sprintfBuffer[512];

void printCLI(void);

void streamData();

#endif /* INC_CONSOLE_H_ */
