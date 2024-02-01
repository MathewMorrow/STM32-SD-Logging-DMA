/*
 * time.c
 *
 *  Created on: Feb 6, 2023
 *      Author: Mathew
 */


#include "time.h"
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef 	*microsTimer;


void microsInit(TIM_HandleTypeDef *_microsTimer)
{
	microsTimer = _microsTimer;
	HAL_TIM_Base_Start(microsTimer);
}

uint32_t getMircos()
{
	return microsTimer->Instance->CNT;

}

void delayMicros(uint32_t micros)
{
	uint32_t startMicros = microsTimer->Instance->CNT;
	while( (microsTimer->Instance->CNT) - startMicros < micros){}
//	HAL_Delay(micros);
}
