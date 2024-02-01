/*
 * cpu_load.h
 *
 *  Created on: Dec 25, 2023
 *      Author: Mathew
 */

#ifndef INC_CPU_LOAD_H_
#define INC_CPU_LOAD_H_

#include "stm32f4xx_hal.h"

/* Structure to hold load times
 * Create for every process to be added to CPU utilization
 * */
typedef struct{

	/* Time variables */
	uint32_t functionStartMicros;
	uint32_t functionEndMicros;
	uint32_t totalFunctionMicros;

	/* Statistics */
	uint8_t isActive;
	uint32_t maxMicros;
	uint32_t minMicros;
	uint32_t avgMicros;
	uint32_t avgMicrosAccum;
	uint8_t numSamples;

} cpuLoad_t;

/* Initialize states of CPU processes */
void cpuLoad_Init(cpuLoad_t *cpuLoad);

/* Start the calculate of CPU load times
 * Call at the beginning of the processes function
 * Call cpuLoad_Stop(cpuLoad_t *cpuLoad) at the end
 * */
void cpuLoad_Start(cpuLoad_t *cpuLoad);

/* Stop/Halt the calculate of CPU load times
 * Retrieve the time and calculate statistics
 * */
void cpuLoad_End(cpuLoad_t *cpuLoad);

/* Must be run every main loop
 * House keeping and checks when a new CPU utilization should be calculated
 * */
void cpuLoad_Process();

/* Reset statistic for CPU utilization */
void cpuLoad_Reset();

/* Returns the CPU load/utilization from 0-100 */
uint8_t cpuLoad_GetPercentLoad();

/* Returns the maximum calculated loop time in microseconds */
uint32_t cpuLoad_GetMaxLoopTime();

/* Returns the minimum calculated loop time in microseconds */
uint32_t cpuLoad_GetMinLoopTime();



#endif /* INC_CPU_LOAD_H_ */
