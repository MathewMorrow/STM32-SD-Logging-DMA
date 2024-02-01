/*
 * cpu_load.c
 *
 *  Created on: Dec 25, 2023
 *      Author: Mathew
 */


#include "cpu_load.h"
#include "stm32f4xx_hal.h"
#include "time.h"

#define CPU_LOAD_NUM_SAMPLES 20
#define UINT32_T_MAX 4294967295

static uint8_t cpuPercentLoad = 0;
static uint32_t cpuLoadStartMicros = 0;
static uint32_t cpuLoadMicrosSum = 0;
static uint32_t cpuLoadMicrosSumPrevious = 0;
static uint32_t cpuLoadMaxLoopMicros = 0;
static uint32_t cpuLoadMinLoopMicros = UINT32_T_MAX;
static const uint32_t cpuLoadPeriodMicros = 100000;


/* Initialize states of CPU processes */
void cpuLoad_Init(cpuLoad_t *cpuLoad)
{
	/* Time variables */
	cpuLoad->functionStartMicros = 0;
	cpuLoad->functionEndMicros = 0;
	cpuLoad->totalFunctionMicros = 0;
	/* Statistics */
	cpuLoad->maxMicros = 0;
	cpuLoad->minMicros = UINT32_T_MAX;
	cpuLoad->avgMicros = 0;
	cpuLoad->numSamples = 0;
}

/* Start the calculate of CPU load times */
void cpuLoad_Start(cpuLoad_t *cpuLoad)
{
	cpuLoad->functionStartMicros = getMircos();
}

/* Stop/Halt the calculate of CPU load times
 * Retrieve the time and calculate statistics
 * */
void cpuLoad_End(cpuLoad_t *cpuLoad)
{
	cpuLoad->functionEndMicros = getMircos();
	cpuLoad->totalFunctionMicros = cpuLoad->functionEndMicros - cpuLoad->functionStartMicros;

	cpuLoadMicrosSum += cpuLoad->totalFunctionMicros;

	if(cpuLoad->totalFunctionMicros > cpuLoad->maxMicros) cpuLoad->maxMicros = cpuLoad->totalFunctionMicros;
	else if(cpuLoad->totalFunctionMicros < cpuLoad->minMicros) cpuLoad->minMicros = cpuLoad->totalFunctionMicros;

	cpuLoad->numSamples++;
	cpuLoad->avgMicrosAccum += cpuLoad->totalFunctionMicros;
	if(cpuLoad->numSamples == CPU_LOAD_NUM_SAMPLES)
	{
		cpuLoad->numSamples = 0;
		cpuLoad->avgMicros = cpuLoad->avgMicrosAccum / CPU_LOAD_NUM_SAMPLES;
	}

}

static uint32_t dummyvalue = 0;
/* Must be run every main loop
 * House keeping and checks when a new CPU utilization should be calculated
 * */
void cpuLoad_Process()
{
	uint32_t loopMicros = cpuLoadMicrosSum - cpuLoadMicrosSumPrevious;
	cpuLoadMicrosSumPrevious = cpuLoadMicrosSum;

	if(loopMicros > cpuLoadMaxLoopMicros) cpuLoadMaxLoopMicros = loopMicros;
	else if(loopMicros < cpuLoadMinLoopMicros) cpuLoadMinLoopMicros = loopMicros;

	uint32_t microsNow = getMircos();
	uint32_t microsDelta = microsNow - cpuLoadStartMicros;
	dummyvalue = microsDelta;
	if(getMircos() - cpuLoadStartMicros >= cpuLoadPeriodMicros)
	{
		cpuLoadStartMicros = getMircos();
		cpuPercentLoad = (100*cpuLoadMicrosSum)/cpuLoadPeriodMicros;
		cpuLoadMicrosSum = 0;
		cpuLoadMicrosSumPrevious = 0;
	}
}

/* Reset statistic for CPU utilization */
void cpuLoad_Reset()
{
	cpuLoadMicrosSum = 0;
	cpuLoadMicrosSumPrevious = 0;
	cpuLoadMaxLoopMicros = 0;
	cpuLoadMinLoopMicros = UINT32_T_MAX;

	cpuLoadStartMicros = getMircos();
}

uint8_t cpuLoad_GetPercentLoad()
{
	return cpuPercentLoad;
}

uint32_t cpuLoad_GetMaxLoopTime()
{
	return cpuLoadMaxLoopMicros;
}

uint32_t cpuLoad_GetMinLoopTime()
{
	return cpuLoadMinLoopMicros;
}









