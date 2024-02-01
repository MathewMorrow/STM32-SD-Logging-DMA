/*
 * project_resources.c
 *
 *  Created on: Dec 26, 2023
 *      Author: Mathew
 */

#include "project_resources.h"
#include "VCP.h"
#include "blackbox.h"
#include "time.h"
#include "cpu_load.h"

cpuLoad_t cpuLoadDelay;
cpuLoad_t cpuLoadBlackbox;
cpuLoad_t cpuLoadUSB;
cpuLoad_t cpuLoadCLI;

uint32_t delayMicroseconds = 100;


uint32_t lastMillis = 0;
uint32_t stopBlink = 0;
uint16_t writePeriod = 10;

/* SD card */
uint32_t loggingFileSize = MEGABYTE*50;
uint8_t doSDBenchmark = 0;
uint32_t streamSDIO = 0;

volatile uint8_t simNewDataFlag = 0;


float gyroPitchRaw = 123.4;
float gyroPitchFilt = 122.4;
float gyroRollRaw = 234.4;
float gyroRollFilt = 232.4;
float gyroYawRaw = 456.7;
float gyroYawFilt = 454.7;
float pitchP = 0.1234;
float pitchI = 0.02134;
float pitchD = 0.2345;

uint8_t blackboxFrame[64] = {0};

