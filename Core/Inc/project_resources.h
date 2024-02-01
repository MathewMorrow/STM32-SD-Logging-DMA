/*
 * project_Resources.h
 *
 *  Created on: Dec 26, 2023
 *      Author: Mathew
 */

#ifndef INC_PROJECT_RESOURCES_H_
#define INC_PROJECT_RESOURCES_H_

#include "VCP.h"
#include "blackbox.h"
#include "time.h"
#include "cpu_load.h"

#define microsTimer htim2

extern cpuLoad_t cpuLoadDelay;
extern cpuLoad_t cpuLoadBlackbox;
extern cpuLoad_t cpuLoadUSB;
extern cpuLoad_t cpuLoadCLI;

extern uint32_t delayMicroseconds;


extern uint32_t lastMillis;
extern uint32_t stopBlink;
extern uint16_t writePeriod;

/* SD card */
extern uint32_t loggingFileSize;
extern uint8_t doSDBenchmark;
extern uint32_t streamSDIO;

extern volatile uint8_t simNewDataFlag;

extern float gyroPitchRaw;
extern float gyroPitchFilt;
extern float gyroRollRaw;
extern float gyroRollFilt;
extern float gyroYawRaw;
extern float gyroYawFilt;
extern float pitchP;
extern float pitchI;
extern float pitchD;

extern uint8_t blackboxFrame[64];


#endif /* INC_PROJECT_RESOURCES_H_ */
