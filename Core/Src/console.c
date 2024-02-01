/*
 * console.c
 *
 *  Created on: Dec 27, 2023
 *      Author: Mathew
 */


#include "VCP.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "blackbox.h"
#include "time.h"
#include "cpu_load.h"
#include "project_resources.h"


///* USB VCP CLI */
uint8_t streamDataSet = 0;
uint8_t streamBuffRem = 0;
uint8_t sprintfBuffer[512] = {'\0'};

void vcp_TransmitCplt()
{
	memset(sprintfBuffer, '\0', strlen(sprintfBuffer));
}

void printCLI(void)
{
//	char read_Value = usart_Read(&huart1);
	char read_Value = vcp_Read();

	/* Reset all streaming variables */
	memset(sprintfBuffer, '\0', strlen(sprintfBuffer));
	if(read_Value == 'h')
	{
		sprintf(sprintfBuffer,"\n\r"
				"a - Blink\n\r"
				"d - SD Detect Pin\n\r"
				"1 - Close File\n\r"
				"2 - Sync\n\r"
				"3 - Create File\n\r"
				"s - SteamSDIO\n\r"
				"f - fetFree\n\r"
				"m - Card Info\n\r"
				"c - Card States\n\r"
				"v - Benchmark Stats\n\r"
				"z - SD Benchmark\n\r"
				"x - Write kBps\n\r"
				"l - CPU Load\n\r"
				"o - Delay--\n\r"
				"p - Delay++\n\r"
				"r - Reset CPU load\n\r"
				"\n"
				);
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'a')
	{
		if(stopBlink) stopBlink = 0;
		else stopBlink = 1;
	}
	else if(read_Value == 'd')
	{
		sprintf(sprintfBuffer,"SD Inserted\n\r");
		if(HAL_GPIO_ReadPin(SDDETECT_GPIO_Port, SDDETECT_Pin))
		{
			sprintf(sprintfBuffer,"NO SD INSERTED\n\r");
		}
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == '1')
	{
		uint8_t res = blackbox_CloseFile();
		if(res == 0){
			sprintf(sprintfBuffer,"Closed Successfully\n\r");
		}
		else{
			sprintf(sprintfBuffer,"Close Failed %u\n\r", res);
		}

		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == '2')
	{
		blackbox_Sync();
	}
	else if(read_Value == '3')
	{
		uint8_t res = blackbox_CreateFile(loggingFileSize);
		if(res == 0){
			sprintf(sprintfBuffer,"File Created\n\r");
		}
		else{
			sprintf(sprintfBuffer,"File Failed %u\n\r", res);
		}
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 's')
	{
		 streamSDIO = streamSDIO ^ 1;
	}
	else if(read_Value == 'f')
	{
		if(!streamDataSet)
		{
			streamDataSet = 1;
			streamBuffRem = 1;
		}
		else
		{
			streamDataSet = 0;
			streamBuffRem = 0;
		}
	}
	else if(read_Value == 'm')
	{
		sprintf(sprintfBuffer,"\n\r"
				"CardType %u\n\r"
				"CardVersion %u\n\r"
				"Class %u\n\r"
				"RelCardAdd %u\n\r"
				"BlockNbr %u\n\r"
				"BlockSize %u\n\r"
				"LogBlockNbr %u\n\r"
				"LogBlockSize %u\n\r"
				"\n",
				sdGetCardType(),
				sdGetCardVersion(),
				sdGetCardClass(),
				sdGetCardAdd(),
				sdGetCardBlockNber(),
				sdGetCardBlockSize(),
				sdGetCardLogBlkNber(),
				sdGetCardLogBlockSize()
				);
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'c')
	{
		sprintf(sprintfBuffer,"\n\r"
				"Ready %u\n\r"
				"Identification %u\n\r"
				"Standby %u\n\r"
				"Transfer %u\n\r"
				"Sending %u\n\r"
				"Receiving %u\n\r"
				"Programming %u\n\r"
				"Disconnected %u\n\r"
				"DMA Busy %u\n\r"
				"Buffer Full %u\n\r"
				"Error %u\n\r"
				"MB Written %u\n\r"
				"File Full %u\n\r"
				"\n",
				SD_CARD_READY_COUNT,
				SD_CARD_IDENTIFICATION_COUNT,
				SD_CARD_STANDBY_COUNT,
				SD_CARD_TRANSFER_COUNT,
				SD_CARD_SENDING_COUNT,
				SD_CARD_RECEIVING_COUNT,
				SD_CARD_PROGRAMMING_COUNT,
				SD_CARD_DISCONNECTED_COUNT,
				SD_CARD_DMA_BUSY_COUNT,
				SD_CARD_BUFFER_FULL_COUNT,
				SD_CARD_ERROR_COUNT,
				(blackboxGetBytesWritten()/(1048576)),
				blackboxIsFileFull()
				);
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'v')
	{
		sprintf(sprintfBuffer,"\n\r"
				"<300us %u\n\r"
				"<500us %u\n\r"
				"<1ms %u\n\r"
				"<2ms %u\n\r"
				"<10ms %u\n\r"
				"<200mS %u\n\r"
				">200mS %u\n\r"
				"Max ms %u\n\r"
				"\n",
				SD_CARD_LATENCY_1_300,
				SD_CARD_LATENCY_301_500,
				SD_CARD_LATENCY_501_1000,
				SD_CARD_LATENCY_1001_2000,
				SD_CARD_LATENCY_2001_10000,
				SD_CARD_LATENCY_10001_200000,
				SD_CARD_LATENCY_200msplus,
				SD_CARD_LATENCY_MAX
				);
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'z')
	{
		doSDBenchmark = doSDBenchmark ^ 1;
	}
	else if(read_Value == 'x')
	{
		sprintf(sprintfBuffer, "Write kBps %u\n\r", sdGetWritekBps());
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'l')
	{
		sprintf(sprintfBuffer,
				"\n\r"
				"CPU Load %u\n\r"
				"Min Loop %u\n\r"
				"Max Loop %u\n\r",
				cpuLoad_GetPercentLoad(),
				cpuLoad_GetMinLoopTime(),
				cpuLoad_GetMaxLoopTime());
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'o')
	{
		if(delayMicroseconds > 1) delayMicroseconds--;
		sprintf(sprintfBuffer, "Delay %u\n\r", delayMicroseconds);
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'p')
	{
		if(delayMicroseconds < 100000) delayMicroseconds++;
		sprintf(sprintfBuffer, "Delay %u\n\r", delayMicroseconds);
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else if(read_Value == 'r')
	{
		cpuLoad_Reset();
		sprintf(sprintfBuffer, "CPU Load Calculation Reset\n\r");
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
	else
	{
		sprintf(sprintfBuffer, "Invalid Command");
		CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
	}
}

void streamData()
{
	static uint32_t printLastTimeMillis = 0;
	if(streamDataSet)
	{
		if(HAL_GetTick() - printLastTimeMillis >= 5)
		{
			printLastTimeMillis = HAL_GetTick();
			if(streamBuffRem)
			{
				sprintf(sprintfBuffer, "%u\n", blackboxGetBufferAvailable());
				CDC_Transmit_FS((uint8_t*) sprintfBuffer, strlen(sprintfBuffer));
			}
		}
	}
}

//void printCLI();
//void streamData();
