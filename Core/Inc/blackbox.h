/*
 * blackbox.h
 *
 *  Created on: Nov 26, 2023
 *      Author: Mathew
 *
 *
 *      FUTURE IMPROVEMENTS
 *      - Unify function call returns to the FRESULT of fatfs
 *      - Check for state based on SD initialize -> file created -> file contiguous
 *      	- Right now only using single SDInitialized variable but no check for file created successfully
 *
 *
 *
 */

#ifndef INC_BLACKBOX_H_
#define INC_BLACKBOX_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"


#define BLACKBOX_OK 0
#define BLACKBOX_ERROR 1
#define BLACKBOX_INIT_FAIL 2

#define KILOBYTE 1024
#define MEGABYTE 1048576

typedef enum {
	BB_FILE_READY = 0,		/* (0) File ready for writing */
	BB_INITIALIZED,			/* (1) SD card initialized but no files created */
	BB_ERROR,				/* (2) SD card error  */
	BB_FILE_FULL,			/* (3) File is full */
	BB_NONE,				/* (4) Default state */
} BLACKBOXSTATE;

/* SD state counts */
extern uint32_t SD_CARD_BUFFER_FULL_COUNT; /* Seperate from SD card */
extern uint32_t SD_CARD_DMA_BUSY_COUNT; /* Seperate from SD card */
extern uint32_t SD_CARD_READY_COUNT; /*!< Card state is ready                     */
extern uint32_t SD_CARD_IDENTIFICATION_COUNT;   /*!< Card is in identification state         */
extern uint32_t SD_CARD_STANDBY_COUNT;   /*!< Card is in standby state                */
extern uint32_t SD_CARD_TRANSFER_COUNT;   /*!< Card is in transfer state               */
extern uint32_t SD_CARD_SENDING_COUNT;   /*!< Card is sending an operation            */
extern uint32_t SD_CARD_RECEIVING_COUNT;   /*!< Card is receiving operation information */
extern uint32_t SD_CARD_PROGRAMMING_COUNT;   /*!< Card is in programming state            */
extern uint32_t SD_CARD_DISCONNECTED_COUNT;   /*!< Card is disconnected                    */
extern uint32_t SD_CARD_ERROR_COUNT;   /*!< Card response Error         */

/* SD latency counts */
extern uint32_t SD_CARD_LATENCY_1_300;
extern uint32_t SD_CARD_LATENCY_301_500;
extern uint32_t SD_CARD_LATENCY_501_1000;
extern uint32_t SD_CARD_LATENCY_1001_2000;
extern uint32_t SD_CARD_LATENCY_2001_10000;
extern uint32_t SD_CARD_LATENCY_10001_200000;
extern uint32_t SD_CARD_LATENCY_200msplus;
extern uint32_t SD_CARD_LATENCY_MAX;



/* Initialize SD card
 * Function calls BSP_SD_Init() which first checks if SD card inserted with detect pin
 * Mounts SD card to root
 * Polls SD card info and stores in variable -- improve to struct later
 * */
uint8_t blackbox_Init();

/* Get card info */
uint8_t blackbox_GetCardInfo();

/* Create new file */
uint8_t blackbox_CreateFile(uint32_t fileSize);

/* Buffer data for the SD card */
uint8_t blackbox_BufferData(uint8_t* Buf, uint16_t Len);

/* Write a sector to the SD card */
uint8_t blackbox_WriteSector(uint8_t* Buf);

/* Write a slow blocking random length of data */
uint8_t blackbox_WriteRandom(uint8_t* Buf, uint16_t Len);

/* Send data to the SD card if there is enough for a full sector
  * @retval FR_OK if data sent or nothing to do */
uint8_t blackbox_ProcessData();

/* Sync file and truncate file size */
uint8_t blackbox_Sync();

/* Close the file */
uint8_t blackbox_CloseFile();

/* Check state of SD card */
uint32_t sdCheckState();

/* Reset buffers and buffer indexing */
void blackboxFlushBuffers();

/* Return number of bytes available in the buffer */
uint16_t blackboxGetBufferAvailable();

/* Return number of bytes stored in the buffer */
uint16_t blackboxGetDataStored();

/* Track logging performance kBps
 * New time based measurement called in the blackbox_ProcessData()
 * */
void blackboxMeasureWritekBps();

uint8_t blackbox_SetHeader(uint8_t* Buf, uint16_t Len);

uint8_t blackbox_WriteHeader();


/* Mutator functions for SD card info
 * I should really figure out a cleaner way to do this
 * At least its better than global variables
 * */
uint32_t blackboxIsInitialized();
uint32_t sdGetCardType();
uint32_t sdGetCardVersion();
uint32_t sdGetCardClass();
uint32_t sdGetCardAdd();
uint32_t sdGetCardBlockNber();
uint32_t sdGetCardBlockSize();
uint32_t sdGetCardLogBlkNber();
uint32_t sdGetCardLogBlockSize();
uint32_t sdGetWritekBps();
uint32_t blackboxGetBytesWritten();
uint8_t blackboxIsFileFull();

void sdMeasureLatency(uint32_t microsDelta);
void blackboxSDBenchmark(uint8_t runFlag);



















#endif /* INC_BLACKBOX_H_ */
