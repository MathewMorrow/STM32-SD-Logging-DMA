/*
 * blackbox.c
 *
 *  Created on: Nov 26, 2023
 *      Author: Mathew
 */

#include "blackbox.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_sd.h"
#include "stm32f4xx_hal_conf.h"
//#include "sd_diskio.h" // Creates issue with Diskio_drivTpeDef
#include "fatfs.h"
#include "stdint.h"
#include "time.h"
//#include "ff_gen_drv.h"
//#include "bsp_driver_sd.h"
#include "main.h"

#define BUFFER_SIZE 32768
//#define SECTOR_SIZE 512
#define SECTOR_SIZE 4096

#define BLACKBOX_DEBUG 1
//#define BLACKBOX_CHECK_CONTIG	// Uncomment to check if the newly created file is contiguous -- takes hundred of milliseconds

extern SD_HandleTypeDef hsd; // Link to the SD card Handler
extern DMA_HandleTypeDef hdma_sdio_tx; // Link to the DMA handler to check if busy

/* SD card global data */
uint8_t SDInitialized = 0;
uint32_t writeSpeedkBps = 0;

/* SD card file data */
FRESULT fr;
FATFS fs;
FIL myFile;
UINT bytesWritten;
BLACKBOXSTATE blackboxState = BB_NONE;

/* SD card info */
BSP_SD_CardInfo sdCardInfo;
uint32_t CardType;                     /*!< Specifies the card Type                         */
uint32_t CardVersion;                  /*!< Specifies the card version                      */
uint32_t Class;                        /*!< Specifies the class of the card class           */
uint32_t RelCardAdd;                   /*!< Specifies the Relative Card Address             */
uint32_t BlockNbr;                     /*!< Specifies the Card Capacity in blocks           */
uint32_t BlockSize;                    /*!< Specifies one block size in bytes               */
uint32_t LogBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */
uint32_t LogBlockSize;                 /*!< Specifies logical block size in bytes           */

/* SD state counts */
uint32_t SD_CARD_BUFFER_FULL_COUNT = 0;		/* External from from SD card */
uint32_t SD_CARD_DMA_BUSY_COUNT = 0; 		/* External from from SD card */
uint32_t SD_CARD_READY_COUNT = 0; 			/*!< Card state is ready                     */
uint32_t SD_CARD_IDENTIFICATION_COUNT = 0;  /*!< Card is in identification state         */
uint32_t SD_CARD_STANDBY_COUNT = 0;   		/*!< Card is in standby state                */
uint32_t SD_CARD_TRANSFER_COUNT = 0;   		/*!< Card is in transfer state               */
uint32_t SD_CARD_SENDING_COUNT = 0;   		/*!< Card is sending an operation            */
uint32_t SD_CARD_RECEIVING_COUNT = 0;  		/*!< Card is receiving operation information */
uint32_t SD_CARD_PROGRAMMING_COUNT = 0;   	/*!< Card is in programming state            */
uint32_t SD_CARD_DISCONNECTED_COUNT = 0;   	/*!< Card is disconnected                    */
uint32_t SD_CARD_ERROR_COUNT = 0;   		/*!< Card response Error                     */

/* SD latency counts */
uint32_t SD_CARD_LATENCY_1_300 = 0;
uint32_t SD_CARD_LATENCY_301_500 = 0;
uint32_t SD_CARD_LATENCY_501_1000 = 0;
uint32_t SD_CARD_LATENCY_1001_2000 = 0;
uint32_t SD_CARD_LATENCY_2001_10000 = 0;
uint32_t SD_CARD_LATENCY_10001_200000 = 0;
uint32_t SD_CARD_LATENCY_200msplus = 0;
uint32_t SD_CARD_LATENCY_MAX = 0;

/* Internally buffered data */
uint8_t blackboxWriteBuff1[SECTOR_SIZE] = {'\0'};
uint8_t blackboxWriteBuff2[SECTOR_SIZE] = {'\0'};
uint8_t *blackboxCurrentBuff;
uint16_t blackboxBuffIndex = 0;
uint8_t blackboxDataPending = 0;

/* File header */
uint8_t blackboxHeader[256] = "No Header Set\n";
uint8_t blackboxHeaderLength = strlen("No Header Set\n");

/* File meta data */
static uint32_t blackboxFileSizeBytes = 0;
static uint32_t blackboxBytesWritten = 0;


/* Initialize SD card
 * Function calls BSP_SD_Init() which first checks if SD card inserted with detect pin
 * Mounts SD card to root
 * Polls SD card info and stores in variable -- improve to struct later
 * */
uint8_t blackbox_Init()
{
	/*Initialize Buffer*/
	blackboxBuffIndex = 0;
	blackboxDataPending = 0;
	blackboxCurrentBuff = blackboxWriteBuff1;

	uint8_t returnMSD = BSP_SD_Init();
	if(returnMSD == MSD_OK){
		fr = f_mount(&fs, "/", 1); // Initialize SD card and get card info
		if(fr == FR_OK)
		{
			SDInitialized = 1; // Depricated for typedef BLACKBOXSTATE
			blackboxState = BB_INITIALIZED;
			blackbox_GetCardInfo();
		}
	}
	else{
		blackboxState = BB_ERROR;
		fr = FR_DISK_ERR;
	}
	return fr;
}

/* Get card info */
uint8_t blackbox_GetCardInfo()
{
	if(blackboxState != BB_INITIALIZED) return FR_NOT_READY;

	if(HAL_SD_GetCardInfo(&hsd, &sdCardInfo) == HAL_OK)
	{
		  CardType = sdCardInfo.CardType;            /*!< Specifies the card Type                         */
		  CardVersion = sdCardInfo.CardVersion;      /*!< Specifies the card version                      */
		  Class = sdCardInfo.Class;                  /*!< Specifies the class of the card class           */
		  RelCardAdd = sdCardInfo.RelCardAdd;        /*!< Specifies the Relative Card Address             */
		  BlockNbr = sdCardInfo.BlockNbr;            /*!< Specifies the Card Capacity in blocks           */
		  BlockSize = sdCardInfo.BlockSize;          /*!< Specifies one block size in bytes               */
		  LogBlockNbr = sdCardInfo.LogBlockNbr;      /*!< Specifies the Card logical Capacity in blocks   */
		  LogBlockSize = sdCardInfo.LogBlockSize;    /*!< Specifies logical block size in bytes           */
		  return BLACKBOX_OK;
	}
	return BLACKBOX_ERROR;
}


/* Create new file */
uint8_t blackbox_CreateFile(uint32_t fileSize)
{
	if(blackboxState != BB_INITIALIZED) return FR_NOT_READY;

	uint8_t fileIndex = 0;	// File indexing count
	TCHAR myFileName[32] = {'\0'}; // Path name
	DIR dj; // Object to hold directory data
	FILINFO fno; // Object to hold file info

	do{ /*!< Search for highest file index count and increment once */
		fileIndex++;
		sprintf(myFileName, "File%i.txt", fileIndex);
		fr = f_findfirst(&dj, &fno, "/", myFileName);
	}while(fr == FR_OK && fno.fname[0]);

	if(fr == FR_OK)
	{
		fr = f_open((&myFile), myFileName, FA_CREATE_NEW  | FA_WRITE); // Open file with settings
		if(fr == FR_OK)
		{
			/* Allocate a contiguous area to the file */
			fr = f_truncate(&myFile);
			HAL_Delay(10);
			if(fr == FR_OK)
			{
				fr = f_expand(&myFile, fileSize, 1); // Create contiguous file for faster write transactions later
				if(fr == FR_OK)
				{
					int isFileContiguous = 1;
#ifdef BLACKBOX_CHECK_CONTIG
					fr = test_contiguous_file(&myFile, &isFileContiguous);
#endif
					if(fr == FR_OK && isFileContiguous)
					{
						fr = f_rewind(&myFile);
						blackboxState = BB_FILE_READY;
						if(fr == FR_OK)
						{
							fr = blackbox_WriteHeader();
							blackboxBytesWritten = 0;
							blackboxFileSizeBytes = fileSize;
						}
					}
				}
			}
		}
	}
	if(fr != FR_OK)
	{
		blackboxState = BB_ERROR;
	}
	return fr;
}

uint8_t blackbox_SetHeader(uint8_t* Buf, uint16_t Len)
{
	if(Len == 0) return BLACKBOX_ERROR;

	blackboxHeaderLength = Len;
	for(int i = 0; i < Len; i++)
	{
		blackboxHeader[i] = Buf[i];
	}

	return BLACKBOX_OK;
}

uint8_t blackbox_WriteHeader()
{
	return blackbox_BufferData(blackboxHeader, blackboxHeaderLength);
}


/* Write a fast sector to the SD card */
uint8_t blackbox_WriteSector(uint8_t* Buf)
{
	if(blackboxState != BB_FILE_READY) return FR_NOT_READY;

	/* Check if the sd card is ready for a write */
	if(sdCheckState() == HAL_SD_CARD_TRANSFER)
	{
		sd_FastWriteFlag = 1;
		fr = f_write(&myFile, Buf, SECTOR_SIZE, &bytesWritten);
		sd_FastWriteFlag = 0;
		if(fr == FR_OK)
		{
			blackboxBytesWritten += SECTOR_SIZE;
		}
	}
	else
	{
		fr = FR_NOT_READY;
	}

	return fr;
}

/* Write a slow blocking random length of data
 * Used for sending last bytes of data before closign file
 *  */
uint8_t blackbox_WriteRandom(uint8_t* Buf, uint16_t Len)
{
	if(blackboxState != BB_FILE_READY) return FR_NOT_READY;

	/* Check if the sd card is ready for a write */
	if(sdCheckState() == HAL_SD_CARD_TRANSFER)
	{
		fr = f_write(&myFile, Buf, Len, &bytesWritten);
		blackboxBytesWritten += bytesWritten;
	}
	else
	{
		fr = FR_NOT_READY;
	}

	return fr;
}

/* Write data to blackbox
 *  @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY*/
uint8_t blackbox_BufferData(uint8_t* Buf, uint16_t Len)
{
	fr = FR_OK;

	if(blackboxState != BB_FILE_READY) return FR_NOT_READY;

	/* Drop packet if SD buffers full */
	if( blackboxDataPending && Len >= (SECTOR_SIZE - blackboxBuffIndex) )
	{
		return FR_NOT_READY;
	}

	/* Store data in current buffer */
	for(int k = 0; k < Len; k++)
	{
		blackboxCurrentBuff[blackboxBuffIndex++] = Buf[k];

		/* If buffer is full, flip flop to other buffer and set buffer full flag */
		if(blackboxBuffIndex == SECTOR_SIZE)
		{
			blackboxDataPending = 1;
			blackboxBuffIndex = 0;
			blackboxCurrentBuff = (blackboxCurrentBuff == blackboxWriteBuff1) ? blackboxWriteBuff2 : blackboxWriteBuff1;
		}
	}
	return fr;
}



/* Send data to the SD card if there is enough for a full sector
 * If new dataPending / send to DMA takes 55uS otherwise <1uS
  * @retval FR_OK if data sent or nothing to do */
uint8_t blackbox_ProcessData()
{
	fr = FR_OK;

#if BLACKBOX_DEBUG == 1
	blackboxMeasureWritekBps();
#endif

	if(blackboxDataPending == 1)
	{
		HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_SET);
		if(blackboxIsFileFull()) return BB_FILE_FULL;

		if(blackboxCurrentBuff == blackboxWriteBuff1)
		{
			fr = blackbox_WriteSector(blackboxWriteBuff2);
		}
		else
		{
			fr = blackbox_WriteSector(blackboxWriteBuff1);
		}
		if(fr == FR_OK)
		{
			blackboxDataPending = 0;
		}
		HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_RESET);
	}

	return fr;
}

/* Check if we have reached the end of the contiguous file
 * @retval FR_OK if data sent or nothing to do
 * */
uint8_t blackboxIsFileFull()
{
	return (blackboxFileSizeBytes - blackboxBytesWritten < SECTOR_SIZE) ? 1 : 0;
}

/* Sync file and truncate file size */
uint8_t blackbox_Sync()
{
	if(blackboxState != BB_FILE_READY) return FR_NOT_READY;

	fr = f_truncate(&myFile);
	if(fr == FR_OK)
	{
		HAL_Delay(20);
		fr = f_sync(&myFile);
	}
	return fr;
}

/* Close the file
 * Send all remaining buffer data
 * Truncate the file
 * Sync and close - f_close calls sync */
uint8_t blackbox_CloseFile()
{
	if(blackboxState != BB_FILE_READY) return FR_NOT_READY;

	uint32_t startFileCLoseMillis = HAL_GetTick();
	while(blackboxDataPending && HAL_GetTick() - startFileCLoseMillis < 300)
	{
		blackbox_ProcessData();
	}

	blackbox_WriteRandom(blackboxCurrentBuff, blackboxBuffIndex);

	fr = f_truncate(&myFile);
	if(fr == FR_OK)
	{
		HAL_Delay(10);
		fr = f_close(&myFile);
		if(fr == FR_OK)
		{
			blackboxFlushBuffers();
			blackboxState = BB_INITIALIZED;
		}
		else
		{
//			blackboxState = BB_ERROR;
		}
	}
	return fr;
}



uint32_t sdCheckState()
{
	if(blackboxState != BB_FILE_READY) return FR_NOT_READY;

	uint32_t SD_CardState = 0;

	/* Check if the DMA is busy */
	if(HAL_DMA_GetState(&hdma_sdio_tx) != HAL_DMA_STATE_READY)
	{
		SD_CARD_DMA_BUSY_COUNT++;
		return FR_NOT_READY;
	}
	/* Check if the SD card is ready for action */

	SD_CardState = HAL_SD_GetCardState(&hsd);
#if BLACKBOX_DEBUG == 1
	if(SD_CardState == HAL_SD_CARD_TRANSFER)SD_CARD_TRANSFER_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_READY) SD_CARD_READY_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_IDENTIFICATION) SD_CARD_IDENTIFICATION_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_STANDBY) SD_CARD_STANDBY_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_SENDING) SD_CARD_SENDING_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_RECEIVING) SD_CARD_RECEIVING_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_PROGRAMMING) SD_CARD_PROGRAMMING_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_DISCONNECTED) SD_CARD_DISCONNECTED_COUNT++;
	else if(SD_CardState == HAL_SD_CARD_ERROR) SD_CARD_ERROR_COUNT++;
#endif
	return SD_CardState;
}

void blackboxFlushBuffers()
{
	for(int k = 0; k < SECTOR_SIZE; k++)
	{
		blackboxWriteBuff1[k] = '\0';
		blackboxWriteBuff2[k] = '\0';
	}
	/*Initialize Buffer*/
	blackboxBuffIndex = 0;
	blackboxDataPending = 0;
	blackboxCurrentBuff = blackboxWriteBuff1;
}

uint16_t blackboxGetBufferAvailable()
{
	uint16_t availableBytes;
	availableBytes = SECTOR_SIZE - blackboxBuffIndex;
//	if(blackboxHead >= blackboxTail)
//	{
//		availableBytes = BUFFER_SIZE - blackboxHead + blackboxTail;
//	}
//	else
//	{
//		availableBytes = blackboxTail - blackboxHead;
//	}
	return availableBytes;
}

uint16_t blackboxGetDataStored()
{
	uint16_t bytesStored;
//	if(blackboxHead >= blackboxTail)
//	{
//		bytesStored = blackboxHead - blackboxTail;
//	}
//	else
//	{
//		bytesStored = BUFFER_SIZE - blackboxTail + blackboxHead;
//	}
	return bytesStored;
}


/* Track logging performance kBps */
void blackboxMeasureWritekBps(uint32_t bytesWritten)
{
	static uint32_t blackboxlastPeriodMillis = 0;
	static uint32_t blackboxLastBytesWritten = 0;

	if(HAL_GetTick() - blackboxlastPeriodMillis >= 1000)
	{
		blackboxlastPeriodMillis = HAL_GetTick();
		writeSpeedkBps = (blackboxBytesWritten - blackboxLastBytesWritten)/1024;
		blackboxLastBytesWritten = blackboxBytesWritten;
	}
}

void blackboxSDBenchmark(uint8_t runFlag)
{
	static uint8_t benchmarkStarted = 0;
	static uint8_t benchmarkReset = 0;
	static uint32_t microsLast = 0;

	if(runFlag)
	{
		if(blackboxState != BB_FILE_READY) return FR_NOT_READY;

		if(benchmarkStarted == 0)
		{
			/* Reset start stop flags */
			benchmarkStarted = 1;
			benchmarkReset = 0;
			/* Reset latency data */
			SD_CARD_LATENCY_1_300 = 0;
			SD_CARD_LATENCY_301_500 = 0;
			SD_CARD_LATENCY_501_1000 = 0;
			SD_CARD_LATENCY_1001_2000 = 0;
			SD_CARD_LATENCY_2001_10000 = 0;
			SD_CARD_LATENCY_10001_200000 = 0;
			SD_CARD_LATENCY_200msplus = 0;
			SD_CARD_LATENCY_MAX = 0;
			/* Fill sector sized buffer with data */
			for(int i = 0; i < SECTOR_SIZE; i++)
			{
				blackboxWriteBuff1[i] = 'a';
			}
			/* Get starting micros */
			microsLast = getMircos();
		}
		else
		{
			/* Check if the sd card is ready for a write */
			if(sdCheckState() == HAL_SD_CARD_TRANSFER)
			{
				sd_FastWriteFlag = 1;
				fr = f_write(&myFile, blackboxWriteBuff1, SECTOR_SIZE, &bytesWritten);
				sd_FastWriteFlag = 0;
				if(fr == FR_OK)
				{
					uint32_t nowMicros = getMircos();
					sdMeasureLatency(nowMicros - microsLast);
					microsLast = nowMicros;
				}
			}

		}

	}
	else
	{
		if(benchmarkReset == 0)
		{
			benchmarkReset = 1;
			benchmarkStarted = 0;
		}
	}

}

void sdMeasureLatency(uint32_t microsDelta)
{
	if(microsDelta <= 300) {SD_CARD_LATENCY_1_300++;}
	else if(microsDelta <= 500) {SD_CARD_LATENCY_301_500++;}
	else if(microsDelta <= 1000) {SD_CARD_LATENCY_501_1000++;}
	else if(microsDelta <= 2000) {SD_CARD_LATENCY_1001_2000++;}
	else if(microsDelta <= 10000) {SD_CARD_LATENCY_2001_10000++;}
	else if(microsDelta <= 200000) {SD_CARD_LATENCY_10001_200000++;}
	else{SD_CARD_LATENCY_200msplus++;}

	if(microsDelta > SD_CARD_LATENCY_MAX)
	{
		SD_CARD_LATENCY_MAX = microsDelta;
	}
}


/* Mutator functions for sd card info */
uint32_t blackboxIsInitialized(){return SDInitialized;}
uint32_t sdGetCardType(){return CardType;}
uint32_t sdGetCardVersion(){return CardVersion;}
uint32_t sdGetCardClass(){return Class;}
uint32_t sdGetCardAdd(){return RelCardAdd;}
uint32_t sdGetCardBlockNber(){return BlockNbr;}
uint32_t sdGetCardBlockSize(){return BlockSize;}
uint32_t sdGetCardLogBlkNber(){return LogBlockNbr;}
uint32_t sdGetCardLogBlockSize(){return LogBlockSize;}
uint32_t sdGetWritekBps(){return writeSpeedkBps;}
uint32_t blackboxGetBytesWritten(){return blackboxBytesWritten;}


