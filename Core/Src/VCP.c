/*
 * VCP.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Mathew
 */

#include "VCP.h"

#include "stm32f4xx_hal.h"

//uint8_t vcpBuffer[VCP_BUFFER_LENGTH] = {'\0'};

volatile uint8_t vcpRxHead = 0;
volatile uint8_t vcpRxTail = 0;
//volatile uint8_t vcpRxBuffer[VCP_BUFFER_LENGTH];
volatile uint8_t vcpRxBuffer[VCP_BUFFER_LENGTH];
volatile uint8_t vcpRxCount = 0;


uint8_t vcp_peek(void)
{
	if(vcpRxTail == vcpRxHead) return -1;
	else return vcpRxBuffer[vcpRxTail];
}

uint8_t vcp_Read()
{
    uint8_t readValue = 0;

    // If there is no data to read return 0
    if(0 == vcpRxCount)
    {
        return readValue;
    }

    // Get data from RxTail and increment RxTail
    readValue = vcpRxBuffer[vcpRxTail++];

    // Check if the RxTail needs to wrap around to zero
    if(sizeof(vcpRxBuffer) <= vcpRxTail)
    {
        vcpRxTail = 0;
    }

    /* Disable the UART Data Register not empty Interrupt */

//    (void)USB_DisableGlobalInt(USBx);
    vcpRxCount--;
//    (void)USB_EnableGlobalInt(USBx);

    return readValue;
}

void vcp_RxDataHandler(uint8_t* _Buf, uint32_t *_Len)
{

	for(int i=0; i< *_Len; i++)
	{
		if(vcpRxTail - 1 != vcpRxHead)
		{
			// Store data in buffer and increment RxHead
			vcpRxBuffer[vcpRxHead++] = _Buf[i];
			// Increment data count
			//    (void)USB_DisableGlobalInt(USBx);
			vcpRxCount++;
			//    (void)USB_EnableGlobalInt(USBx);

			// Check if the RxHead needs to be wrapped back to zero
			if(sizeof(vcpRxBuffer) <= vcpRxHead)
			{
				vcpRxHead = 0;
			}
		}
	}

}

void vcp_DefaultFramingErrorHandler(){}

void vcp_DefaultOverrunErrorHandler(){} // vcp error - restart

//void vcp_DefaultErrorHandler(){}

// Safely return RxCount without race conditions
uint8_t vcp_getRxCount()
{
//  __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
  uint8_t RxCount = vcpRxCount;
//  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
  return RxCount;
}

__weak void vcp_TransmitCplt()
{
//	unused();
}

