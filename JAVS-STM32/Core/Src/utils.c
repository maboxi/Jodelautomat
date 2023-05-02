/*
 * utils.c
 *
 *  Created on: May 3, 2023
 *      Author: maxip
 */
#include "utils.h"

char USB_TxBuffer[USB_TX_BUFFERSIZE];
volatile char USB_RxBuffer[USB_RX_BUFFERSIZE];

void USB_PrintDebugForce(const char *str)
{
	char txTmpBuffer[USB_TX_BUFFERSIZE + 2];

	sprintf(txTmpBuffer, "%s", str);
	uint16_t length = strlen(txTmpBuffer);

	txTmpBuffer[USB_TX_BUFFERSIZE + 1] = 0;

	while (CDC_Transmit_FS((uint8_t*) txTmpBuffer, length) != USBD_OK)
		;
}

void USB_PrintDebug(const char *str)
{
	char txTmpBuffer[USB_TX_BUFFERSIZE + 2];

	sprintf(txTmpBuffer, "%s", str);
	uint16_t length = strlen(txTmpBuffer);

	txTmpBuffer[USB_TX_BUFFERSIZE + 1] = 0;

	uint8_t counter = 0, result;
	while (counter < 3)
	{
		result = CDC_Transmit_FS((uint8_t*) txTmpBuffer, length);
		if (result == USBD_OK)
			break;

		counter++;
		if (counter > 0)
		{

		}
	}
}
