/*
 * utils.h
 *
 *  Created on: May 3, 2023
 *      Author: maxip
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "usbd_cdc_if.h"


/*
 * USB Transmission and Receive
 */

#define USB_TX_BUFFERSIZE 512
#define USB_RX_BUFFERSIZE 64

extern char USB_TxBuffer[USB_TX_BUFFERSIZE];
extern volatile char USB_RxBuffer[USB_RX_BUFFERSIZE];

void USB_PrintDebugForce(const char *str);
void USB_PrintDebug(const char *str);

#endif /* INC_UTILS_H_ */
