/*
 * ffthandler.c
 *
 *  Created on: May 3, 2023
 *      Author: maxip
 */

#include "ffthandler.h"
#include "utils.h"

#include "usart.h"
#include "usbd_cdc_if.h"

uint8_t FFT_UARTBuffer[FFT_UARTBUFFERSIZE * sizeof(float32_t) + FFT_UARTBUFFERFLAGBYTES];

// uart tranmission information

void FFTHandler_Init(FFTHandler *fft)
{
	/*
	 * FFT Buffers and Instance
	 */
	fft->FFT_CallbackFlag = FFT_NODATA;

	arm_rfft_fast_init_f32(&fft->rfftf_Instance, FFT_NUM_SAMPLES);

	fft->fftTransformsCompleted = 0;
	fft->fftTransformsCompletedOld = 0;

	/*
	 * Init UART for continuous FFT Data Stream
	 */
	fft->uartTxStarted = 0;
	fft->uartTxStartedOld = 0;
	fft->uartTxMissed = 0;
	fft->uartTxMissedOld = 0;

	*((uint32_t*) &fft->FFT_OutputBuffer[FFT_NUM_SAMPLES]) = 0x55550A0D; // set end of buffer to UU\r\n
	fft->uartTxSkipCounter = 0;

	fft->bassHistoryCounter = 0;
	for (uint8_t i = 0; i < 10; i++)
		fft->bassHistory[i] = 0.0f;
	fft->bassAvg = 0.0f;

	fft->FFTPrintDebugFlag = 0;
	fft->FFTPrintDebugOnceFlag = 0;
}

void FFTHandler_ProcessData(FFTHandler *fft, uint16_t ADC_ReadBuffer[])
{
	// do fft
	if (fft->FFT_CallbackFlag != FFT_NODATA)
	{
		// convert adc readings
		fft->ADC_ReadMin = 0 - 1;
		fft->ADC_ReadMax = 0;

		uint16_t offset = 0;
		if (fft->FFT_CallbackFlag == FFT_CALLBACK_FULL)
			offset = FFT_NUM_SAMPLES;

		for (uint16_t i = 0; i < FFT_NUM_SAMPLES; i++)
		{
			//FFT_InputDoubleBuffer[i + offset] = ((float) ADC_ReadBuffer[i + offset] * 2.0f) / 4096.0f - 1.0f;
			float reading = (float) ADC_ReadBuffer[i + offset];
			float converted = (reading - 2048.0f) / 2048.0f;

			fft->FFT_InputBuffer[i] = converted;

			if (ADC_ReadBuffer[i + offset] < fft->ADC_ReadMin)
				fft->ADC_ReadMin = ADC_ReadBuffer[i + offset];
			if (ADC_ReadBuffer[i + offset] > fft->ADC_ReadMax)
				fft->ADC_ReadMax = ADC_ReadBuffer[i + offset];
		}


		arm_rfft_fast_f32(&fft->rfftf_Instance, fft->FFT_InputBuffer, fft->FFT_OutputBuffer, 0);
		//arm_cfft_f32(arm_cfft_sR_f32_len4096, p1, ifftFlag, bitReverseFlag)

		fft->uartTxSkipCounter++;
		if (fft->uartTxSkipCounter > UART_TXSKIP)
		{
			if (huart1.gState == HAL_UART_STATE_READY)
			{
				// setup uart buffer because fft output buffer will be overwritten once next fft starts; add \n at the end for easier transmission check
				memcpy((void*) FFT_UARTBuffer, (const void*) fft->FFT_OutputBuffer, FFT_UARTBUFFERSIZE * sizeof(float32_t));
				//memcpy((void*) FFT_UARTBuffer, (const void*) fft_InputBuffer, FFT_UARTBUFFERSIZE * sizeof(float32_t));

				for (uint8_t i = 0; i < FFT_UARTBUFFERFLAGBYTES; i++)
					FFT_UARTBuffer[FFT_UARTBUFFERSIZE * sizeof(float32_t) + i] = 0x00;

				if (HAL_UART_Transmit_DMA(&huart1, FFT_UARTBuffer, FFT_UARTBUFFERSIZE * sizeof(float32_t) + FFT_UARTBUFFERFLAGBYTES) != HAL_BUSY)
					fft->uartTxStarted++;
				else
					fft->uartTxMissed++;

				fft->uartTxSkipCounter = 0;
			}
			else
			{
				fft->uartTxMissed++;
			}

		}


		if (fft->FFTPrintDebugFlag || fft->FFTPrintDebugOnceFlag) // change via debugger to enable fft output; dont forget breakpoints!
		{
			HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_RESET);
			sprintf(USB_TxBuffer, "\r\n-- FFT Data START --\r\n");
			USB_PrintDebugForce(USB_TxBuffer);

			for (uint32_t i = 0; i < FFT_NUM_SAMPLES; i++)
			{
				sprintf(USB_TxBuffer, "%f ", fft->FFT_OutputBuffer[i]);

				USB_PrintDebugForce(USB_TxBuffer);
			}

			sprintf(USB_TxBuffer, "\r\n-- FFT Data END --\r\n");
			USB_PrintDebugForce(USB_TxBuffer);

			HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_SET);
			fft->FFTPrintDebugOnceFlag = 0;
		}


		fft->fftTransformsCompleted++;
		fft->FFT_CallbackFlag = FFT_NODATA;
	}
}
