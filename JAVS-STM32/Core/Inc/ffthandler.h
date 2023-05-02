/*
 * ffthandler.h
 *
 *  Created on: May 3, 2023
 *      Author: maxip
 */

#ifndef INC_FFTHANDLER_H_
#define INC_FFTHANDLER_H_


/*
 * Needed defines (either in here or in main file):
 *

#define FFT_NUM_SAMPLES 4096
#define FFT_UARTBUFFERSIZE 512
#define FFT_UARTBUFFERFLAGBYTES 8

#define FFT_NODATA 		(1<<0)
#define FFT_CALLBACK_HALF 	(1<<1)
#define FFT_CALLBACK_FULL 	(1<<2)

 */

#include "main.h"
#include "arm_math.h"

/*
 * FFT stuff
 *
 * Test Results:
 * 		FFT Length		Num. Samples / sec
 *			 1024			1340
 *			 2048			 580
 *			 4096			 265
 *			 8192			 154
 *			16384			!! Buffer Size too large !!
 *
 *
 *
 *		Num Samples		Bit -> Cycles		Num Cycles	=>	Samples / s
 *			4096			15					56
 */

#define UART_TXSKIP 0

struct FFTHandler_s {
	uint32_t uartTxStarted, uartTxStartedOld;
	uint32_t uartTxMissed, uartTxMissedOld;

	arm_rfft_fast_instance_f32 rfftf_Instance;
	uint8_t FFTPrintDebugFlag, FFTPrintDebugOnceFlag;
	uint8_t FFT_CallbackFlag;
	float32_t FFT_InputBuffer[FFT_NUM_SAMPLES];
	float32_t FFT_OutputBuffer[FFT_NUM_SAMPLES];
	uint32_t fftTransformsCompleted, fftTransformsCompletedOld;
	uint8_t uartTxSkipCounter;

	uint16_t ADC_ReadMin, ADC_ReadMax;

	uint8_t bassHistoryCounter;
	float32_t bassHistory[10];
	float32_t bassAvg;
};

typedef struct FFTHandler_s FFTHandler;

/*
 * Function prototypes
 */
void FFTHandler_Init(FFTHandler *fft);
void FFTHandler_ProcessData(FFTHandler *fft, uint16_t ADC_ReadBuffer[]);


#endif /* INC_FFTHANDLER_H_ */
