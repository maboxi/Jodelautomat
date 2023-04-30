/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
//#include "LCD_PCF8574.h"

// USB Communication Include
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * LCD Stuff
 */

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

volatile uint8_t FFT_CallbackFlag;

volatile uint16_t ADC_ReadBuffer[FFT_NUM_SAMPLES * 2];
float32_t FFT_InputBuffer[FFT_NUM_SAMPLES];
float32_t FFT_OutputBuffer[FFT_NUM_SAMPLES];

volatile uint32_t ADC_CallbackCounter;
volatile uint32_t ADC_CallbackResultsSkippedCounter;
volatile uint16_t ADC_ReadMin, ADC_ReadMax;

volatile uint8_t FFTPrintDebugOnceFlag;
uint8_t FFT_UARTBuffer[FFT_UARTBUFFERSIZE * sizeof(float32_t) + FFT_UARTBUFFERFLAGBYTES];

/*
 * USB Transmission and Receive
 */
char USB_TxBuffer[USB_TX_BUFFERSIZE];
volatile char USB_RxBuffer[USB_RX_BUFFERSIZE];

volatile uint8_t CDCReceiveFlag;
volatile uint32_t CDCReceiveDiscarded;
/*
 * Button Memory
 */
volatile uint8_t ButtonChange_Flag;
volatile uint8_t ButtonStatus_Flag, ButtonStatus;

/*
 * LCD Timeout counter and flag
 */

volatile uint32_t LCD_TimeoutCounter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	/*
	 * USB transmission and receiving
	 */
	USB_TxBuffer[USB_TX_BUFFERSIZE - 1] = 0;
	CDCReceiveFlag = 0;
	CDCReceiveDiscarded = 0;

	/*
	 * FFT Buffers and Instance
	 */
	FFT_CallbackFlag = FFT_NODATA;

	arm_rfft_fast_instance_f32 rfftf_Instance;
	arm_rfft_fast_init_f32(&rfftf_Instance, FFT_NUM_SAMPLES);

	uint32_t fftTransformsCompleted = 0, fftTransformsCompletedOld = 0;

	float32_t fftMax = 0.0;

	/*
	 * LCD 20x04 I2C
	 */
	LCD2004_I2C lcd;

	LCD_Init(&lcd, LCD_DEFAULT_ADDR);
	LCD_CursorOff(&lcd);

	HAL_Delay(2000);
	LCD_Clear(&lcd);

	// Init LED stuff
	LEDMenu_Init(&lcd);
	LEDStrips_Init();

	// Button stuff
	ButtonChange_Flag = 0;
	ButtonStatus = 0xFF;
	ButtonStatus_Flag = 0;

	// this will be incremented every 1 ms; check in loop if lcd kathode off or on
	LCD_TimeoutCounter = 0;

	// ms interrupt timer
	HAL_TIM_Base_Start_IT(&htim11);

	/*
	 * Init UART for continuous FFT Data Stream
	 */
	*((uint32_t*) &FFT_OutputBuffer[FFT_NUM_SAMPLES]) = 0x55550A0D; // set end of buffer to UU\r\n
	uint8_t uartTxSkipCounter = 0;
	uint32_t uartTxStarted = 0, uartTxStartedOld = 0;
	uint32_t uartTxMissed = 0, uartTxMissedOld = 0;

	/*
	 * ADC DMA init
	 */
	ADC_CallbackCounter = 0;
	uint32_t adcCallbackCounterOld = 0;
	ADC_CallbackResultsSkippedCounter = 0;
	uint32_t adcCallbackResultsSkippedCounterOld = 0;
	ADC_ReadMax = 0;
	ADC_ReadMin = 0 - 1;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_ReadBuffer, FFT_NUM_SAMPLES * 2);

	float32_t bassHistory[10];
	for (uint8_t i = 0; i < 10; i++)
		bassHistory[i] = 0.0f;
	float32_t bassAvg = 0.0f;

	uint8_t bassHistoryCounter = 0;

	/*
	 * Timing main loop
	 */
	uint32_t timerLast = HAL_GetTick(), timerNow;

	volatile uint8_t FFT_Print_Output = 0; // change via debugger to enable fft output; dont forget breakpoints!
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*
		 * Handle USB CDC receiving
		 */

		if (CDCReceiveFlag)
		{
			if (strncmp((const char*) USB_RxBuffer, "sendfft", 7) == 0)
			{
				FFTPrintDebugOnceFlag = 1;
			}

			CDCReceiveFlag = 0;
		}

		// do fft
		if (FFT_CallbackFlag != FFT_NODATA)
		{
			// convert adc readings
			ADC_ReadMin = 0 - 1;
			ADC_ReadMax = 0;

			uint16_t offset = 0;
			if(FFT_CallbackFlag == FFT_CALLBACK_FULL)
				offset = FFT_NUM_SAMPLES;

			for (uint16_t i = 0; i < FFT_NUM_SAMPLES; i++)
			{
				//FFT_InputDoubleBuffer[i + offset] = ((float) ADC_ReadBuffer[i + offset] * 2.0f) / 4096.0f - 1.0f;
				float reading = (float) ADC_ReadBuffer[i + offset];
				float converted = (reading - 2048.0f) / 2048.0f;

				FFT_InputBuffer[i] = converted;

				if (ADC_ReadBuffer[i + offset] < ADC_ReadMin)
					ADC_ReadMin = ADC_ReadBuffer[i + offset];
				if (ADC_ReadBuffer[i + offset] > ADC_ReadMax)
					ADC_ReadMax = ADC_ReadBuffer[i + offset];
			}

			// print adc readings
			if (FFT_Print_Output || FFTPrintDebugOnceFlag) // change via debugger to enable fft output; dont forget breakpoints!
			{
				HAL_ADC_Stop_DMA(&hadc1);
				HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_RESET);
				sprintf(USB_TxBuffer, "\r\n-- ADC Data START --\r\n");
				USB_PrintDebugForce(USB_TxBuffer);

				for (uint32_t i = 0; i < FFT_NUM_SAMPLES; i++)
				{
					//sprintf(USB_TxBuffer, "%f ", FFT_OutputBuffer[i]);
					//float32_t *fft_InputBuffer = FFT_CallbackFlag == FFT_CALLBACK_HALF ? FFT_InputBufferLower : FFT_InputBufferUpper;
					if (FFT_CallbackFlag == FFT_CALLBACK_HALF)
						sprintf(USB_TxBuffer, "%u ", ADC_ReadBuffer[i]);
					else
						sprintf(USB_TxBuffer, "%u ", ADC_ReadBuffer[FFT_NUM_SAMPLES + i]);

					USB_PrintDebugForce(USB_TxBuffer);
				}

				sprintf(USB_TxBuffer, "\r\n-- /\\ Raw Data | Converted  \\/ --\r\n");
				USB_PrintDebugForce(USB_TxBuffer);

				for (uint32_t i = 0; i < FFT_NUM_SAMPLES; i++)
				{
					sprintf(USB_TxBuffer, "%f ", FFT_InputBuffer[i]);

					USB_PrintDebugForce(USB_TxBuffer);
				}

				sprintf(USB_TxBuffer, "\r\n-- ADC Data END --\r\n");
				USB_PrintDebugForce(USB_TxBuffer);

				HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_SET);
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_ReadBuffer, FFT_NUM_SAMPLES * 2);
				//FFTPrintDebugOnceFlag = 0;
			}

			arm_rfft_fast_f32(&rfftf_Instance, FFT_InputBuffer, FFT_OutputBuffer, 0);
			//arm_cfft_f32(arm_cfft_sR_f32_len4096, p1, ifftFlag, bitReverseFlag)

			uartTxSkipCounter++;
			if (uartTxSkipCounter > UART_TXSKIP)
			{
				if (huart1.gState == HAL_UART_STATE_READY)
				{
					// setup uart buffer because fft output buffer will be overwritten once next fft starts; add \n at the end for easier transmission check
					memcpy((void*) FFT_UARTBuffer, (const void*) FFT_OutputBuffer, FFT_UARTBUFFERSIZE * sizeof(float32_t));
					//memcpy((void*) FFT_UARTBuffer, (const void*) fft_InputBuffer, FFT_UARTBUFFERSIZE * sizeof(float32_t));

					for (uint8_t i = 0; i < FFT_UARTBUFFERFLAGBYTES; i++)
						FFT_UARTBuffer[FFT_UARTBUFFERSIZE * sizeof(float32_t) + i] = 0x00;

					if (HAL_UART_Transmit_DMA(&huart1, FFT_UARTBuffer, FFT_UARTBUFFERSIZE * sizeof(float32_t) + FFT_UARTBUFFERFLAGBYTES) != HAL_BUSY)
						uartTxStarted++;
					else
						uartTxMissed++;

					uartTxSkipCounter = 0;
				}
				else
				{
					uartTxMissed++;
				}

			}

			float32_t bassMax = 0.0;

			for (uint16_t i = 1; i < FFT_NUM_SAMPLES; i++)
			{
				if (FFT_OutputBuffer[i] < 0.0f)
					FFT_OutputBuffer[i] = -FFT_OutputBuffer[i];

				if (FFT_OutputBuffer[i] > fftMax)
					fftMax = FFT_OutputBuffer[i];

				if (i < 10)
				{
					if (FFT_OutputBuffer[i] > bassMax)
						bassMax = FFT_OutputBuffer[i];
				}
			}

			if (FFT_Print_Output || FFTPrintDebugOnceFlag) // change via debugger to enable fft output; dont forget breakpoints!
			{
				HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_RESET);
				sprintf(USB_TxBuffer, "\r\n-- FFT Data START --\r\n");
				USB_PrintDebugForce(USB_TxBuffer);

				for (uint32_t i = 0; i < FFT_NUM_SAMPLES; i++)
				{
					sprintf(USB_TxBuffer, "%f ", FFT_OutputBuffer[i]);

					USB_PrintDebugForce(USB_TxBuffer);
				}

				sprintf(USB_TxBuffer, "\r\n-- FFT Data END --\r\n");
				USB_PrintDebugForce(USB_TxBuffer);

				HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_SET);
				FFTPrintDebugOnceFlag = 0;
			}

			bassAvg -= bassHistory[bassHistoryCounter];
			bassAvg += bassMax;
			bassHistory[bassHistoryCounter] = bassMax;
			bassHistoryCounter = (bassHistoryCounter + 1) % 7;

			uint16_t bassVal = 0;

			if (bassAvg > 1500.0f)
			{
				if (bassAvg > 3000.0f)
					bassVal = LEDSTRIPS_MAX_PWMDC;
				else
					bassVal = (uint16_t) ((bassAvg - 2000.0f) * 0.75f);
			}
			else
				bassVal = LEDSTRIPS_MIN_PWMDC;

			LEDStrips_UpdateFFT(bassVal, 0, 500);

			fftTransformsCompleted++;
			FFT_CallbackFlag = FFT_NODATA;
		}

		// check if afk
		if (LCD_TimeoutCounter >= LCD_TIMEOUT && lcd.lcd_backlight == LCD_BACKLIGHT_ON)
		{
			LCD_BacklightOff(&lcd);
		}

		// Handle Button events
		if (ButtonChange_Flag)
		{
			// if lcd is on, use btn events as menu input
			// else, wake up lcd
			if (lcd.lcd_backlight == LCD_BACKLIGHT_ON)
			{
				if (ButtonStatus_Flag & (1 << BTN_A_FLAG))
				{
					ButtonStatus_Flag &= ~(1 << BTN_A_FLAG);

					// btn back
					LEDMenu_UpdateState(LEDMENU_BTN_BACK, &lcd);
				}

				if (ButtonStatus_Flag & (1 << BTN_B_FLAG))
				{
					ButtonStatus_Flag &= ~(1 << BTN_B_FLAG);

					// btn select
					LEDMenu_UpdateState(LEDMENU_BTN_SELECT, &lcd);
				}

				if (ButtonStatus_Flag & (1 << BTN_C_FLAG))
				{
					ButtonStatus_Flag &= ~(1 << BTN_C_FLAG);

					// btn next item
					LEDMenu_UpdateState(LEDMENU_BTN_NEXTITEM, &lcd);
				}

				if (LEDMenu_StatusFlags & LEDMENU_FLAG_STATECHANGE)
				{
					LEDMenu_StatusFlags &= ~LEDMENU_FLAG_STATECHANGE;
				}
			}
			else
			{
				// backlight is off -> turn it on
				LCD_BacklightOn(&lcd);
				ButtonStatus_Flag = 0;
			}

			LCD_TimeoutCounter = 0;

			ButtonChange_Flag &= ~BTN_EVENT_FLAG;
		}

		// Debugging
		timerNow = HAL_GetTick();
		if (timerNow - timerLast >= 1000)
		{
			if ((ADC_CallbackResultsSkippedCounter - adcCallbackResultsSkippedCounterOld) != 0)
			{
				sprintf(USB_TxBuffer, "FFT can't keep up! Skipped %lu ADC reads (%lu total).\r\n", ADC_CallbackResultsSkippedCounter - adcCallbackResultsSkippedCounterOld, ADC_CallbackResultsSkippedCounter);
				USB_PrintDebug(USB_TxBuffer);

				adcCallbackResultsSkippedCounterOld = ADC_CallbackResultsSkippedCounter;
			}

			sprintf(USB_TxBuffer, "1/s: FFT = %lu (%f) , ADC = %lu (%u %u), UART = %lu (%lu)\r\n", fftTransformsCompleted - fftTransformsCompletedOld, fftMax, ADC_CallbackCounter - adcCallbackCounterOld, ADC_ReadMin, ADC_ReadMax, uartTxStarted - uartTxStartedOld, uartTxMissed - uartTxMissedOld);
			USB_PrintDebug(USB_TxBuffer);

			adcCallbackCounterOld = ADC_CallbackCounter;
			fftTransformsCompletedOld = fftTransformsCompleted;
			uartTxStartedOld = uartTxStarted;
			uartTxMissedOld = uartTxMissed;
			fftMax = 0.0f;

			timerLast = timerNow;
		}

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// IRQ
/*
 * IRQ for TIM11
 * used for button polling
 */
void IRQ_TIM11()
{
	/*
	 * This IRQ is called every 1ms
	 * Use for counters etc
	 */

	LCD_TimeoutCounter++;

	static uint32_t counterButtons = 0;

	counterButtons++;
	if (counterButtons >= 20)
	{
		uint8_t ButtonRead_A = HAL_GPIO_ReadPin(BTN_A_GPIO_Port, BTN_A_Pin) == GPIO_PIN_SET ? 1 : 0;
		if (ButtonRead_A == 1 && (ButtonStatus & (1 << BTN_A_FLAG)) == 0)
		{
			ButtonStatus |= (1 << BTN_A_FLAG);
		}
		else if (ButtonRead_A == 0 && (ButtonStatus & (1 << BTN_A_FLAG)) != 0)
		{
			ButtonStatus &= ~(1 << BTN_A_FLAG);
			ButtonStatus_Flag |= (1 << BTN_A_FLAG);
			ButtonChange_Flag |= BTN_EVENT_FLAG;
		}

		uint8_t ButtonRead_B = HAL_GPIO_ReadPin(BTN_B_GPIO_Port, BTN_B_Pin) == GPIO_PIN_SET ? 1 : 0;
		if (ButtonRead_B == 1 && (ButtonStatus & (1 << BTN_B_FLAG)) == 0)
		{
			ButtonStatus |= (1 << BTN_B_FLAG);
		}
		else if (ButtonRead_B == 0 && (ButtonStatus & (1 << BTN_B_FLAG)) != 0)
		{
			ButtonStatus &= ~(1 << BTN_B_FLAG);
			ButtonStatus_Flag |= (1 << BTN_B_FLAG);
			ButtonChange_Flag |= BTN_EVENT_FLAG;
		}

		uint8_t ButtonRead_C = HAL_GPIO_ReadPin(BTN_C_GPIO_Port, BTN_C_Pin) == GPIO_PIN_SET ? 1 : 0;
		if (ButtonRead_C == 1 && (ButtonStatus & (1 << BTN_C_FLAG)) == 0)
		{
			ButtonStatus |= (1 << BTN_C_FLAG);
		}
		else if (ButtonRead_C == 0 && (ButtonStatus & (1 << BTN_C_FLAG)) != 0)
		{
			ButtonStatus &= ~(1 << BTN_C_FLAG);
			ButtonStatus_Flag |= (1 << BTN_C_FLAG);
			ButtonChange_Flag |= BTN_EVENT_FLAG;
		}

		counterButtons = 0;
	}
}

// ADC interrupts

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
	{
		ADC_CallbackCounter++;
		if (FFT_CallbackFlag != FFT_NODATA)
			ADC_CallbackResultsSkippedCounter++;
		FFT_CallbackFlag = FFT_CALLBACK_FULL;
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
	{
		ADC_CallbackCounter++;
		if (FFT_CallbackFlag != FFT_NODATA)
			ADC_CallbackResultsSkippedCounter++;
		FFT_CallbackFlag = FFT_CALLBACK_HALF;
	}
}

// IRQ end

// Reset all button flags

void Buttons_ResetFlags()
{
	ButtonChange_Flag = 0;
	ButtonStatus_Flag = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
