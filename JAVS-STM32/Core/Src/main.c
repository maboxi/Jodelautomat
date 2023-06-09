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

// USB Communication Include
#include "usbd_cdc_if.h"
#include "utils.h"

#include "ffthandler.h"

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
 * USB virtual com port data receiving flags since receive callback cannot be moved out of usbd_cdc_if.c
 */
volatile uint8_t CDCReceiveFlag;
volatile uint32_t CDCReceiveDiscarded;

/*
 * LCD Timeout counter and flag
 */
volatile uint32_t LCD_TimeoutCounter;

/*
 * FFT handler instances
 */
FFTHandler FFT;

/*
 * Double buffer for ADC DMA
 * ADC DMA counters
 */
volatile uint16_t ADC_ReadBuffer[FFT_NUM_SAMPLES * 2];
volatile uint32_t ADC_CallbackCounter;
volatile uint32_t ADC_CallbackResultsSkippedCounter;

/*
 * Button status information
 */
volatile uint8_t ButtonChange_Flag;
volatile uint8_t ButtonStatus_Flag, ButtonStatus;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	 * init I2C LCD
	 */
	LCD2004_I2C lcd;

	LCD_Init(&lcd, LCD_DEFAULT_ADDR);
	LCD_CursorOff(&lcd);

	HAL_Delay(2000);
	LCD_Clear(&lcd);

	/*
	 * Init LED menu and LED strip handling
	 */
	LEDMenu_Init(&lcd);
	LEDStrips_Init();

	/*
	 * Init FFT handler
	 */
	FFTHandler_Init(&FFT);

	/*
	 * ADC DMA init
	 */
	ADC_CallbackCounter = 0;
	uint32_t adcCallbackCounterOld = 0;
	ADC_CallbackResultsSkippedCounter = 0;
	uint32_t adcCallbackResultsSkippedCounterOld = 0;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_ReadBuffer, FFT_NUM_SAMPLES * 2);

	/*
	 * Flags and counters for timed actions via TIM11
	 */
	ButtonChange_Flag = 0;
	ButtonStatus = 0xFF;
	ButtonStatus_Flag = 0;

	LCD_TimeoutCounter = 0; // this will be incremented every 1 ms; check in loop if lcd kathode off or on

	HAL_TIM_Base_Start_IT(&htim11);

	// main loop local variables

	/*
	 * Timing counters for main loop
	 */
	uint32_t timerLast = HAL_GetTick(), timerNow;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/*
		 * Handle USB CDC commands / data
		 */

		if (CDCReceiveFlag)
		{
			if (strncmp((const char*) USB_RxBuffer, "sendfft", 7) == 0)
			{
				FFT.FFTPrintDebugOnceFlag = 1;
			}

			CDCReceiveFlag = 0;
		}

		/*
		 * let handler process new adc readings if available
		 */
		FFTHandler_ProcessData(&FFT, (uint16_t*) ADC_ReadBuffer);

		/*
		 * check if afk -> turn off lcd to reduce power consumption
		 */
		if (LCD_TimeoutCounter >= LCD_TIMEOUT && lcd.lcd_backlight == LCD_BACKLIGHT_ON)
		{
			LCD_BacklightOff(&lcd);
		}

		/*
		 * Handle Button events
		 */
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

		/*
		 * debugging output via usb virtual com port
		 */
		timerNow = HAL_GetTick();
		if (timerNow - timerLast >= 1000)
		{
			if ((ADC_CallbackResultsSkippedCounter - adcCallbackResultsSkippedCounterOld) != 0)
			{
				sprintf(USB_TxBuffer, "FFT can't keep up! Skipped %lu ADC reads (%lu total).\r\n", ADC_CallbackResultsSkippedCounter - adcCallbackResultsSkippedCounterOld, ADC_CallbackResultsSkippedCounter);
				USB_PrintDebug(USB_TxBuffer);

				adcCallbackResultsSkippedCounterOld = ADC_CallbackResultsSkippedCounter;
			}

			sprintf(USB_TxBuffer, "1/s: FFT = %lu, ADC = %lu (%u %u), UART = %lu (%lu)\r\n", FFT.fftTransformsCompleted - FFT.fftTransformsCompletedOld, ADC_CallbackCounter - adcCallbackCounterOld, FFT.ADC_ReadMin, FFT.ADC_ReadMax, FFT.uartTxStarted - FFT.uartTxStartedOld, FFT.uartTxMissed - FFT.uartTxMissedOld);
			USB_PrintDebug(USB_TxBuffer);

			adcCallbackCounterOld = ADC_CallbackCounter;
			FFT.fftTransformsCompletedOld = FFT.fftTransformsCompleted;
			FFT.uartTxStartedOld = FFT.uartTxStarted;
			FFT.uartTxMissedOld = FFT.uartTxMissed;

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
		if (FFT.FFT_CallbackFlag != FFT_NODATA)
			ADC_CallbackResultsSkippedCounter++;
		FFT.FFT_CallbackFlag = FFT_CALLBACK_FULL;
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
	{
		ADC_CallbackCounter++;
		if (FFT.FFT_CallbackFlag != FFT_NODATA)
			ADC_CallbackResultsSkippedCounter++;
		FFT.FFT_CallbackFlag = FFT_CALLBACK_HALF;
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
