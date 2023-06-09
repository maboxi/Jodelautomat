/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "LCD_PCF8574.h"
#include "LEDConfigMenu.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void USB_PrintDebug(const char *str);
void IRQ_TIM11();
void Buttons_ResetFlags();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ONBOARD_Pin GPIO_PIN_13
#define LED_ONBOARD_GPIO_Port GPIOC
#define BTN_ONBOARD_Pin GPIO_PIN_0
#define BTN_ONBOARD_GPIO_Port GPIOA
#define LED_1_B_Pin GPIO_PIN_0
#define LED_1_B_GPIO_Port GPIOB
#define LED_1_G_Pin GPIO_PIN_1
#define LED_1_G_GPIO_Port GPIOB
#define LED_1_R_Pin GPIO_PIN_10
#define LED_1_R_GPIO_Port GPIOB
#define BTN_C_Pin GPIO_PIN_12
#define BTN_C_GPIO_Port GPIOB
#define BTN_B_Pin GPIO_PIN_13
#define BTN_B_GPIO_Port GPIOB
#define BTN_A_Pin GPIO_PIN_14
#define BTN_A_GPIO_Port GPIOB
#define LED_2_R_Pin GPIO_PIN_15
#define LED_2_R_GPIO_Port GPIOA
#define LED_2_G_Pin GPIO_PIN_3
#define LED_2_G_GPIO_Port GPIOB
#define LED_2_B_Pin GPIO_PIN_4
#define LED_2_B_GPIO_Port GPIOB
#define LED_3_R_Pin GPIO_PIN_5
#define LED_3_R_GPIO_Port GPIOB
#define LED_3_G_Pin GPIO_PIN_6
#define LED_3_G_GPIO_Port GPIOB
#define LED_3_B_Pin GPIO_PIN_7
#define LED_3_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/*
 *
 */

#define LCD_TIMEOUT 20000


/*
 * FFT Defines
 */
#define FFT_NUM_SAMPLES 4096
#define FFT_UARTBUFFERSIZE 512
#define FFT_UARTBUFFERFLAGBYTES 8

#define FFT_NODATA 		(1<<0)
#define FFT_CALLBACK_HALF 	(1<<1)
#define FFT_CALLBACK_FULL 	(1<<2)

/*
 * Button Defines
 */

#define BTN_EVENT_FLAG 1

#define BTN_A_FLAG 0
#define BTN_B_FLAG 1
#define BTN_C_FLAG 2

/*
 * Variables
 */


extern volatile uint32_t LCD_TimeoutCounter;

extern volatile uint8_t CDCReceiveFlag;
extern volatile uint32_t CDCReceiveDiscarded;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
