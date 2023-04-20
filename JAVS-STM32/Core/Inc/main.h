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
#define LED_1_B_Pin GPIO_PIN_10
#define LED_1_B_GPIO_Port GPIOB
#define BTN_A_Pin GPIO_PIN_12
#define BTN_A_GPIO_Port GPIOB
#define BTN_B_Pin GPIO_PIN_13
#define BTN_B_GPIO_Port GPIOB
#define BTN_C_Pin GPIO_PIN_14
#define BTN_C_GPIO_Port GPIOB
#define LED_1_R_Pin GPIO_PIN_15
#define LED_1_R_GPIO_Port GPIOA
#define LED_1_G_Pin GPIO_PIN_3
#define LED_1_G_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define USB_TX_BUFFERSIZE 512

#define LCD_TIMEOUT 20000


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

extern char USB_TxBuffer[USB_TX_BUFFERSIZE];

extern volatile uint32_t LCD_TimeoutCounter;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
