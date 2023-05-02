/*
 * LEDConfigMenu.c
 *
 *  Created on: 17.04.2023
 *      Author: maxip
 */

#include "LEDConfigMenu.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "tim.h"
#include "utils.h"

/*
 * Menu variables
 */
uint8_t LEDMenu_StatusFlags;

/*
 * Config memory and string lookup arrays
 */
uint8_t LEDStrips_Config[LED_NUM_STRIPS][LED_NUM_COLORS];
uint8_t LEDSelectionMenu_State;

volatile uint32_t *LEDStrips_Config_Counter[LED_NUM_STRIPS][LED_NUM_COLORS];

uint8_t LEDSelectionMenu_Memory[LEDMENU_NUM];

const char LED_StripNames_Long[LED_NUM_STRIPS][9] = { "Strip 01", "Unten", "Vorne" };
const char LED_StripNames_Short[LED_NUM_STRIPS][4] = { "S01", "U", "V" };

const char LED_ColorNames_Long[LED_NUM_COLORS][6] = { "Red", "Green", "Blue" };
const char LED_ColorNames_Short[LED_NUM_COLORS][2] = { "R", "G", "B" };

const char LED_ConfigNames[LEDCONFIG_NUM][5] = { "Off", "On", "Low", "Mid", "High" };

/*
 * Output variables
 */

/*
 * Menu functions
 */

void LEDMenu_Init(LCD2004_I2C *lcd)
{
	// Start config with all off
	// TODO: save & load config for restarts
	for (uint8_t i = 0; i < LED_NUM_STRIPS; i++)
		for (uint8_t j = 0; j < LED_NUM_COLORS; j++)
			LEDStrips_Config[i][j] = LEDCONFIG_OFF;

	LEDSelectionMenu_State = LEDMENU_START;
	LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION] = 0;
	LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION] = 0;
	LEDSelectionMenu_Memory[LEDMENU_CONFIG_SELECTION] = 0;

	LEDMenu_StatusFlags = 0;

	LEDMenu_UpdateDisplay(lcd);
}

void LEDMenu_UpdateState(LEDMenu_ButtonAction action, LCD2004_I2C *lcd)
{
	switch (action)
	{
	case LEDMENU_BTN_BACK: // Go back to previous menu
	{
		switch (LEDSelectionMenu_State)
		{
		case LEDMENU_START:

			// cant go back from start state -> do nothing

			break;
		case LEDMENU_STRIP_SELECTION:
			LEDSelectionMenu_State = LEDMENU_START;
			break;
		case LEDMENU_COLOR_SELECTION:
			LEDSelectionMenu_State = LEDMENU_STRIP_SELECTION;
			break;
		case LEDMENU_CONFIG_SELECTION:
			LEDSelectionMenu_State = LEDMENU_COLOR_SELECTION;
			break;
		case LEDMENU_NUM:
		default:
			// default to start
			LEDSelectionMenu_State = LEDMENU_START;
			break;
		}

		LCD_Clear(lcd);

		break;
	}

	case LEDMENU_BTN_SELECT: // select current item, remember selection and go to next menu
	{
		switch (LEDSelectionMenu_State)
		{
		case LEDMENU_START:
		{
			LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION] = 0;
			LEDSelectionMenu_State = LEDMENU_STRIP_SELECTION;
			break;
		}

		case LEDMENU_STRIP_SELECTION:
		{
			LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION] = 0;
			LEDSelectionMenu_State = LEDMENU_COLOR_SELECTION;
			break;
		}

		case LEDMENU_COLOR_SELECTION:
		{
			//LEDSelectionMenu_Memory[LEDMENU_INPUT_SELECTION] = 0;
			LEDSelectionMenu_State = LEDMENU_CONFIG_SELECTION;
			break;
		}

		case LEDMENU_CONFIG_SELECTION:
		{
			// save selection result in led strip config
			uint8_t Selection_Strip = LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION];
			uint8_t Selection_Color = LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION];
			uint8_t Selection_Config = LEDSelectionMenu_Memory[LEDMENU_CONFIG_SELECTION];

			LEDStrips_Config[Selection_Strip][Selection_Color] = Selection_Config;
			LEDStrips_Update();

			LEDMenu_StatusFlags |= LEDMENU_FLAG_STATECHANGE;

			sprintf(USB_TxBuffer, "led config changed: strip %s %s -> %s\r\n", LED_StripNames_Long[Selection_Strip], LED_ColorNames_Long[Selection_Color], LED_ConfigNames[Selection_Config]);
			USB_PrintDebug(USB_TxBuffer);

			// output changed config
			LCD_Clear(lcd);
			LCD_DisplayStringLineCentered2(lcd, "New config:", 0);

			sprintf(lcd->printBuffer, "%s %s", LED_StripNames_Long[Selection_Strip], LED_ColorNames_Long[Selection_Color]);
			LCD_DisplayStringLineCentered2(lcd, lcd->printBuffer, 2);

			sprintf(lcd->printBuffer, "=> %s", LED_ConfigNames[Selection_Config]);
			LCD_DisplayStringLineCentered2(lcd, lcd->printBuffer, 3);

			// let update message stay on lcd for 5 sec, then discard any button events that happened during waiting time
			HAL_Delay(3000);
			Buttons_ResetFlags();
			LCD_TimeoutCounter = 0;

			LEDSelectionMenu_State = LEDMENU_COLOR_SELECTION;
			break;
		}
		case LEDMENU_NUM:
		default:
		{
			// default to start
			LEDSelectionMenu_State = LEDMENU_START;
			break;
		}
		}

		LCD_Clear(lcd);

		break;
	}

	case LEDMENU_BTN_NEXTITEM: // select next possible item in current menu
	{
		switch (LEDSelectionMenu_State)
		{
		case LEDMENU_START:

			// nothing to select

			break;

		case LEDMENU_STRIP_SELECTION:
		{
			LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION]++;

			if (LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION] >= LED_NUM_STRIPS)
				LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION] = 0;

			break;
		}

		case LEDMENU_COLOR_SELECTION:
		{
			LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION]++;

			if (LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION] >= LED_NUM_COLORS)
				LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION] = 0;

			break;
		}

		case LEDMENU_CONFIG_SELECTION:
		{
			LEDSelectionMenu_Memory[LEDMENU_CONFIG_SELECTION]++;

			if (LEDSelectionMenu_Memory[LEDMENU_CONFIG_SELECTION] >= LEDCONFIG_NUM)
				LEDSelectionMenu_Memory[LEDMENU_CONFIG_SELECTION] = 0;
			break;
		}

		case LEDMENU_NUM:
		default:
			// nothing to do / select
			break;
		}

		break;
	}

	case LEDMENU_BTN_NUM:
	default:
	{
		sprintf(USB_TxBuffer, "invalid btn action: %x\r\n", action);
		CDC_Transmit_FS((uint8_t*) USB_TxBuffer, strlen(USB_TxBuffer));
		break;
	}
	}

	LEDMenu_UpdateDisplay(lcd);
}

void LEDMenu_UpdateDisplay(LCD2004_I2C *lcd)
{

	switch (LEDSelectionMenu_State)
	{
	case LEDMENU_START:
		LCD_Clear(lcd);
		LCD_DisplayStringLineCentered2(lcd, "LED Config", 0);

		LCD_DisplayStringLineCentered2(lcd, "Press Select to", 2);
		LCD_DisplayStringLineCentered2(lcd, "start configuration", 3);
		break;

	case LEDMENU_STRIP_SELECTION:
		// print selection list

		LCD_DisplayStringLineCentered2(lcd, "Select Strip:", 0);

		for (uint8_t menuNr = 0; menuNr < 3; menuNr++)
		{
			int8_t stripNamesIndex = (LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION] + menuNr - 1 + LED_NUM_STRIPS) % LED_NUM_STRIPS;
			sprintf(lcd->printBuffer, "%s    ", LED_StripNames_Long[stripNamesIndex]);
			LCD_SetCursor(lcd, menuNr + 1, 6);
			LCD_DisplayString2(lcd, lcd->printBuffer);
		}

		LCD_SetCursor(lcd, 2, 2);
		LCD_DisplayString2(lcd, "->");

		break;

	case LEDMENU_COLOR_SELECTION:
		sprintf(lcd->printBuffer, "[%s] Select Color:", LED_StripNames_Short[LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION]]);
		LCD_DisplayStringLineCentered2(lcd, lcd->printBuffer, 0);

		for (uint8_t menuNr = 0; menuNr < 3; menuNr++)
		{
			int8_t colorNamesIndex = (LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION] + menuNr - 1 + LED_NUM_COLORS) % LED_NUM_COLORS;

			uint8_t stripIndex = LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION];

			sprintf(lcd->printBuffer, "%s     ", LED_ColorNames_Long[colorNamesIndex]);

			LCD_SetCursor(lcd, menuNr + 1, 8);
			LCD_DisplayString2(lcd, lcd->printBuffer);

			sprintf(lcd->printBuffer, "(%s)", LED_ConfigNames[LEDStrips_Config[stripIndex][colorNamesIndex]]);
			LCD_SetCursor(lcd, menuNr + 1, 20 - strlen(lcd->printBuffer));
			LCD_DisplayString2(lcd, lcd->printBuffer);
		}

		sprintf(lcd->printBuffer, "%s->", LED_StripNames_Short[LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION]]);
		LCD_SetCursor(lcd, 2, 0);
		LCD_DisplayString2(lcd, lcd->printBuffer);

		break;

	case LEDMENU_CONFIG_SELECTION:
		sprintf(lcd->printBuffer, "[%s] Select Input:", LED_StripNames_Short[LEDSelectionMenu_Memory[LEDMENU_STRIP_SELECTION]]);
		LCD_DisplayStringLineCentered2(lcd, lcd->printBuffer, 0);

		for (uint8_t menuNr = 0; menuNr < 3; menuNr++)
		{
			int8_t configNamesIndex = (LEDSelectionMenu_Memory[LEDMENU_CONFIG_SELECTION] + menuNr - 1 + LEDCONFIG_NUM) % LEDCONFIG_NUM;
			sprintf(lcd->printBuffer, "%s   ", LED_ConfigNames[configNamesIndex]);
			LCD_SetCursor(lcd, menuNr + 1, 8);
			LCD_DisplayString2(lcd, lcd->printBuffer);
		}

		sprintf(lcd->printBuffer, "%s->", LED_ColorNames_Long[LEDSelectionMenu_Memory[LEDMENU_COLOR_SELECTION]]);
		LCD_SetCursor(lcd, 2, 0);
		LCD_DisplayString2(lcd, lcd->printBuffer);

		break;

	case LEDMENU_NUM:
	default:
		// invalid menu title
		LCD_Clear(lcd);

		LCD_DisplayStringLineCentered2(lcd, "invalid state!", 1);
		LCD_DisplayStringLineCentered2(lcd, "press Select", 2);

		break;
	}
}

/*
 * Output functions
 */
static inline void LEDStrips_Update_StripColorCounter(uint8_t strip, uint8_t color, uint32_t newVal)
{
	*(LEDStrips_Config_Counter[strip][color]) = newVal;
}

void LEDStrips_Init()
{
	/*
	 * Strip 1
	 *
	 * R: TIM2 CH3
	 * G: TIM3 CH4
	 * B: TIM3 CH3
	 */
	LEDStrips_Config_Counter[0][0] = &TIM2->CCR3;
	LEDStrips_Config_Counter[0][1] = &TIM3->CCR4;
	LEDStrips_Config_Counter[0][2] = &TIM3->CCR3;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	/*
	 * Strip 2
	 *
	 * R: TIM2 CH1
	 * G: TIM2 CH2
	 * B: TIM3 CH1
	 */
	LEDStrips_Config_Counter[1][0] = &TIM2->CCR1;
	LEDStrips_Config_Counter[1][1] = &TIM2->CCR2;
	LEDStrips_Config_Counter[1][2] = &TIM3->CCR1;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	/*
	 * Strip 3
	 *
	 * R: TIM3 CH2
	 * G: TIM4 CH1
	 * B: TIM4 CH2
	 */
	LEDStrips_Config_Counter[2][0] = &TIM3->CCR2;
	LEDStrips_Config_Counter[2][1] = &TIM4->CCR1;
	LEDStrips_Config_Counter[2][2] = &TIM4->CCR2;

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	LEDStrips_Config[0][0] = LEDCONFIG_PWM1;
	LEDStrips_Config[0][1] = LEDCONFIG_PWM2;
	LEDStrips_Config[0][2] = LEDCONFIG_PWM3;

	LEDStrips_Update();
}

void LEDStrips_Update()
{
	for (uint8_t stripIndex = 0; stripIndex < LED_NUM_STRIPS; stripIndex++)
	{
		for (uint8_t colorIndex = 0; colorIndex < LED_NUM_COLORS; colorIndex++)
		{
			switch (LEDStrips_Config[stripIndex][colorIndex])
			{
			case LEDCONFIG_OFF:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, 0);
				break;
			case LEDCONFIG_ON:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, LEDSTRIPS_MAX_PWMDC);
				break;
			case LEDCONFIG_PWM1:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, LEDSTRIPS_LOW_PWMDC);
				break;
			case LEDCONFIG_PWM2:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, LEDSTRIPS_MID_PWMDC);
				break;
			case LEDCONFIG_PWM3:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, LEDSTRIPS_HIGH_PWMDC);
				break;
			case LEDCONFIG_NUM:
			default:
				break;
			}
		}
	}
}

void LEDStrips_UpdateFFT(uint16_t low, uint16_t mid, uint16_t high)
{
	for (uint8_t stripIndex = 0; stripIndex < LED_NUM_STRIPS; stripIndex++)
	{
		for (uint8_t colorIndex = 0; colorIndex < LED_NUM_COLORS; colorIndex++)
		{
			switch (LEDStrips_Config[stripIndex][colorIndex])
			{
			case LEDCONFIG_OFF:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, 0);
				break;
			case LEDCONFIG_ON:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, LEDSTRIPS_MAX_PWMDC);
				break;
			case LEDCONFIG_PWM1:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, low);
				break;
			case LEDCONFIG_PWM2:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, mid);
				break;
			case LEDCONFIG_PWM3:
				LEDStrips_Update_StripColorCounter(stripIndex, colorIndex, high);
				break;
			case LEDCONFIG_NUM:
			default:
				break;
			}
		}
	}
}
