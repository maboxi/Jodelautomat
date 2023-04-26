/*
 * LEDConfigMenu.h
 *
 *  Created on: 17.04.2023
 *      Author: maxip
 */

#ifndef INC_LEDCONFIGMENU_H_
#define INC_LEDCONFIGMENU_H_

#include "LCD_PCF8574.h"

#define LED_NUM_STRIPS 3
#define LED_NUM_COLORS 3

/*
 * Config enums
 */

typedef enum
{
	LEDCONFIG_OFF,
	LEDCONFIG_ON,
	LEDCONFIG_PWM1,
	LEDCONFIG_PWM2,
	LEDCONFIG_PWM3,
	LEDCONFIG_NUM
} LEDMENU_Config;

typedef enum
{
	LEDMENU_START,
	LEDMENU_STRIP_SELECTION,
	LEDMENU_COLOR_SELECTION,
	LEDMENU_CONFIG_SELECTION,
	LEDMENU_NUM
} LEDMENU_State;

typedef enum
{
	LEDMENU_BTN_BACK,
	LEDMENU_BTN_SELECT,
	LEDMENU_BTN_NEXTITEM,
	LEDMENU_BTN_NUM
} LEDMenu_ButtonAction;


/*
 * Menu variables
 */

#define LEDMENU_FLAG_STATECHANGE 1
extern uint8_t LEDMenu_StatusFlags;

/*
 * Output variables
 */
#define LEDSTRIPS_MIN_PWMDC 0
#define LEDSTRIPS_MAX_PWMDC 1000
#define LEDSTRIPS_LOW_PWMDC 10
#define LEDSTRIPS_MID_PWMDC 200
#define LEDSTRIPS_HIGH_PWMDC 750


/*
 * Menu function prototypes
 */
void LEDMenu_Init(LCD2004_I2C *lcd);
void LEDMenu_UpdateState(LEDMenu_ButtonAction action, LCD2004_I2C *lcd);
void LEDMenu_UpdateDisplay(LCD2004_I2C *lcd);

/*
 * Output function prototypes
 */
void LEDStrips_Init();
void LEDStrips_Update();
void LEDStrips_UpdateFFT(uint16_t low, uint16_t mid, uint16_t high);



#endif /* INC_LEDCONFIGMENU_H_ */
