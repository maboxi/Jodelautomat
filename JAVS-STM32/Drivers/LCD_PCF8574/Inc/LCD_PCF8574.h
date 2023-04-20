/*
 * LCD_PCF8574.h
 *
 *  Created on: Apr 12, 2023
 *      Author: maxip
 *
 *    	LCD Module 20x04 \w I2C:  https://www.handsontec.com/dataspecs/I2C_2004_LCD.pdf
 *    	LCD Module 20x04: https://uk.beta-layout.com/download/rk/RK-10290_410.pdf
 *		PCF8574: https://www.mouser.de/datasheet/2/302/PCF8574_PCF8574A-1127673.pdf
 *		SPLC780D-01: file:///C:/Users/maxip/Downloads/SPLC780D_v0.1.pdf
 *
 *		PCF8574:
 *			P0-P7:
 *				default high (internal pullup)
 *
 *				connections:
 *					P0 -> RS: 1 = Display Data; 0 = Display Instruction
 *					P1 -> RW: 1 = Data Read (LCD->); 0 = Data Write (->LCD)
 *					P2 -> E:  	RW = 1 (Data Read)  -> Reading Data appears on D0-D7 while E = 1
 *								RW = 0 (Data Write) -> Data is latched at falling edge of E (=> 0)
 *								Cycle Time: Min 1000ns; Pulse Width: Min 450ns
 *					P3 -> K': 1 = Transistor Active, Backlight on; 0 = Transistor Inactive, Backlight Off
 *					P4 -> D4: Data Bus
 *					P5 -> D5: Data Bus
 *					P6 -> D6: Data Bus
 *					P7 -> D7: Data Bus
 *
 *		LCD 2004A:
 *			Timings Read Mode:
 *				E Cycle Time: Min 1000ns
 *				E Pulse Width: Min 450ns
 *				Address Setup Time: min 60ns
 *				Address Hold Time: min 20ns
 *				Data Setup Time: min 195ns
 *				Data Hold Time: min 10ns
 *			Timings Write Mode:
 *				E Cycle Time: Min 1000ns
 *				E Pulse Width: Min 450ns
 *				Address Setup Time: min 60ns
 *				Address Hold Time: min 20ns
 *				Data Output Delay Time: max 360ns
 *				Data Hold Time: min 5ns
 *			Power Consumption:
 *				LCD Backlight: 5V 30mA
 *
 *		SPLC780D-01 (LCD Driver IC):
 *			MPU Interface:
 *				4 or 8 bit (here 4):
 *					=> two data transfers: first = DB7 - DB4, second = DB3 - DB0
 */

#ifndef LCD_PCF8574_INC_LCD_PCF8574_H_
#define LCD_PCF8574_INC_LCD_PCF8574_H_

#include "stdint.h"

#define LCD_BIT_ENABLE (1<<2)

// LCD 2004A Instruction Set
#define LCD_CMD_ClearDisplay 	(0b00000001)
#define LCD_CMD_ReturnHome 		(0b00000010)
#define LCD_CMD_EntryModeSet	(0b00000100)
#define LCD_CMD_DisplayOnOff	(0b00001000)
#define LCD_CMD_CursorDispShift	(0b00010000)
#define LCD_CMD_FunctionSet		(0b00100000)
#define LCD_CMD_SetCGRAMAddress	(0b01000000)
#define LCD_CMD_SetDDRAMAddress	(0b10000000)

#define LCD_BUFFER_NUMCHARS 20

#define LCD_DEFAULT_ADDR (0x27 << 1) // i2c address without read / write bit; R/W Bit: 1 = Read, 0 = Write (Default)

#define LCD_BACKLIGHT_OFF 0
#define LCD_BACKLIGHT_ON 1

/*
 * LCD struct definition
 */

struct LCD2004_I2C_t
{
	uint8_t i2cAddress;
	uint8_t lcd_backlight, lcd_displayOnOff, lcd_cursorOnOff, lcd_cursorBlink;
	uint8_t displayDataBuffer[LCD_BUFFER_NUMCHARS * 4];
	char printBuffer[LCD_BUFFER_NUMCHARS + 1];
};
typedef struct LCD2004_I2C_t LCD2004_I2C;


void LCD_Init(LCD2004_I2C *lcd, uint8_t lcdI2CAddress);
void LCD_Clear(LCD2004_I2C *lcd);
void LCD_ReturnHome(LCD2004_I2C *lcd);
void LCD_DisplayOn(LCD2004_I2C *lcd);
void LCD_DisplayOff(LCD2004_I2C *lcd);
void LCD_CursorOn(LCD2004_I2C *lcd);
void LCD_CursorOff(LCD2004_I2C *lcd);
void LCD_BacklightOn(LCD2004_I2C *lcd);
void LCD_BacklightOff(LCD2004_I2C *lcd);

void LCD_Test(LCD2004_I2C *lcd);

void LCD_DisplayString1(LCD2004_I2C *lcd, char *string, uint8_t length);
void LCD_DisplayString2(LCD2004_I2C *lcd, const char *string);
void LCD_DisplayStringLine1(LCD2004_I2C *lcd, char *string, uint8_t length, uint8_t lineNr);
void LCD_DisplayStringLine2(LCD2004_I2C *lcd, char *string, uint8_t lineNr);
void LCD_DisplayStringLineCentered1(LCD2004_I2C *lcd, char *string, uint8_t length, uint8_t lineNr);
void LCD_DisplayStringLineCentered2(LCD2004_I2C *lcd, char *string, uint8_t lineNr);

void LCD_SetCursor(LCD2004_I2C *lcd, uint8_t row, uint8_t column);

#endif /* LCD_PCF8574_INC_LCD_PCF8574_H_ */
