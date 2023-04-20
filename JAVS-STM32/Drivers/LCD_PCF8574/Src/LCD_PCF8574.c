/*
 * LCD_PCF8574.c
 *
 *  Created on: Apr 12, 2023
 *      Author: maxip

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
 *
 *		SPLC780D-01 (LCD Driver IC):
 *			MPU Interface:
 *				4 or 8 bit (here 4):
 *					=> two data transfers: first = DB7 - DB4, second = DB3 - DB0

 */

#include "LCD_PCF8574.h"
#include "main.h"
#include "i2c.h"


/*
 * LCD communication wrapper functions
 */

static inline void LCDSet4BitOperation(LCD2004_I2C *lcd)
{
	uint8_t i2cData = 0, dataBits = LCD_CMD_FunctionSet;

	// prepare non-data pins
	// RW & RS = 0 => nothing to be done

	// backlight
	i2cData |= (lcd->lcd_backlight & 1) << 3;

	// compose array
	uint8_t i2cDataArray[2];
	// 1: dataHigh Transmit
	i2cDataArray[0] = i2cData | (dataBits & 0xF0) | LCD_BIT_ENABLE;
	// 2: dataHigh latch
	i2cDataArray[1] = i2cData | (dataBits & 0xF0); // Enable 1->0 => Data latches

	// send array
	HAL_I2C_Master_Transmit(&hi2c1, lcd->i2cAddress, i2cDataArray, 2, 1000); //Sending in Blocking mode
}

static void SendLCDData(LCD2004_I2C *lcd, uint8_t dataBits, uint8_t RS, uint8_t RW)
{
	/* this orders data as MSB

	 PCF8574: send i2c addr -> P7...P0

	 P7: D7
	 P6: D6
	 P5: D5
	 P4: D4

	 P3: Backlight Active
	 P2: E -> Falling Edge = Data is latched
	 P1: RW
	 P0: RS


	 Have to send DB7-DB4 first, then DB3-DB0

	 */
	uint8_t i2cData = 0;

	// prepare non-data pins
	// RW & RS
	i2cData |= (RS & 1) | ((RW & 1) << 1);

	// backlight
	i2cData |= (lcd->lcd_backlight & 1) << 3;

	// compose array
	uint8_t i2cDataArray[4];
	// 1: dataHigh Transmit
	i2cDataArray[0] = i2cData | (dataBits & 0xF0) | LCD_BIT_ENABLE;
	// 2: dataHigh latch
	i2cDataArray[1] = i2cData | (dataBits & 0xF0); // Enable 1->0 => Data latches
	// 3: dataLow Transmit
	i2cDataArray[2] = i2cData | ((dataBits & 0x0F) << 4) | LCD_BIT_ENABLE;
	// 4: dataLow latch
	i2cDataArray[3] = i2cData | ((dataBits & 0x0F) << 4);

	// send array
	HAL_I2C_Master_Transmit(&hi2c1, lcd->i2cAddress, i2cDataArray, 4, 1000); //Sending in Blocking mode
}

static void SendLCDDataMultiple(LCD2004_I2C *lcd, uint8_t *data, uint8_t dataLen, uint8_t RS, uint8_t RW)
{
	uint8_t i2cData = 0;

	// prepare non-data pins
	// RW & RS
	i2cData |= (RS & 1) | ((RW & 1) << 1);

	// backlight
	i2cData |= (lcd->lcd_backlight & 1) << 3;

	uint8_t index, charsToSend;

	// compose array
	for (uint8_t j = 0; j < dataLen; j += LCD_BUFFER_NUMCHARS)
	{
		for (uint8_t i = 0; i < dataLen; i++)
		{
			index = j + i;
			// 1: dataHigh Transmit
			lcd->displayDataBuffer[4 * i + 0] = i2cData | (data[index] & 0xF0) | LCD_BIT_ENABLE;
			// 2: dataHigh latch
			lcd->displayDataBuffer[4 * i + 1] = i2cData | (data[index] & 0xF0); // Enable 1->0 => Data latches
			// 3: dataLow Transmit
			lcd->displayDataBuffer[4 * i + 2] = i2cData | ((data[index] & 0x0F) << 4) | LCD_BIT_ENABLE;
			// 4: dataLow latch
			lcd->displayDataBuffer[4 * i + 3] = i2cData | ((data[index] & 0x0F) << 4);
		}

		charsToSend = dataLen - j;
		if (charsToSend > LCD_BUFFER_NUMCHARS)
			charsToSend = LCD_BUFFER_NUMCHARS;

		// send array
		HAL_I2C_Master_Transmit(&hi2c1, lcd->i2cAddress, lcd->displayDataBuffer, charsToSend * 4, 1000); //Sending in Blocking mode
	}

}

static inline void SendLCDInstruction(LCD2004_I2C *lcd, uint8_t dataBits)
{
	SendLCDData(lcd, dataBits, 0, 0);
}

static inline void SendLCDDisplayData(LCD2004_I2C *lcd, uint8_t dataBits)
{
	SendLCDData(lcd, dataBits, 1, 0);
}

static inline void SendLCDDisplayCursorBlinkUpdate(LCD2004_I2C *lcd)
{
	SendLCDInstruction(lcd, LCD_CMD_DisplayOnOff | ((lcd->lcd_displayOnOff & 1) << 2) | ((lcd->lcd_cursorOnOff & 1) << 1) | (lcd->lcd_cursorBlink & 1));
}

/*
 * API functions
 */

void LCD_Init(LCD2004_I2C *lcd, uint8_t lcdI2CAddress)
{
	// init state variables
	lcd->i2cAddress = lcdI2CAddress;

	lcd->lcd_backlight = LCD_BACKLIGHT_OFF;
	lcd->lcd_displayOnOff = 1;
	lcd->lcd_cursorOnOff = 1;
	lcd->lcd_cursorBlink = 0;

	HAL_Delay(10);

	// set interface data length control bit to 4 bit data length
	LCDSet4BitOperation(lcd);

	// interface length = 4bit, 2-line display mode, 5x8 format display mode
	SendLCDInstruction(lcd, LCD_CMD_FunctionSet | (1 << 3) | (0 << 2));

	// display on, cursor on, cursor blink off
	SendLCDInstruction(lcd, LCD_CMD_DisplayOnOff | (1 << 2) | (1 << 1) | 0);

	// cursor: move to right, dont shift display
	SendLCDInstruction(lcd, LCD_CMD_EntryModeSet | (1 << 1) | 0);

	// clear display
	LCD_Clear(lcd);

	// display on
	LCD_BacklightOn(lcd);

	HAL_Delay(1);
}

void LCD_DisplayString1(LCD2004_I2C *lcd, char *string, uint8_t length)
{
	SendLCDDataMultiple(lcd, (uint8_t*) string, length, 1, 0);
}

void LCD_DisplayString2(LCD2004_I2C *lcd, const char *string)
{
	uint8_t strlen = 0;
	while (string[strlen] != 0 && strlen < 40)
		strlen++;

	LCD_DisplayString1(lcd, (char*) string, strlen);
}

void LCD_DisplayStringLine1(LCD2004_I2C *lcd, char *string, uint8_t length, uint8_t lineNr)
{
	LCD_SetCursor(lcd, lineNr, 0);
	LCD_DisplayString1(lcd, string, length);
}

void LCD_DisplayStringLine2(LCD2004_I2C *lcd, char *string, uint8_t lineNr)
{
	uint8_t strlen = 0;
	while (string[strlen] != 0 && strlen < 40)
		strlen++;

	LCD_DisplayStringLine1(lcd, string, strlen, lineNr);
}

void LCD_DisplayStringLineCentered1(LCD2004_I2C *lcd, char *string, uint8_t length, uint8_t lineNr)
{
	uint8_t offset = (20 - length) / 2;
	LCD_SetCursor(lcd, lineNr, 0);

	for(uint8_t i = 0; i < offset; i++)
		SendLCDDisplayData(lcd, ' ');

	LCD_DisplayString1(lcd, string, length);
}

void LCD_DisplayStringLineCentered2(LCD2004_I2C *lcd, char *string, uint8_t lineNr)
{
	uint8_t strlen = 0;
	while (string[strlen] != 0 && strlen < 40)
		strlen++;

	LCD_DisplayStringLineCentered1(lcd, string, strlen, lineNr);
}

void LCD_SetCursor(LCD2004_I2C *lcd, uint8_t row, uint8_t column)
{
	// row = lineNr 0-4
	// column 0-19

	if (row >= 4)
		row = 3;
	if (column >= 20)
		column = 19;

	switch (row)
	{
	case 0:
	default:
		SendLCDInstruction(lcd, LCD_CMD_SetDDRAMAddress | (0x00 + column));
		break;
	case 1:
		SendLCDInstruction(lcd, LCD_CMD_SetDDRAMAddress | (0x40 + column));
		break;
	case 2:
		SendLCDInstruction(lcd, LCD_CMD_SetDDRAMAddress | (20 + column));
		break;
	case 3:
		SendLCDInstruction(lcd, LCD_CMD_SetDDRAMAddress | (0x40 + 20 + column));
		break;
	}
}

void LCD_Test(LCD2004_I2C *lcd)
{
	SendLCDDisplayData(lcd, 'A');
}

// basic management functions

void LCD_Clear(LCD2004_I2C *lcd)
{
	SendLCDInstruction(lcd, LCD_CMD_ClearDisplay);
	HAL_Delay(2);
	//LCD_SetCursor(lcd, 0, 0);
}

void LCD_ReturnHome(LCD2004_I2C *lcd)
{
	SendLCDInstruction(lcd, LCD_CMD_ReturnHome);
}

void LCD_DisplayOn(LCD2004_I2C *lcd)
{
	if (!lcd->lcd_displayOnOff)
	{
		lcd->lcd_displayOnOff = 1;
		SendLCDDisplayCursorBlinkUpdate(lcd);
	}
}

void LCD_DisplayOff(LCD2004_I2C *lcd)
{
	if (lcd->lcd_displayOnOff)
	{
		lcd->lcd_displayOnOff = 0;
		SendLCDDisplayCursorBlinkUpdate(lcd);
	}
}

void LCD_CursorOn(LCD2004_I2C *lcd)
{
	if (!lcd->lcd_cursorOnOff)
	{
		lcd->lcd_cursorOnOff = 1;
		SendLCDDisplayCursorBlinkUpdate(lcd);
	}
}

void LCD_CursorOff(LCD2004_I2C *lcd)
{
	if (lcd->lcd_cursorOnOff)
	{
		lcd->lcd_cursorOnOff = 0;
		SendLCDDisplayCursorBlinkUpdate(lcd);
	}
}
void LCD_BacklightOn(LCD2004_I2C *lcd)
{
	if (lcd->lcd_backlight == LCD_BACKLIGHT_OFF)
	{
		lcd->lcd_backlight = LCD_BACKLIGHT_ON;
		SendLCDDisplayCursorBlinkUpdate(lcd);
	}
}

void LCD_BacklightOff(LCD2004_I2C *lcd)
{
	if (lcd->lcd_backlight == LCD_BACKLIGHT_ON)
	{
		lcd->lcd_backlight = LCD_BACKLIGHT_OFF;
		SendLCDDisplayCursorBlinkUpdate(lcd);
	}
}
