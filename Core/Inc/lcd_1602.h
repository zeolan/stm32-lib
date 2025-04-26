/*
 * lcd.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Oleksandr Z (zeolan)
 */

#ifndef LCD_H_

#define LCD_H_

#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "main.h"

// For row start addresses
extern const uint8_t ROW_16[];

/************************************* Commands  **************************************/

#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x04			// Entry mode group:
#define OPT_ENABLE_SHIFT 0x01			// Enable display shift
#define OPT_CURSOR_INCREMENT 0x02		// Cursor increment

#define DISPLAY_ON_OFF_CONTROL 0x08	// Display control group:
#define OPT_DISPLAY_ON 0x04				// Turn on display
#define OPT_CURSOR_ON 0x02				// Turn on cursor
#define OPT_CURSOR_OFF 0x00				// Turn off cursor
#define OPT_BLINK_ON 0x01				// Turn on cursor blink
#define OPT_BLINK_OFF 0x00				// Turn off cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10	// Move and shift cursor
#define OPT_DISPLAY_SHIFT 0x08
#define OPT_CURSOR_SHIFT 0x00
#define OPT_RIGHT_SHIFT 0x04
#define OPT_LEFT_SHIFT 0x00

#define FUNCTION_SET 0x20			// Function set group:
#define OPT_8_BIT_MODE 0x10 			// Set interface data length 8-bit
#define OPT_4_BIT_MODE 0x00 			// Set interface data length 4-bit
#define OPT_2_LINES_MODE 0x08 			// Set number of display lines 2
#define OPT_1_LINES_MODE 0x00 			// Set number of display lines 1
#define OPT_5_10_DOTS_FONT 0x04 		// Set alternate font 5x10 dots

#define SETCGRAM_ADDR 0x040			// Set CGRAM address group:

#define SET_DDRAM_ADDR 0x80			// Set DDRAM address group:

/************************************** Helper macros **************************************/

#define DELAY(X) HAL_Delay(X)

/************************************** LCD defines **************************************/

#define LCD_NIB 4
#define LCD_BYTE 8

#define LCD_DATA_REG 1
#define LCD_COMMAND_REG 0

/************************************** LCD typedefs **************************************/

#define Lcd_PortType GPIO_TypeDef*

#define Lcd_PinType uint16_t

typedef enum {
	LCD_4_BIT_MODE,
	LCD_8_BIT_MODE
} Lcd_ModeTypeDef;

typedef struct {
	Lcd_PortType * data_port;
	Lcd_PinType * data_pin;
	Lcd_PortType rs_port;
	Lcd_PinType rs_pin;
	Lcd_PortType en_port;
	Lcd_PinType en_pin;
	Lcd_ModeTypeDef mode;
} Lcd_HandleTypeDef;

/************************************** Public functions **************************************/

void Lcd_init(Lcd_HandleTypeDef * lcd);
void Lcd_string(Lcd_HandleTypeDef * lcd, char * string);
void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col);

Lcd_HandleTypeDef Lcd_create(
	Lcd_PortType port[], Lcd_PinType pin[],
	Lcd_PortType rs_port, Lcd_PinType rs_pin,
	Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode
);

void Lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command);
void Lcd_define_char(Lcd_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]);
void Lcd_clear(Lcd_HandleTypeDef * lcd);
void Lcd_shift_right(Lcd_HandleTypeDef * lcd);
void Lcd_shift_left(Lcd_HandleTypeDef * lcd);

#endif /* LCD_H_ */
