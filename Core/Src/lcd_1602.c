/*
 * lcd.c
 *
 *  Created on: Apr 26, 2025
 *      Author: Oleksandr Z (zeolan)
 */

#include "lcd_1602.h"

const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};


/************************************** Static declarations **************************************/

static void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data);

static void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command);

static void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len);

/************************************** Function definitions **************************************/

/**

 * Create new Lcd_HandleTypeDef and initialize the Lcd

 */

Lcd_HandleTypeDef Lcd_create(
	Lcd_PortType port[], Lcd_PinType pin[],
	Lcd_PortType rs_port, Lcd_PinType rs_pin,
	Lcd_PortType en_port, Lcd_PinType en_pin,
	Lcd_ModeTypeDef mode
)
{
	Lcd_HandleTypeDef lcd;

	lcd.mode = mode;
	lcd.en_pin = en_pin;
	lcd.en_port = en_port;
	lcd.rs_pin = rs_pin;
	lcd.rs_port = rs_port;
	lcd.data_pin = pin;
	lcd.data_port = port;

	Lcd_init(&lcd);

	return lcd;
}

/**
 * Initialize 16x2-lcd without cursor
 */

void Lcd_init(Lcd_HandleTypeDef * lcd)
{
	if(lcd->mode == LCD_4_BIT_MODE) {

		lcd_write_command(lcd, 0x33);
		DELAY(5);
		lcd_write_command(lcd, 0x32);
		DELAY(1);

		lcd_write_command(lcd, FUNCTION_SET | OPT_4_BIT_MODE | OPT_2_LINES_MODE);
		lcd_write_command(lcd, CLEAR_DISPLAY);
		lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_ON /*| OPT_CURSOR_ON | OPT_BLINK_ON*/); // Lcd-on, cursor-off, no-blink
		lcd_write_command(lcd, ENTRY_MODE_SET | OPT_CURSOR_INCREMENT/* | OPT_ENABLE_SHIFT*/);
	}
	else {
		lcd_write_command(lcd, FUNCTION_SET | OPT_8_BIT_MODE | OPT_2_LINES_MODE);
		lcd_write_command(lcd, CLEAR_DISPLAY);
		lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_ON); // Lcd-on, cursor-off, no-blink
		lcd_write_command(lcd, ENTRY_MODE_SET | OPT_CURSOR_INCREMENT); // Increment cursor
	}
}

void Lcd_string(Lcd_HandleTypeDef * lcd, char * string) {
	for(uint8_t i = 0; i < strlen(string); i++) {
		lcd_write_data(lcd, string[i]);
	}
}

void Lcd_char(Lcd_HandleTypeDef * lcd, char ch) {
	lcd_write_data(lcd, ch);
}

/**
 * Set the cursor position
 */

void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col) {
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_16[row] + col);
}

void Lcd_clear(Lcd_HandleTypeDef * lcd) {
	lcd_write_command(lcd, CLEAR_DISPLAY);
}

void Lcd_shift_right(Lcd_HandleTypeDef * lcd) {
	lcd_write_command(lcd, CURSOR_DISPLAY_SHIFT | OPT_DISPLAY_SHIFT | OPT_RIGHT_SHIFT);
}

void Lcd_shift_left(Lcd_HandleTypeDef * lcd) {
	lcd_write_command(lcd, CURSOR_DISPLAY_SHIFT | OPT_DISPLAY_SHIFT | OPT_LEFT_SHIFT);
}

void Lcd_define_char(Lcd_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]) {
	lcd_write_command(lcd, SET_CGRAM_ADDR + (code << 3));

	for(uint8_t i=0;i<8;++i) {
		lcd_write_data(lcd, bitmap[i]);
	}
}

void Lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command){
	lcd_write_command(lcd, command);
}

/************************************** Static function definition **************************************/
/**
 * Write a byte to the command register
 */

void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command) {
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_COMMAND_REG); // Write to command register

	if(lcd->mode == LCD_4_BIT_MODE) {
		lcd_write(lcd, (command >> 4), LCD_NIB);
		lcd_write(lcd, command & 0x0F, LCD_NIB);
	} else {
		lcd_write(lcd, command, LCD_BYTE);
	}
}

/**
 * Write a byte to the data register
 */

void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data) {
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_DATA_REG); // Write to data register

	if(lcd->mode == LCD_4_BIT_MODE) {
		lcd_write(lcd, data >> 4, LCD_NIB);
		lcd_write(lcd, data & 0x0F, LCD_NIB);
	} else {
		lcd_write(lcd, data, LCD_BYTE);
	}
}

/**
 * Set len bits on the bus and toggle the enable line
 */

void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len) {
	for(uint8_t i = 0; i < len; i++) {
		HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 0x01);
	}
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 1);
	DELAY(1);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 0);
	DELAY(1);
}

//Code snippet to check LCD functions
//switch (keyPressed) {
//	case '1':					// Cursor ON / OFF
//		keyboard_toggle++;
//		keyboard_toggle % 2 ?
//				Lcd_write_command(&lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_ON | OPT_CURSOR_ON) :
//				Lcd_write_command(&lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_ON | OPT_CURSOR_OFF);
//		break;
//	case '2':					// Blink ON / OFF
//		keyboard_toggle++;
//		keyboard_toggle % 2 ?
//				Lcd_write_command(&lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_ON | OPT_CURSOR_OFF | OPT_BLINK_ON) :
//				Lcd_write_command(&lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_ON | OPT_CURSOR_OFF | OPT_BLINK_OFF);
//		break;
//	case '3':					// Display ON / OFF
//		keyboard_toggle++;
//		keyboard_toggle % 2 ?
//				Lcd_write_command(&lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_ON | OPT_CURSOR_OFF | OPT_BLINK_OFF) :
//				Lcd_write_command(&lcd, DISPLAY_ON_OFF_CONTROL | OPT_DISPLAY_OFF | OPT_CURSOR_OFF | OPT_BLINK_OFF);
//		break;
//	case '4':					// Cursor shift LEFT
//		Lcd_write_command(&lcd, CURSOR_LEFT_SHIFT);
//		break;
//	case '5':					// Cursor shift RIGHT
//		Lcd_write_command(&lcd, CURSOR_RIGHT_SHIFT);
//		break;
//	case '7':					// Display shift LEFT
//		Lcd_write_command(&lcd, DISPLAY_LEFT_SHIFT);
//		break;
//	case '8':					// Display shift RIGHT
//		Lcd_write_command(&lcd, DISPLAY_RIGHT_SHIFT);
//		break;
//	case '*':					// Show char
//		keyboard_toggle++;
//		Lcd_cursor(&lcd, 0, 0);
//		Lcd_clear(&lcd);
//		for (keyboard_toggle=0; keyboard_toggle<78; keyboard_toggle++) {
//			if (keyboard_toggle==39) Lcd_cursor(&lcd, 1, 0);
//			Lcd_char(&lcd, 0x20 + keyboard_toggle);
//			//Lcd_char(&lcd, 0xB0 + keyboard_toggle);	239 - grad sign
//		}
//		break;
//	default:
//		break;
//};
