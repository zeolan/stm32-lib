/*
 * ds18b20.h
 *
 *  Created on: Apr 28, 2025
 *      Author: Oleksandr Z (zeolan)
 */

#ifndef INC_DS18B20_DELAY_H_
#define INC_DS18B20_DELAY_H_

#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_gpio.h"

#define	DS18B20_SEARCH_ROM_CMD		0xF0
#define	DS18B20_READ_ROM_CMD		0x33
#define	DS18B20_MATCH_ROM_CMD		0x55
#define	DS18B20_SKIP_ROM_CMD		0xCC

#define	DS18B20_CONVERT_T_CMD		0x44

#define	DS18B20_ALARM_SEARCH_CMD	0xEC

#define	DS18B20_COPY_SCRPAD_CMD		0x48
#define	DS18B20_WRITE_SCRPAD_CMD	0x4E
#define	DS18B20_READ_SCRPAD_CMD		0xBE

#define	DS18B20_RECALL_E2_CMD		0xB8

#define	DS18B20_READ_POWER_CMD		0xB4

#define DS18B20_NO_ERROR			0
#define DS18B20_ERROR_NO_DEVICE		1

typedef struct {
	GPIO_TypeDef *GPIO;
	uint16_t GPIO_PIN;
	TIM_TypeDef *TIMER;
} DS18B20_TypeDef;

uint8_t ds_start(DS18B20_TypeDef);

void ds_send_command(DS18B20_TypeDef, uint8_t);

uint8_t ds_read_byte(DS18B20_TypeDef);

void ds_write_byte(DS18B20_TypeDef, uint8_t);

void ds_delay_us(DS18B20_TypeDef, uint16_t);

void ds_set_pin_output(DS18B20_TypeDef);

void ds_set_pin_input(DS18B20_TypeDef);

uint8_t DS18B20_start_t(DS18B20_TypeDef);

float DS18B20_read_t(DS18B20_TypeDef);

void DS18B20_read_rom(DS18B20_TypeDef, char*);

void DS18B20_read_scrpad(DS18B20_TypeDef, char *);

#endif /* INC_DS18B20_DELAY_H_ */
