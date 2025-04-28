/*
 * ds18b20_dela.c
 *
 *  Created on: Apr 28, 2025
 *      Author: Oleksandr Z (zeolan)
 */

#include <ds18b20_delay.h>

uint8_t ds_start(DS18B20_TypeDef ds) {
	uint8_t value = DS18B20_NO_ERROR;
	ds_set_pin_output(ds);							// set as output
	HAL_GPIO_WritePin(ds.GPIO, ds.GPIO_PIN, 0);		// pull the data pin LOW
	ds_delay_us(ds, 480);  							// wait for 2 us
	ds_set_pin_input(ds);							// set as input
	ds_delay_us(ds, 80);
	if (HAL_GPIO_ReadPin(ds.GPIO, ds.GPIO_PIN) == GPIO_PIN_SET) {
		value = DS18B20_ERROR_NO_DEVICE;
	}
	ds_delay_us(ds, 400);
	return value;
}

void ds_send_command(DS18B20_TypeDef ds, uint8_t command) {
	ds_write_byte(ds, command);
}

uint8_t ds_read_byte(DS18B20_TypeDef ds) {
	uint8_t value = 0;
	ds_set_pin_input(ds);

	for (int i = 0; i < 8; i++) {
		ds_set_pin_output(ds);							// set as output
		HAL_GPIO_WritePin(ds.GPIO, ds.GPIO_PIN, 0);		// pull the data pin LOW
		ds_delay_us(ds, 2);  							// wait for 2 us
		ds_set_pin_input(ds);							// set as input
		if (HAL_GPIO_ReadPin(ds.GPIO, ds.GPIO_PIN))		// if the pin is HIGH
				{
			value |= 1 << i;							// read = 1
		}
		ds_delay_us(ds, 60);							// wait for 60 us
	}
	return value;
}

void ds_write_byte(DS18B20_TypeDef ds, uint8_t data) {
	ds_set_pin_output(ds);
	for (int i = 0; i < 8; i++) {
		if ((data & (1 << i)) != 0)  // if the bit is high
				{
			// write 1
			ds_set_pin_output(ds);							// set as output
			HAL_GPIO_WritePin(ds.GPIO, ds.GPIO_PIN, 0);		// pull the pin LOW
			ds_delay_us(ds, 1);								// wait for 1 us

			ds_set_pin_input(ds);							// set as input
			ds_delay_us(ds, 50);							// wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			ds_set_pin_output(ds);
			HAL_GPIO_WritePin(ds.GPIO, ds.GPIO_PIN, 0);	// pull the pin LOW
			ds_delay_us(ds, 50);							// wait for 60 us

			ds_set_pin_input(ds);
		}
	}
}

void ds_delay_us(DS18B20_TypeDef ds, uint16_t delay_us) {
	ds.TIMER->CNT = 0;
	while (ds.TIMER->CNT < delay_us)
		;
}

void ds_set_pin_output(DS18B20_TypeDef ds) {
//	LL_GPIO_SetPinMode(ds.GPIO, (uint32_t) ds.GPIO_PIN, LL_GPIO_MODE_OUTPUT);
	GPIO_InitTypeDef GPIO_InitStructPrivate = { 0 };
	GPIO_InitStructPrivate.Pin = ds.GPIO_PIN;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ds.GPIO, &GPIO_InitStructPrivate);
}

void ds_set_pin_input(DS18B20_TypeDef ds) {
//	LL_GPIO_SetPinMode(ds.GPIO, (uint32_t) ds.GPIO_PIN, LL_GPIO_MODE_INPUT);
	GPIO_InitTypeDef GPIO_InitStructPrivate = { 0 };
	GPIO_InitStructPrivate.Pin = ds.GPIO_PIN;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ds.GPIO, &GPIO_InitStructPrivate);
}

uint8_t DS18B20_start_t(DS18B20_TypeDef ds) {
	uint8_t ret_code = ds_start(ds);
	if (ret_code != DS18B20_NO_ERROR) {
		return ret_code;
	}
	ds_send_command(ds, DS18B20_SKIP_ROM_CMD);
	ds_send_command(ds, DS18B20_CONVERT_T_CMD);

	return ret_code;
}

float DS18B20_read_t(DS18B20_TypeDef ds) {
	uint8_t t_LSB, t_MSB;
	uint16_t t_16;
	uint8_t ret_code = ds_start(ds);
	if (ret_code != DS18B20_NO_ERROR) {
		return ret_code;
	}
	ds_send_command(ds, DS18B20_SKIP_ROM_CMD);
	ds_send_command(ds, DS18B20_READ_SCRPAD_CMD);

	t_LSB = ds_read_byte(ds);
	t_MSB = ds_read_byte(ds);
	t_16 = (t_MSB << 8) | t_LSB;
	return (float) t_16 / 16.0;
}

void DS18B20_read_rom(DS18B20_TypeDef ds, char *buf) {
	uint8_t ds_read_data = 0;
	ds_start(ds);
//	uint8_t ret_code =
//	if (ret_code != DS18B20_NO_ERROR) {
//		return ret_code;
//	}
	ds_send_command(ds, DS18B20_READ_ROM_CMD);
	for (int i = 0; i < 8; i++) {
		ds_read_data = ds_read_byte(ds);
		sprintf(buf, "%02x", ds_read_data);
		buf++;
		buf++;
		//28e82bd0020000
	}
}

void DS18B20_read_scrpad(DS18B20_TypeDef ds, char *buf) {
	uint8_t ds_read_data = 0;
	ds_start(ds);
//	if (ret_code != DS18B20_NO_ERROR) {
//		return ret_code;
//	}
	ds_send_command(ds, DS18B20_SKIP_ROM_CMD);
	ds_send_command(ds, DS18B20_READ_SCRPAD_CMD);
	for (int i = 0; i < 8; i++) {
		ds_read_data = ds_read_byte(ds);
		sprintf(buf, "%02x", ds_read_data);
		buf++;
		buf++;
	}
}
