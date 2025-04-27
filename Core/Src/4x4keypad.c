/*
 * 4x4keypad.c
 *
 *  Created on: Apr 26, 2025
 *      Author: Oleksandr Z (zeolan)
 */


#include "4x4keypad.h"

GPIO_TypeDef* COL_GPIOS[4] = {COL1_GPIO, COL2_GPIO, COL3_GPIO, COL4_GPIO};
uint16_t COL_PINS[4] = {COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN};

GPIO_TypeDef* ROW_GPIOS[4] = {ROW1_GPIO, ROW2_GPIO, ROW3_GPIO, ROW4_GPIO};
uint16_t ROW_PINS[4] = {ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN};

char keyMap[4][4] = {
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
};

char KEYPAD4x4_GetPressedKey (uint16_t GPIO_Pin) {

	char keyPressed = ' ';
	uint8_t keyFound = 0;

	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};

	GPIO_InitStructPrivate.Pin = ROW1_PIN|ROW2_PIN|ROW3_PIN|ROW4_PIN;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ROW1_GPIO, &GPIO_InitStructPrivate);

	for (uint8_t col = 0; col < 4; col++) {
		if (col > 0) {
			HAL_GPIO_WritePin(COL_GPIOS[col - 1], COL_PINS[col - 1], GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(COL_GPIOS[col], COL_PINS[col], GPIO_PIN_SET);

		for (uint8_t row = 0; row < 4; row++) {
			if(GPIO_Pin == ROW_PINS[row] && HAL_GPIO_ReadPin(ROW_GPIOS[row], ROW_PINS[row]) == GPIO_PIN_SET) {
				keyPressed = keyMap[row][col];
				keyFound = 1;
				break;
			}
		}

		if (keyFound) {
			break;
		}
	}

	HAL_GPIO_WritePin(COL1_GPIO, COL1_PIN, 0);
	HAL_GPIO_WritePin(COL2_GPIO, COL2_PIN, 0);
	HAL_GPIO_WritePin(COL3_GPIO, COL3_PIN, 0);
	HAL_GPIO_WritePin(COL4_GPIO, COL4_PIN, 0);
	GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ROW1_GPIO, &GPIO_InitStructPrivate);

	return keyPressed;
}
