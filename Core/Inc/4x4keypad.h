/*
 * 4x4keypad.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Oleksandr Z (zeolan)
 */

#include "stm32f1xx_hal.h"

#ifndef INC_4X4KEYPAD_H_
#define INC_4X4KEYPAD_H_

#define	COL1_GPIO	GPIOC
#define	COL2_GPIO	GPIOC
#define	COL3_GPIO	GPIOC
#define	COL4_GPIO	GPIOC
#define	COL1_PIN	GPIO_PIN_3
#define	COL2_PIN	GPIO_PIN_2
#define	COL3_PIN	GPIO_PIN_1
#define	COL4_PIN	GPIO_PIN_0

#define	ROW1_GPIO	GPIOA
#define	ROW2_GPIO	GPIOA
#define	ROW3_GPIO	GPIOA
#define	ROW4_GPIO	GPIOA
#define	ROW1_PIN	GPIO_PIN_3
#define	ROW2_PIN	GPIO_PIN_2
#define	ROW3_PIN	GPIO_PIN_1
#define	ROW4_PIN	GPIO_PIN_0

#define KEYPAD_DEBOUNCE	500u

char KEYPAD4x4_GetPressedKey (uint16_t);

#endif /* INC_4X4KEYPAD_H_ */
