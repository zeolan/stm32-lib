/*
 * menu.h
 *
 *  Created on: May 13, 2025
 *      Author: Oleksandr Z (zeolan)
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "stm32f1xx_hal.h"

typedef struct MenuItem {
    const char *name;
    struct MenuItem *parent;
    struct MenuItem *children[3];
    uint8_t menu_index;
    uint8_t child_index;
} MenuItem;


void menuNavigateUp();

void menuNavigateDown(int);

void menuNavigateLeft();

void menuNavigateRight();

void menuDisplayMenu(MenuItem *);

#endif /* INC_MENU_H_ */
