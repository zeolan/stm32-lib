/*
 * menu.c
 *
 *  Created on: May 13, 2025
 *      Author: Oleksandr Z (zeolan)
 */

#include <stddef.h>

#include "menu.h"

#include "lcd_1602.h"


MenuItem mainMenu, item1, item2, subItem1, subItem2, subItem3;


MenuItem mainMenu = { "    Main Menu   ", NULL, {&item1, &item2}, 0, 0 };
MenuItem item1 = { "    Settings    ", &mainMenu, {&subItem1, &subItem2, &subItem3}, 1, 0 };
MenuItem item2 = { "      Info      ", &mainMenu, {NULL}, 2, 0 };
MenuItem subItem1 = { "   Brightness   ", &item1, {NULL}, 3, 0 };
MenuItem subItem2 = { "      Sound     ", &item1, {NULL}, 4, 0 };
MenuItem subItem3 = { "      Date      ", &item1, {NULL}, 5, 0 };

MenuItem *currentMenu = &mainMenu;

extern Lcd_HandleTypeDef lcd;


void menuNavigateUp() {
    if (currentMenu->parent) {
    	currentMenu->child_index = 0;
    	currentMenu = currentMenu->parent;
    }
}

void menuNavigateDown(int index) {
    if (currentMenu->children[index]) currentMenu = currentMenu->children[index];
}

void menuNavigateLeft() {
    if (currentMenu->children[currentMenu->child_index]) {
    	if (currentMenu->child_index > 0) {
    		currentMenu->child_index--;
    	} else {
    		currentMenu->child_index = sizeof(currentMenu->children) / sizeof(currentMenu->children[0]) - 1;
    	}
    }
}

void menuNavigateRight() {
    if (currentMenu->children[currentMenu->child_index]) {
    	if ((currentMenu->child_index + 1) < sizeof(currentMenu->children) / sizeof(currentMenu->children[0])) {
    		currentMenu->child_index++;
    	} else {
    		currentMenu->child_index = 0;
    	}
    }
}

void menuDisplayMenu(MenuItem *menu) {
	//Lcd_clear(&lcd);
	Lcd_cursor(&lcd, 0, 0);
//	char first_line[16];
//	sprintf(first_line, "<%s>", menu->name);
//	Lcd_string(&lcd, (char *)first_line );
	Lcd_string(&lcd, (char *)menu->name );

    if (menu->children[0]) {
    	Lcd_cursor(&lcd, 1, 0);
    	Lcd_string(&lcd, (char *)menu->children[menu->child_index]->name);
    }
}
