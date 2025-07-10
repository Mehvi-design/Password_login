/*
 * menu.h
 *
 *  Created on: Jul 10, 2025
 *      Author: DELL
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include<stdint.h>
#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern uint8_t brightness;
extern int selectted;

typedef void (*MenuAction)(void);
//It's a pointer to a function.
//The function takes no arguments ((void)).
//The function returns nothing (void).
typedef struct{
 const char *label;
 MenuAction action;
}MenuItem;

void action_test_lcd();


// Draw a simple light bulb ON
void draw_bulb_on(void);
//draw simple led off
void draw_bulb_off(void);
void action_led_on();
void action_led_off();
void led_toggle();
extern MenuItem menu[];
extern const int NUM_OPTIONS ;      // Declare length variable



void draw_menu(int selected);



void set_backlight_brightness(uint8_t percent) ;
void backlight_display();
#endif /* INC_MENU_H_ */
