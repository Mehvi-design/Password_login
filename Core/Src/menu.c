/*
 * menu.c
 *
 *  Created on: Jul 10, 2025
 *      Author: DELL
 */
#include<menu.h>
#include <ST7735.h>
#include<GFX_FUNCTIONS.h>
#include<stdint.h>

MenuItem menu[] = {
		{"TEST LCD",action_test_lcd},
		{"LED ON",action_led_on},
		{"LED OFF",action_led_off},
		{"LED TOGGLE",led_toggle}

};
const int NUM_OPTIONS =(sizeof(menu))/sizeof(MenuItem);


uint8_t brightness = 50;  // Default 50%


void action_test_lcd(){testAll();}

void draw_bulb_on(void) {
	fillScreen(BLACK);
    fillCircle(64, 40, 20, YELLOW);         // Bulb head
    ST7735_FillRectangle(60, 60, 8, 15, GRAY);          // Bulb base
    drawLine(64, 75, 64, 100, ORANGE);      // Glow line
}
//draw simple led off
void draw_bulb_off(void) {
    fillScreen(BLACK);
    fillCircle(64, 40, 20, GRAY);         // Bulb head
    ST7735_FillRectangle(60, 60, 8, 15, WHITE);          // Bulb base
    drawLine(64, 75, 64, 100, ORANGE);      // Glow line
}
void action_led_on(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // LED ON
	char msg[] = "LED turned ON\r\n";
	fillScreen(BLACK);
	draw_bulb_on();
	ST7735_WriteString(5, 90,msg, Font_11x18, YELLOW, BLACK);
}
void action_led_off(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // LED OFF
	char msg[] = "LED turned OFF\r\n";
	fillScreen(BLACK);
	draw_bulb_off();
	ST7735_WriteString(5, 90,msg, Font_11x18,CYAN, BLACK);
}
void led_toggle(){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	char msg[] = "LED Toggled\r\n";

	fillScreen(BLACK);

	ST7735_WriteString(5, 90,msg, Font_11x18, GREEN, BLACK);
}




void draw_menu(int selected) {
	fillScreen(BLACK);

    for (int i = 0; i < NUM_OPTIONS; i++) {
        uint16_t y = 10 + i * 20;
        if (i == selected) {
        	 ST7735_WriteString(10, y, "->", Font_11x18, YELLOW, BLACK);

        }
        ST7735_WriteString(30, y, menu[i].label, Font_11x18,//check if the string is selected, if selected make it yellow
                i == selected ? YELLOW : WHITE, BLACK);
    }

}



void set_backlight_brightness(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint32_t pulse = (htim1.Init.Period + 1) * percent / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);


}
void backlight_display(){
	  char b_str[8];  // Enough for "100%" + null terminator

	    // Easy conversion
	    b_str[0] = (brightness / 100) + '0';                 // Hundreds
	    b_str[1] = ((brightness / 10) % 10) + '0';           // Tens
	    b_str[2] = (brightness % 10) + '0';                  // Ones
	    b_str[3] = '%';                                      // Percent symbol
	    b_str[4] = '\0';                                     // Null-terminate
	    ST7735_WriteString(0, 100, "Bright: ", Font_11x18, WHITE, BLACK);
	    ST7735_WriteString(95, 100, b_str, Font_11x18, WHITE, BLACK);
}
