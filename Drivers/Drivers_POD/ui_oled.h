
#ifndef UI_OLED_H
#define UI_OLED_H

#include <stdlib.h>
#include <stdint.h>

extern const uint8_t batt_h_25[];
extern const uint8_t batt_h_50[];
extern const uint8_t batt_h_75[];
extern const uint8_t batt_h_100[];

void Clear_Screen_ssd1306(void);
void Text_Screen_ssd1306(char myText[], uint8_t x_pos,uint8_t y_pos);
void POD_UI_AnimaLoad_Task();
const uint8_t* POD_UI_SelectBattIcon(uint16_t V_Batt_indicator);
void POD_UI_MainScreen();
void POD_UI_FAULTSCREEN(void);

#endif
