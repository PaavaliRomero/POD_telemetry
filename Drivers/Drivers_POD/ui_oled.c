
#include <ui_oled.h>
#include <ssd1306_fonts.h>
#include <ssd1306.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pod_shared.h>

static uint32_t main_last_tick = 0;
static uint32_t anim_last_tick = 0; // for loading screen
static uint8_t anim_y = 0; //time by come back animation (Loading Screen)
static uint8_t anim_dir = 1;  // 1 = up, 0 = down

const uint8_t batt_h_25[20] = {
		/*
		 * status battery image: 25%
		 * */

	0x00, 0x00,  // ................
	0x7F, 0xF8,  // .############# .
	0x78, 0x0C,  // .#####.......#..
	0x78, 0x0E,  // .#####.......##.
	0x78, 0x0E,  // .#####.......##.
	0x78, 0x0E,  // .#####.......##.
	0x78, 0x0E,  // .#####.......##.
	0x78, 0x0C,  // .#####.......#..
	0x7F, 0xF8,  // .############# .
	0x00, 0x00   // ................
};

const uint8_t batt_h_50[20] = {
		/*
		 * status battery image: 50%
		 * */

	0x00, 0x00,  // ................
	0x7F, 0xF8,  // .############# .
	0x7E, 0x0C,  // .#######.....#..
	0x7E, 0x0E,  // .#######.....##.
	0x7E, 0x0E,  // .#######.....##.
	0x7E, 0x0E,  // .#######.....##.
	0x7E, 0x0E,  // .#######.....##.
	0x7E, 0x0C,  // .#######.....#..
	0x7F, 0xF8,  // .############# .
	0x00, 0x00   // ................
};

const uint8_t batt_h_75[20] = {
		/*
		 * status battery image: 75%
		 * */

	0x00, 0x00,  // ................
	0x7F, 0xF8,  // .############# .
	0x7F, 0x8C,  // .########...##..
	0x7F, 0x8E,  // .########...###.
	0x7F, 0x8E,  // .########...###.
	0x7F, 0x8E,  // .########...###.
	0x7F, 0x8E,  // .########...###.
	0x7F, 0x8C,  // .########...##..
	0x7F, 0xF8,  // .############# .
	0x00, 0x00   // ................
};

const uint8_t batt_h_100[20] = {
		/*
		 * status battery image: 100%
		 * */

	0x00, 0x00,  // ................
	0x7F, 0xF8,  // .############# .
	0x7F, 0xFC,  // .#############..
	0x7F, 0xFE,  // .##############.
	0x7F, 0xFE,  // .##############.
	0x7F, 0xFE,  // .##############.
	0x7F, 0xFE,  // .##############.
	0x7F, 0xFC,  // .#############..
	0x7F, 0xF8,  // .############# .
	0x00, 0x00   // ................
};

void Clear_Screen_ssd1306(void)
{
	/*
	 * Fill black screen.
	 * */
	  ssd1306_Fill(Black);
}
void Text_Screen_ssd1306(char myText[], uint8_t x_pos,uint8_t y_pos)
{
	/*
	 * Make a text.
	 * */
	Clear_Screen_ssd1306();
	ssd1306_SetCursor(x_pos, y_pos);
	ssd1306_WriteString(myText, Font_7x10, White);

}

void POD_UI_AnimaLoad_Task(void)
{
	/*ANIMATION
	 *Create a wave with bars thats effect of sea wave
	 *Its a initial system for loading all APIs of hardware
	 * */

	if (pod_state != POD_STATE_INIT)
		return;

    if (HAL_GetTick() - anim_last_tick < 40)
        return;   // Not draw yet

    anim_last_tick = HAL_GetTick();

    uint8_t x_init = 20;
    uint8_t x_endl = 27;
    uint8_t y_init = 10;
    uint8_t y_endl = 30;
    uint8_t sprt_btw_eyes = 10;
    uint8_t bars = 8;
    uint8_t ampli_sen_wave = 8;
    float phase_shift = 0.8f;
    float speed_wave = 0.2f; //<-- radians

    Clear_Screen_ssd1306();

    for(int i = 0; i < bars; i++)
    {
        int wave_offset = 5 + (int)(ampli_sen_wave * sin((anim_y * speed_wave + i * phase_shift)));

        //if(wave_offset < 1) wave_offset = 1;

        ssd1306_FillRectangle(
            x_init + sprt_btw_eyes * i,
            y_init + wave_offset,
            x_endl + sprt_btw_eyes * i,
            y_endl ,
            White
        );
    }

    ssd1306_UpdateScreen();

    // vertical drawing
    if (anim_dir) anim_y++;
    else anim_y--;

    if (anim_y > 40) anim_dir = 0;
    if (anim_y == 0) anim_dir = 1;
}

const uint8_t* POD_UI_SelectBattIcon(uint16_t V_Batt_indicator)
{
	/*
	 *	Check state of battery and send correct map of bits
	 *	prefer return maps than avoid checksums
	 * */


	if(V_Batt_indicator < 3200)
		return batt_h_25;
	else if (V_Batt_indicator < 3700)
		return batt_h_50;
	else if (V_Batt_indicator < 4000)
		return batt_h_75;
	else
		return batt_h_100;
}
uint8_t BATT_GetPercent(uint16_t vbatt_mv)
{
	if(vbatt_mv >= 4200) return 100;
	if(vbatt_mv <= 3000) return 0;

	return (uint8_t)((vbatt_mv - 3000) * 100/1200);
}

void POD_UI_MainScreen(void)
{
	/* MAIN SCREEN
	 * show state led
	 * show state batt
	 * show alerts
	 */

	if(pod_state_ui != POD_STATE_MainScreen)
		return;

	if(HAL_GetTick() - main_last_tick < 500)
		return;

	main_last_tick = HAL_GetTick();

	if(State_LED) Text_Screen_ssd1306("LED:ON",20,20);
	else Text_Screen_ssd1306("LED:OFF",20,20);

	char string_value[20];
	snprintf(string_value,sizeof(string_value),"%d",BATT_GetPercent(VBATT));
	ssd1306_SetCursor(80, 5);
	ssd1306_WriteString(string_value, Font_7x10, White);
	ssd1306_SetCursor(95, 5);
	ssd1306_WriteString("%",Font_7x10,White);

	const uint8_t *batt_icon = POD_UI_SelectBattIcon(VBATT);
	ssd1306_DrawBitmap(105, 5, batt_icon, 16, 10, White);

	ssd1306_UpdateScreen();
}

void POD_UI_FAULTSCREEN(void)
{
	/*
	 * SEND states indicator a fault of system
	 * */

	if(pod_state_ui != POD_STATE_FaultScreen)
		return;

	Clear_Screen_ssd1306();
	Text_Screen_ssd1306("ERROR",20,20);
}
