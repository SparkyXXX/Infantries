/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-23 00:21:52
 */

#ifndef APP_FUCK_UI_H
#define APP_FUCK_UI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "config_ctrl.h"
#include "protocol_board.h"
#include "periph_cap.h"
	
#define COLOR_RED 0x00
#define COLOR_BLUE 0x01

	typedef struct
	{
		uint16_t robot_id;
		uint16_t client_id;
		uint8_t rest_energy;
		float pitch;
		uint8_t mag_state;
		uint8_t gyro_state;
		uint8_t shooter_state;
		uint8_t auto_shoot_state;
		uint8_t cap_mode;
		uint8_t flyslope_flag;
		uint8_t homehurt;
		uint8_t fly_state;
		uint8_t is_get_target;
	} UI_DataTypeDef;
	
	typedef struct
	{
		uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;        
		uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
		
		uint16_t red_7_robot_HP_last;
        uint16_t red_outpost_HP_last;
        uint16_t red_base_HP_last;        
		uint16_t blue_7_robot_HP_last;
        uint16_t blue_outpost_HP_last;
        uint16_t blue_base_HP_last;
	} HP_DataTypeDef;

    UI_DataTypeDef *UI_GetDataPtr(void);
	HP_DataTypeDef *HP_GetDataPtr(void);
	void UI_Init();
	void UI_Update();
	void UI_Refresh();
	void HomeHurt_Detect();

#ifdef __cplusplus
}
#endif

#endif
