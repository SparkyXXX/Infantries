/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-06 00:46:29
 */

#include "periph_cap.h"
#include "app_chassis.h"
#include "protocol_board.h"
#include "protocol_referee.h"

Cap_DataTypeDef CapData;

Cap_DataTypeDef *Cap_GetDataPtr(void)
{
    return &CapData;
}

void Cap_Init(void)
{
    Cap_DataTypeDef *cap = Cap_GetDataPtr();
    cap->SD_flag = 0;
    cap->mode = 0;
    cap->cap_mode_starting = 0;
    cap->cap_mode_stall = 0;
    cap->ui_state = 0;
    cap->last_update_time = HAL_GetTick();
    cap->starting_time = 0;
}

void Cap_Update(void)
{
    Cap_DataTypeDef *cap = Cap_GetDataPtr();
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    Referee_DataTypeDef *referee = Referee_GetDataPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    cap->sum_power = boardcom->cap_power;
	cap->rest_energy = boardcom->cap_rest_energy;
    cap->voltage = boardcom->cap_voltage;
    cap->sum_current = boardcom->cap_current;
    cap->mode = boardcom->cap_mode_user;
	realtime_power_watch = cap->sum_power;
	rest_energy_watch = cap->rest_energy;
	if (referee->game_progress == 4) // 4表示比赛进行中
	{
		cap->SD_flag = 1;
	}
	else
	{
		cap->SD_flag = 0;
	}

	if (chassis->move_ref.forward_back_ref != 0 || chassis->move_ref.left_right_ref != 0 || chassis->move_ref.rotate_ref > 20)
	{
		boardcom->cap_mode_flag = 1;
	}
	else
	{
		boardcom->cap_mode_flag = 0;
	}
	boardcom->boost_mode_flag = cap->SD_flag;
    boardcom->power_limit = referee->chassis_power_limit;
	boardcom->power_level_limit = referee->chassis_power_level_limit;
    boardcom->power_buffer = referee->buffer_energy;
    boardcom->chassis_power = referee->chassis_power;
	cap->last_update_time = HAL_GetTick();
}
