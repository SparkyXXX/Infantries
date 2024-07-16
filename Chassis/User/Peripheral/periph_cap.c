/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-07-06 17:49:37
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
    cap->ui_state = 0;
    cap->last_update_time = HAL_GetTick();
}

void Cap_Update(void)
{
    Cap_DataTypeDef *cap = Cap_GetDataPtr();
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    Referee_DataTypeDef *referee = Referee_GetDataPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    cap->sum_power = boardcom->cap_power;
	cap->rest_energy = boardcom->cap_rest_energy;
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
    cap->last_update_time = HAL_GetTick();
}
