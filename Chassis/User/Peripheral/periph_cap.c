/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-25 20:29:14
 */

#include "periph_cap.h"
#include "app_chassis.h"
#include "protocol_board.h"
#include "protocol_referee.h"

Cap_DataTypeDef CapData;

const float Cap_Version_Table[4][3] = {
	{11.5f, 11.5f, 11.5f}, // buck_input_current_max
	{11.5f, 11.5f, 11.5f}, // buck_output_current_max
	{11.5f, 23.0f, 1.0f},  // boost_input_current_max
	{11.5f, 23.0f, 1.0f}   // boost_output_current_max
};

Cap_DataTypeDef *Cap_GetDataPtr(void)
{
	return &CapData;
}

void Cap_Init(void)
{
	Cap_DataTypeDef *cap = Cap_GetDataPtr();
	cap->SD_flag = 0;
	cap->ui_state = 0;
	cap->buck_version = 2;
	cap->boost_version = 2;
	cap->buck_input_current_max = Cap_Version_Table[0][cap->buck_version - 1];
	cap->buck_output_current_max = Cap_Version_Table[1][cap->buck_version - 1];
	cap->boost_input_current_max = Cap_Version_Table[2][cap->boost_version - 1];
	cap->boost_output_current_max = Cap_Version_Table[3][cap->boost_version - 1];
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
	cap->buck_version = boardcom->buck_version;
	cap->boost_version = boardcom->boost_version;
	cap->buck_input_current_max = Cap_Version_Table[0][cap->buck_version - 1];
	cap->buck_output_current_max = Cap_Version_Table[1][cap->buck_version - 1];
	cap->boost_input_current_max = Cap_Version_Table[2][cap->boost_version - 1];
	cap->boost_output_current_max = Cap_Version_Table[3][cap->boost_version - 1];
	if (referee->game_progress == 4)
	{
		cap->SD_flag = 1;
	}
	else
	{
		cap->SD_flag = 0;
	}
	if (chassis->chassis_coordinate_ref.vz != 0 || chassis->chassis_coordinate_ref.vx != 0 || chassis->chassis_coordinate_ref.w > 20)
	{
		boardcom->cap_mode_flag = 1;
	}
	else
	{
		boardcom->cap_mode_flag = 0;
	}
	cap->last_update_time = HAL_GetTick();
}
