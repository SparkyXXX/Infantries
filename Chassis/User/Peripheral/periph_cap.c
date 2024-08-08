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
#include "app_power.h"
#include "protocol_board.h"
#include "protocol_referee.h"

Cap_DataTypeDef CapData;

Cap_DataTypeDef *Cap_GetDataPtr(void)
{
    return &CapData;
}

const float Cap_Version_Table[4][3] = {
	{11.5f,11.5f,11.5f},	//BUCK_INPUT_CURRENT_MAX
	{11.5f,11.5f,11.5f},	//BUCK_OUTPUT_CURRENT_MAX
	{11.5f,23.0f,1.0f},		//BOOST_INPUT_CURRENT_MAX
	{11.5f,23.0f,1.0f}		//BOOST_OUTPUT_CURRENT_MAX
};//lzq

//#define BUCK_V1_1					1 		//8 MOSFET 无防倒灌电路 输出TMCS1100A的参考电压1.8V
//#define BUCK_V1_2					2 		//8 MOSFET 无防倒灌电路 输出TMCS1100A的参考电压1.25V
//#define BUCK_V2						3			//4 MOSFET 有防倒灌电路 //在另一版代码中
//#define BOOST_V1					1			//单3790 无散热片
//#define BOOST_V2					2			//并联3790 有散热片

void Cap_Init(void)
{
	Cap_DataTypeDef *cap = Cap_GetDataPtr();
	cap->boost_mode = 0;
	cap->SD_flag = 0;
	cap->mode = 0;
	cap->cap_mode_starting = 0;
	cap->cap_mode_stall = 0;
	cap->ui_state = 0;
	cap->rest_energy = 40;
	cap->last_update_time = HAL_GetTick();
	cap->starting_time = 0;
	cap->BUCK_VERSION = 2;//lzq
	cap->BOOST_VERSION = 2;
	cap->BUCK_INPUT_CURRENT_MAX = Cap_Version_Table[0][cap->BUCK_VERSION - 1];
	cap->BUCK_OUTPUT_CURRENT_MAX = Cap_Version_Table[1][cap->BUCK_VERSION - 1];
	cap->BOOST_INPUT_CURRENT_MAX = Cap_Version_Table[2][cap->BOOST_VERSION - 1];
	cap->BOOST_OUTPUT_CURRENT_MAX = Cap_Version_Table[3][cap->BOOST_VERSION - 1];
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
	
	cap->BUCK_VERSION = boardcom->buck_version;//lzq
	cap->BOOST_VERSION = boardcom->boost_version;
	cap->BUCK_INPUT_CURRENT_MAX = Cap_Version_Table[0][cap->BUCK_VERSION - 1];
	cap->BUCK_OUTPUT_CURRENT_MAX = Cap_Version_Table[1][cap->BUCK_VERSION - 1];
	cap->BOOST_INPUT_CURRENT_MAX = Cap_Version_Table[2][cap->BOOST_VERSION - 1];
	cap->BOOST_OUTPUT_CURRENT_MAX = Cap_Version_Table[3][cap->BOOST_VERSION - 1];//lzq
	
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
    boardcom->boost_mode_flag = cap->SD_flag;//SD卡
    boardcom->power_limit = referee->chassis_power_limit;
    boardcom->power_level_limit = referee->chassis_power_level_limit;
    boardcom->power_buffer = referee->buffer_energy;
    boardcom->chassis_power = referee->chassis_power;
    cap->last_update_time = HAL_GetTick();
}