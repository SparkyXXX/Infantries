/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-10 14:19:05
 */

#include "task_ctrl.h"
#include "config_ctrl.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_ui.h"
#include "protocol_referee.h"
#include "protocol_board.h"
#include "periph_cap.h"
#include "sys_dwt.h"

/**
 * @brief          Chassis task
 * @param          NULL
 * @retval         NULL
 */
void Chassis_Task(void const *argument)
{
    for (;;)
    {
		Motor_IsLost(&Motor_ForwardLeft);
		Motor_IsLost(&Motor_ForwardRight);
		Motor_IsLost(&Motor_BackwardRight);
		Motor_IsLost(&Motor_BackwardLeft);
		Calc_ChassisVel(wheelvel, Vx, Vy, Wm, 0.076f, 0.223f);
        OmniChassis_Output();
        osDelay(1);
    }
}

/**
 * @brief          Gimbal task
 * @param          NULL
 * @retval         NULL
 */
extern uint8_t start_ident_flag;
void Gimbal_Task(void const *argument)
{
    for (;;)
    {
		Motor_IsLost(&Motor_GimbalYaw);
		// start_ident_flag = 1;
        GimbalYaw_Output();
        osDelay(1);
    }
}

void BoardCom_Task(void const *argument)
{
    for (;;)
    {
        BoardCom_Send();
        osDelay(1);
    }
}

/**
 * @brief          SuperCap task
 * @param          NULL
 * @retval         NULL
 */
void Cap_Task(void const *argument)
{
    for (;;)
    {
        Cap_Update();
        osDelay(1);
    }
}

/**
 * @brief          Referee draw task
 * @param          NULL
 * @retval         NULL
 */

uint8_t ui_cmd_last = 0;
void UI_Task(void const *argument)
{
#if IF_SYS_IDENT == NO_SYS_IDENT
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    for (;;)
    {
        if (boardcom->ui_cmd != ui_cmd_last)
        {
            UI_Refresh();
        }
        ui_cmd_last = boardcom->ui_cmd;
        HomeHurt_Detect();
        UI_Update();
        osDelay(20);
    }
#endif
#if IF_SYS_IDENT == SYS_IDENT
	for (;;)
	{
		Test_Response();
		osDelay(1);
	}
#endif
}
