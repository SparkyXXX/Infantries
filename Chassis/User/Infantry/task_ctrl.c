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
        OmmiChassis_Output();
        osDelay(1);
    }
}

/**
 * @brief          Gimbal task
 * @param          NULL
 * @retval         NULL
 */
void Gimbal_Task(void const *argument)
{
    for (;;)
    {
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
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    for (;;)
    {
#if IF_SYS_IDENT == SYS_IDENT
        Test_Response();
#endif
        if (boardcom->ui_cmd != ui_cmd_last)
        {
            UI_Refresh();
        }

        ui_cmd_last = boardcom->ui_cmd;
        HomeHurt_Detect();
        UI_Update();
        osDelay(20);
    }
}
