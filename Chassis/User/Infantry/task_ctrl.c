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

int Global_Init_Flag = 0;

void Init_Task(void const *argument)
{
    Init_All();
    TaskHandle_t InitTask_Handler = xTaskGetHandle(pcTaskGetName(NULL));
    for (;;)
    {
        Global_Init_Flag = 1;
        vTaskSuspend(InitTask_Handler);
        osDelay(1);
    }
}

/**
 * @brief          Chassis task
 * @param          NULL
 * @retval         NULL
 */
void Chassis_Task(void const *argument)
{
    while (!Global_Init_Flag)
    {
        osDelay(1);
    }
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
    while (!Global_Init_Flag)
    {
        osDelay(1);
    }
    for (;;)
    {
        GimbalYaw_Output();
        osDelay(1);
    }
}

void BoardCom_Task(void const *argument)
{
    while (!Global_Init_Flag)
    {
        osDelay(1);
    }
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
    while (!Global_Init_Flag)
    {
        osDelay(1);
    }
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
    while (!Global_Init_Flag)
    {
        osDelay(1);
    }
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
