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
    for (;;)
    {
        while (!Global_Init_Flag)
        {
            osDelay(1);
        }
		MecChassis_Output();
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
        while (!Global_Init_Flag)
        {
            osDelay(1);
        }
        GimbalYaw_Output();
        osDelay(1);
    }
}

void BoardCom_Task(void const *argument)
{
    for (;;)
    {
        while (!Global_Init_Flag)
        {
            osDelay(1);
        }
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
        while (!Global_Init_Flag)
        {
            osDelay(1);
        }
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
        while (!Global_Init_Flag)
        {
            osDelay(1);
        }
		if (boardcom->ui_cmd != ui_cmd_last)
		{
			UI_Refresh();
		}
	    ui_cmd_last = boardcom->ui_cmd;
		HomeHurt_Detect();
		UI_Update();
        osDelay(1);
    }
}
