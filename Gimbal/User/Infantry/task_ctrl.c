 /*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-02-29 01:46:30
 */

#include "task_ctrl.h"

int GLOBAL_INIT_FLAG = 0;

void Init_Task(void const *argument)
{
    Init_All();
    TaskHandle_t InitTask_Handler = xTaskGetHandle(pcTaskGetName(NULL));
    for (;;)
    {
        GLOBAL_INIT_FLAG = 1;
        vTaskSuspend(InitTask_Handler);
        osDelay(1);
    }
}

uint8_t pkg_flag = 1;
void BoardCom_Task(void const *argument)
{
    for (;;)
    {
        while (!GLOBAL_INIT_FLAG)
        {
            osDelay(1);
        }
        if (pkg_flag == 1)
        {
            BoardComPkg1_Send();
            pkg_flag = 2;
            osDelay(1);
        }
        if (pkg_flag == 2)
        {
            BoardComPkg2_Send();
            pkg_flag = 1;
            osDelay(1);
        }
    }
}

void Gimbal_Task(void const *argument)
{
    for (;;)
    {
        while (!GLOBAL_INIT_FLAG)
        {
            osDelay(1);
        }
        Gimbal_PitchOutput();
        Gimbal_YawModeSet();
        osDelay(1);
    }
}

void Shoot_Task(void const *argument)
{
    for (;;)
    {
        while (!GLOBAL_INIT_FLAG)
        {
            osDelay(1);
        }
        Shoot_Update();
        Shoot_FeederLockedJudge();
        Shoot_ShooterControl();
        Shoot_FeederControl();
        Shoot_Output();
        osDelay(1);
    }
}

void AutoAim_Task(void const *argument)
{
    for (;;)
    {
        while (!GLOBAL_INIT_FLAG)
        {
            osDelay(1);
        }
        AutoAim_IsLost();
        AutoAim_Output();
        MiniPC_Update();
        MiniPC_Send();
        osDelay(1);
    }
}

void Remote_Task(void const *argument)
{
    for (;;)
    {
        while (!GLOBAL_INIT_FLAG)
        {
            osDelay(1);
        }
        Remote_DriveModeSet();
        osDelay(1);
    }
}

void Ins_Task(void const *argument)
{
    while (!GLOBAL_INIT_FLAG)
    {
        osDelay(1);
    }
    INS_Upadte(&BMI088_Data);
}

/**
 * @brief          Monitor
 * @param          NULL
 * @retval         NULL
 */
void Monitor_Task(void const *argument)
{
	for (;;)
    {
        while (!GLOBAL_INIT_FLAG)
        {
            osDelay(1);
        }
        //Monitor_Check();
        osDelay(10);
    }
}
