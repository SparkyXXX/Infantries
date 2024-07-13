/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-07-06 22:47:22
 */

#include "task_ctrl.h"

uint8_t pkg_flag = 1;
void BoardCom_Task(void const* argument)
{
    for(;;)
    {
        if(pkg_flag == 1)
        {
            BoardComPkg1_Send();
            pkg_flag = 2;
            osDelay(1);
        }
        if(pkg_flag == 2)
        {
            BoardComPkg2_Send();
            pkg_flag = 1;
            osDelay(1);
        }
    }
}

void Gimbal_Task(void const* argument)
{
    for(;;)
    {
		Motor_IsLost(&Motor_gimbalMotorPitch);
        Gimbal_PitchOutput();
        Gimbal_YawModeSet();
		Check_Task_Freq();
        osDelay(1);
    }
}

void Shoot_Task(void const* argument)
{
    for(;;)
    {
		Motor_IsLost(&Motor_feederMotor);
        Shoot_Update();
        Heat_Update();
        Shoot_FeederLockedJudge();
        Shoot_ShooterControl();
        Shoot_FeederControl();
        Shoot_Output();
        osDelay(1);
    }
}

void AutoAim_Task(void const* argument)
{
    for(;;)
    {
        AutoAim_IsLost();
        AutoAim_Output();
        MiniPC_Update();
        MiniPC_Send();
        osDelay(1);
    }
}

void Remote_Task(void const* argument)
{
    for(;;)
    {
        Remote_DriveModeSet();
        osDelay(1);
    }
}

void Ins_Task(void const* argument)
{
    for(;;)
    {
        INS_Upadte(&BMI088_Data);
        osDelay(1);
    }
}
