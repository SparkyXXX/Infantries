/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-07-06 22:47:22
 */

#include "task_ctrl.h"
#include "lib_timer_tim1.h"

uint64_t begin = 0;
uint64_t end = 0;
uint64_t mid = 0;
float diff = 0;

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
		Check_Task_Freq();
		//	begin = DWT_GetTimeline_us();
        Gimbal_PitchOutput();
        Gimbal_YawModeSet();
        osDelay(1);
        //		Timer_Delay_usE_1(1, 500 - (DWT_GetTimeline_us() - begin)); //DWT不准，500当1000用
        //		end = DWT_GetTimeline_us();
        //		diff = (end - begin) / 1000000.0f;
    }
}

void Shoot_Task(void const* argument)
{
    for(;;)
    {
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
