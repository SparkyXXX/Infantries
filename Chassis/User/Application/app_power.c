/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-07 12:05:19
 */
#include "config_ctrl.h"
#include "app_power.h"
#include "app_chassis.h"
#include "protocol_referee.h"
#include "protocol_motor.h"
#include "lib_math.h"
#include "periph_motor_can.h"
#include "periph_cap.h"

PowerCtrl_ControlTypeDef PowerCtrl_Data;
uint8_t Lowpower_Protect_Flag = 0;

PowerCtrl_ControlTypeDef *PowerCtrl_GetDataPtr(void)
{
    return &PowerCtrl_Data;
}

/**
 * @brief 	功率控制初始化
 * @param 	None
 * @retval	None
 * @note	None
 */
void PowerCtrl_Init(PowerCtrl_ModeMnum Ctrl_state, Motor_GroupDataTypeDef *Wheel)
{
    PowerCtrl_ControlTypeDef *powerctrl = PowerCtrl_GetDataPtr();
    powerctrl->wheel_motor = Wheel;
    powerctrl->starting_flag = 0;
    powerctrl->down_flag = 0;

    PID_Init(&powerctrl->chassi_speed_pid, PID_POSITION, 75.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 11000.0f, 1.0f, 1.0f, 1.0f);
    //                                       mode          p     i     d     kf1   kf2   sum_max  output_max d_fil  kf1_fil  kf2_fil
    for (int i = 0; i < 4; i++)
    {
        PID_Init(&powerctrl->wheel_current_pid[i], PID_POSITION, 4.0f, 0.1f, 50.0f, 0.0f, 0.0f, 1000.0f, 13000.0f, 1.0f, 1.0f, 1.0f);
        Filter_Lowpass_Init(2 * PI * 0.001f * 2, &powerctrl->wheel_current_lpf[i]);
        Filter_Lowpass_Init(2 * PI * 0.001f * 5, &powerctrl->wheel_speed_lpf[i]);
        Filter_Aver_Init(&powerctrl->wheel_speed_avf[i], 20);
        powerctrl->wheel_mode[i] = STOP;
    }
}

void PowerCtrl_SpeedCalc(PID_TypeDef *pid)
{
    PowerCtrl_ControlTypeDef *powerctrl = PowerCtrl_GetDataPtr();
    if (Lowpower_Protect_Flag == 1)
    {
        pid->ref /= 2.0;
    }
    PID_Calc(&powerctrl->chassi_speed_pid);
}

void PowerCtrl_CurrentCalc(uint8_t motor_num)
{
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    PowerCtrl_ControlTypeDef *powerctrl = PowerCtrl_GetDataPtr();

    if (powerctrl->wheel_mode[motor_num] != DOWN)
    {
        powerctrl->wheel_current[motor_num] = Filter_Lowpass(powerctrl->wheel_motor->motor_handle[motor_num]->encoder.current, &powerctrl->wheel_current_lpf[motor_num]);
        powerctrl->wheel_motor->motor_handle[motor_num]->output = Filter_Lowpass(powerctrl->wheel_motor->motor_handle[motor_num]->output, &powerctrl->wheel_speed_lpf[motor_num]);
        powerctrl->wheel_motor->motor_handle[motor_num]->output = Filter_Aver(powerctrl->wheel_motor->motor_handle[motor_num]->output, &powerctrl->wheel_speed_avf[motor_num]);
        PID_SetRef(&powerctrl->wheel_current_pid[motor_num], powerctrl->wheel_motor->motor_handle[motor_num]->output);
        PID_SetFdb(&powerctrl->wheel_current_pid[motor_num], powerctrl->wheel_current[motor_num]);
        PID_Calc(&powerctrl->wheel_current_pid[motor_num]);
    }
    else
    {
        powerctrl->wheel_current_pid[motor_num].output = powerctrl->wheel_motor->motor_handle[motor_num]->output;
    }
}

/**
 * @brief 	功率PID计算
 * @param 	None
 * @retval	None
 * @note	None
 */
void PowerPID_Calc(float power_ref, float power_fdb)
{
    PowerCtrl_ControlTypeDef *powerctrl = PowerCtrl_GetDataPtr();
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();

    if (power_fdb > power_ref)
    {
		PID_SetRef(&powerctrl->powerctrl_pid, power_ref / 100); // 缩小一百倍PID系数好设计
		PID_SetFdb(&powerctrl->powerctrl_pid, power_fdb / 100);
		PID_Calc(&powerctrl->powerctrl_pid);
        powerctrl->power_scale = powerctrl->powerctrl_pid.output;
    }
    else
    {
        powerctrl->power_scale = 0;
    }
}

/**
 * @brief 	功控方案
 * @param 	None
 * @retval	None
 * @note	None
 */
void PowerCtrl_Calc(void)
{
    PowerCtrl_ControlTypeDef *powerctrl = PowerCtrl_GetDataPtr();
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    Referee_DataTypeDef *referee = Referee_GetDataPtr();
    Cap_DataTypeDef *cap = Cap_GetDataPtr();
    for (int i = 0; i < 4; i++)
    {
        PowerCtrl_SpeedCalc(&chassis->Chassis_MotorSpdPID[i]);
        PowerCtrl_CurrentCalc(i);
    }

    if (cap->rest_energy >= 25)
    {
        Lowpower_Protect_Flag = 0;
    }

    if (cap->rest_energy < 15)
    {
        Lowpower_Protect_Flag = 1;
    }
    if (cap->rest_energy == 0)
    {
        Lowpower_Protect_Flag = 1;
    }

    if (cap->boost_mode == 1)
    {
        PowerPID_Calc(320.0f, cap->sum_power);
    }
    else
    {
        powerctrl->power_scale = 0;
    }

    if (referee->buffer_energy <= 20)
    {
        powerctrl->power_scale = 0.99;
    }

    // 最终输出电流环结果乘以功率PID计算分度系数
    powerctrl->wheel_current_pid[0].output = (powerctrl->wheel_current_pid[0].output) * (1 - fabs(powerctrl->power_scale));
    powerctrl->wheel_current_pid[1].output = (powerctrl->wheel_current_pid[1].output) * (1 - fabs(powerctrl->power_scale));
    powerctrl->wheel_current_pid[2].output = (powerctrl->wheel_current_pid[2].output) * (1 - fabs(powerctrl->power_scale));
    powerctrl->wheel_current_pid[3].output = (powerctrl->wheel_current_pid[3].output) * (1 - fabs(powerctrl->power_scale));

    Motor_SetOutput(powerctrl->wheel_motor->motor_handle[0], (powerctrl->wheel_current_pid[0].output));
    Motor_SetOutput(powerctrl->wheel_motor->motor_handle[1], (powerctrl->wheel_current_pid[1].output));
    Motor_SetOutput(powerctrl->wheel_motor->motor_handle[2], (powerctrl->wheel_current_pid[2].output));
    Motor_SetOutput(powerctrl->wheel_motor->motor_handle[3], (powerctrl->wheel_current_pid[3].output));
}

void JudgeMotor_Mode(PID_TypeDef *speed_pid, uint8_t motor_num, float last_ref)
{
    PowerCtrl_ControlTypeDef *powerctrl = PowerCtrl_GetDataPtr();
    if (fabs(speed_pid->ref) - fabs(last_ref) >= 0.3f)
    {
        powerctrl->wheel_mode[motor_num] = STARTING;
    }
    else if (fabs(last_ref) - fabs(speed_pid->ref) >= 0.3f)
    {
        powerctrl->wheel_mode[motor_num] = DOWN;
    }
    else if (fabs(speed_pid->ref) <= 1.0f)
    {
        powerctrl->wheel_mode[motor_num] = STOP;
    }
    else
    {
        powerctrl->wheel_mode[motor_num] = STABLE;
    }
}

/**
 * @brief 	电机整体状态控制
 * @param 	None
 * @retval	None
 * @note	None
 */
void Motor_ModeControl(void)
{
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    PowerCtrl_ControlTypeDef *powerctrl = PowerCtrl_GetDataPtr();
    Cap_DataTypeDef *cap = Cap_GetDataPtr();

    for (int i = 0; i < 4; i++)
    {
        JudgeMotor_Mode(&chassis->Chassis_MotorSpdPID[i], i, chassis->last_motor_ref[i]);
    }

    if (cap->starting_time >= 300 && cap->cap_mode_starting == 1)
    {
        cap->starting_time = 0;
        cap->cap_mode_starting = 0;

        if (powerctrl->wheel_mode[0] == STARTING &&
            powerctrl->wheel_mode[1] == STARTING &&
            powerctrl->wheel_mode[2] == STARTING &&
            powerctrl->wheel_mode[3] == STARTING)
        {
            cap->cap_mode_starting = 1;
        }
        else
        {
            cap->cap_mode_starting = 0;
        }
    }
    else if (cap->cap_mode_starting == 1)
    {
        cap->starting_time++;
    }
    else
    {
        if (powerctrl->wheel_mode[0] == STARTING &&
            powerctrl->wheel_mode[1] == STARTING &&
            powerctrl->wheel_mode[2] == STARTING &&
            powerctrl->wheel_mode[3] == STARTING)
        {
            cap->cap_mode_starting = 1;
        }
        else
        {
            cap->cap_mode_starting = 0;
        }
    }

    if (cap->cap_mode_starting == 1)
    {
        powerctrl->starting_flag = 1;
    }
    else if (powerctrl->wheel_mode[0] == DOWN &&
             powerctrl->wheel_mode[1] == DOWN &&
             powerctrl->wheel_mode[2] == DOWN &&
             powerctrl->wheel_mode[3] == DOWN)
    {
        powerctrl->down_flag = 1;
    }
    else
    {
        powerctrl->starting_flag = 0;
        powerctrl->down_flag = 0;
    }
}

/**
 * @brief 	Motor output control
 * @param 	None
 * @retval	None
 * @note	None
 */
void Output_Control(void)
{
#if POWER_CTRL == NEW
	Motor_CAN_SendGroupOutput(&Motor_ChassisMotors);
#endif
#if POWER_CTRL == OLD
    Motor_ModeControl();
    PowerCtrl_Calc();
    Motor_CAN_SendGroupOutput(&Motor_ChassisMotors);
#endif
}
