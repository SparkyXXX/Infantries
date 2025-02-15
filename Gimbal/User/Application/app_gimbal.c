/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-08-16 18:14:18
 */
#include "app_gimbal.h"

Gimbal_ControlTypeDef Gimbal_Control;
Gimbal_ControlTypeDef *Gimbal_GetControlPtr()
{
    return &Gimbal_Control;
}

// pitch轴初始化
void Gimbal_Init()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    gimbal->mode_change_flag = 0;
    gimbal->last_mode = GIMBAL_NO_AUTO;
    gimbal->present_mode = GIMBAL_NO_AUTO;
    gimbal->yaw_mode = GIMBAL_YAW_NO_AUTO;
    Motor_gimbalMotorPitch.encoder.angle = 4000;
    Gimbal_ParamInit();
}

// 计算控制输出，使用模糊PID
float test_ref = 0.0f;
void Gimbal_PitchOutput()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    INS_DataTypeDef *ins = INS_GetControlPtr();

    if (gimbal->mode_change_flag == 1)
    {
        FuzzyPID_Clear(&(gimbal->pitch_spd));
        FuzzyPID_Clear(&(gimbal->pitch_pos));
        gimbal->pitch_spd.ref = gimbal->last_pitch_spd_ref;
        gimbal->pitch_pos.ref = gimbal->last_pitch_pos_ref;
        gimbal->mode_change_flag = 0;
    }
#if PITCH_MODE == PITCH_REMOTE
    FuzzyPID_SetRef(&(gimbal->pitch_pos), Math_AngleToRad(-gimbal->pitch_position_ref));
#endif
#if PITCH_MODE == PITCH_STEP
    FuzzyPID_SetRef(&(gimbal->pitch_pos), -test_ref);
#endif
    FuzzyPID_SetFdb(&(gimbal->pitch_pos), Math_AngleToRad(-ins->pitch));
    FuzzyPID_SetRef(&(gimbal->pitch_spd), FuzzyPID_Calc(&gimbal->pitch_pos));
    FuzzyPID_SetFdb(&(gimbal->pitch_spd), -ins->gyro[PITCH]);
    Motor_SetOutput(&Motor_gimbalMotorPitch, FuzzyPID_Calc(&(gimbal->pitch_spd)));
    Motor_CAN_SendGroupOutput(&Motor_gimbalMotors);
    gimbal->last_pitch_spd_ref = gimbal->pitch_spd.ref;
    gimbal->last_pitch_pos_ref = gimbal->pitch_pos.ref;
}

// pitch轴模式设置
void Gimbal_ModeSet(Gimbal_ModeEnum mode)
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    gimbal->last_mode = gimbal->present_mode;
    gimbal->present_mode = mode;
    if (gimbal->last_mode != gimbal->present_mode)
    {
        gimbal->mode_change_flag = 1;
    }
}

// yaw轴模式设置，用于板通发给底盘
void Gimbal_YawModeSet()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    switch (gimbal->present_mode)
    {
    case GIMBAL_NO_AUTO:
        gimbal->yaw_mode = GIMBAL_YAW_NO_AUTO;
        break;
    case GIMBAL_ARMOR:
        gimbal->yaw_mode = GIMBAL_YAW_ARMOR;
        break;
    case GIMBAL_BIG_ENERGY:
        gimbal->yaw_mode = GIMBAL_YAW_BIG_ENERGY;
        break;
    case GIMBAL_SMALL_ENERGY:
        gimbal->yaw_mode = GIMBAL_YAW_SMALL_ENERGY;
        break;
    }
}
