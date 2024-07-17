/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 20:07:58
 */

#include "app_gimbal.h"
#include "app_chassis.h"
#include "config_ctrl.h"
#include "lib_math.h"
#include "protocol_board.h"

GimbalYaw_ControlTypeDef GimbalYaw_Control;

/**
 * @brief      Get the pointer of gimbal control object
 * @param      NULL
 * @retval     Pointer to gimbal control object
 */
GimbalYaw_ControlTypeDef *GimbalYaw_GetControlPtr()
{
    return &GimbalYaw_Control;
}

/**
 * @brief      Gimbal yaw control initialization
 * @param      NULL
 * @retval     NULL
 */
void GimbalYaw_Init()
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    gimbalyaw->mode_change_flag = 0;
    gimbalyaw->yaw_ref = 0;
    gimbalyaw->present_mode = GimbalYaw_NO_AUTO;
    gimbalyaw->last_mode = GimbalYaw_NO_AUTO;
    Motor_GimbalYaw.encoder.angle = 4000;
    GimbalYaw_ParamInit();
}

/**
 * @brief      Set up the yaw mode of gimbal
 * @param      mode: gimbal yaw mode
 * @retval     NULL
 */
void GimbalYaw_ModeSet(GimbalYaw_ModeEnum mode)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    gimbalyaw->last_mode = gimbalyaw->present_mode;
    gimbalyaw->present_mode = mode;

    if (gimbalyaw->last_mode != gimbalyaw->present_mode)
    {
        gimbalyaw->mode_change_flag = 1;
    }
}

void GimbalYaw_ModeProcess()
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    switch (boardcom->yaw_mode)
    {
    case GIMBAL_YAW_NO_AUTO:
        GimbalYaw_ModeSet(GimbalYaw_NO_AUTO);
        break;
    case GIMBAL_YAW_ARMOR:
        GimbalYaw_ModeSet(GimbalYaw_ARMOR);
        break;
    case GIMBAL_YAW_BIG_ENERGY:
        GimbalYaw_ModeSet(GimbalYaw_BIG_ENERGY);
        break;
    case GIMBAL_YAW_SMALL_ENERGY:
        GimbalYaw_ModeSet(GimbalYaw_SMALL_ENERGY);
        break;
    default:
        return;
    }
    if (gimbalyaw->mode_change_flag == 1)
    {
        PID_Clear(&(gimbalyaw->spd_no_auto));
        PID_Clear(&(gimbalyaw->pos_no_auto));
        PID_Clear(&(gimbalyaw->spd_armor));
        PID_Clear(&(gimbalyaw->pos_armor));
        PID_Clear(&(gimbalyaw->spd_big_energy));
        PID_Clear(&(gimbalyaw->pos_big_energy));
        PID_Clear(&(gimbalyaw->spd_small_energy));
        PID_Clear(&(gimbalyaw->pos_small_energy));

        if (gimbalyaw->present_mode == GimbalYaw_NO_AUTO)
        {
            gimbalyaw->spd_no_auto.ref = gimbalyaw->last_yaw_spd_ref;
            gimbalyaw->pos_no_auto.ref = gimbalyaw->last_yaw_pos_ref;
        }
        if (gimbalyaw->present_mode == GimbalYaw_ARMOR)
        {
            gimbalyaw->spd_armor.ref = gimbalyaw->last_yaw_spd_ref;
            gimbalyaw->pos_armor.ref = gimbalyaw->last_yaw_pos_ref;
        }
        if (gimbalyaw->present_mode == GimbalYaw_BIG_ENERGY)
        {
            gimbalyaw->spd_big_energy.ref = gimbalyaw->last_yaw_spd_ref;
            gimbalyaw->pos_big_energy.ref = gimbalyaw->last_yaw_pos_ref;
        }
        if (gimbalyaw->present_mode == GimbalYaw_SMALL_ENERGY)
        {
            gimbalyaw->spd_small_energy.ref = gimbalyaw->last_yaw_spd_ref;
            gimbalyaw->pos_small_energy.ref = gimbalyaw->last_yaw_pos_ref;
        }
        gimbalyaw->mode_change_flag = 0;
    }
}

float test_ref = 0.0f;
void GimbalYaw_CalcOutput()
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    switch (gimbalyaw->present_mode)
    {
    case GimbalYaw_NO_AUTO:
#if YAW_MOVE == YAW_REMOTE
        PID_SetRef(&(gimbalyaw->pos_no_auto), gimbalyaw->yaw_ref);
#endif
#if YAW_MOVE == YAW_STEP
        PID_SetRef(&gimbalyaw->pos_no_auto, test_ref);
#endif
        PID_SetFdb(&gimbalyaw->pos_no_auto, Math_Consequent_To_Limited(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_no_auto, Filter_Lowpass(PID_Calc(&gimbalyaw->pos_no_auto), &(gimbalyaw->spd_ref_filter)));
        PID_SetFdb(&gimbalyaw->spd_no_auto, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_no_auto));
        gimbalyaw->last_yaw_spd_ref = gimbalyaw->spd_no_auto.ref;
        gimbalyaw->last_yaw_pos_ref = gimbalyaw->pos_no_auto.ref;
#if IF_SYS_IDENT == SYS_IDENT
        Motor_SetOutput(&Motor_GimbalYaw, openloop_spd_ref);
#endif
        break;
    case GimbalYaw_ARMOR:
        PID_SetRef(&gimbalyaw->pos_armor, Filter_Lowpass(gimbalyaw->yaw_ref, &(gimbalyaw->autoaim_yaw_filter)));
        PID_SetFdb(&gimbalyaw->pos_armor, Math_Consequent_To_Limited(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_armor, Filter_Lowpass(PID_Calc(&gimbalyaw->pos_armor), &(gimbalyaw->spd_ref_filter)));
        PID_SetFdb(&gimbalyaw->spd_armor, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_armor));
        gimbalyaw->last_yaw_spd_ref = gimbalyaw->spd_armor.ref;
        gimbalyaw->last_yaw_pos_ref = gimbalyaw->pos_armor.ref;
        break;
    case GimbalYaw_BIG_ENERGY:
        PID_SetRef(&gimbalyaw->pos_big_energy, gimbalyaw->yaw_ref);
        PID_SetFdb(&gimbalyaw->pos_big_energy, Math_Consequent_To_Limited(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_big_energy, Filter_Lowpass(PID_Calc(&gimbalyaw->pos_big_energy), &(gimbalyaw->spd_ref_filter)));
        PID_SetFdb(&gimbalyaw->spd_big_energy, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_big_energy));
        gimbalyaw->last_yaw_spd_ref = gimbalyaw->spd_big_energy.ref;
        gimbalyaw->last_yaw_pos_ref = gimbalyaw->pos_big_energy.ref;
        break;
    case GimbalYaw_SMALL_ENERGY:
        PID_SetRef(&gimbalyaw->pos_small_energy, gimbalyaw->yaw_ref);
        PID_SetFdb(&gimbalyaw->pos_small_energy, Math_Consequent_To_Limited(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_small_energy, Filter_Lowpass(PID_Calc(&gimbalyaw->pos_small_energy), &(gimbalyaw->spd_ref_filter)));
        PID_SetFdb(&gimbalyaw->spd_small_energy, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_small_energy));
        gimbalyaw->last_yaw_spd_ref = gimbalyaw->spd_small_energy.ref;
        gimbalyaw->last_yaw_pos_ref = gimbalyaw->pos_small_energy.ref;
        break;
    default:
        break;
    }
}

/**
 * @brief      Control function of gimbal yaw
 * @param      NULL
 * @retval     NULL
 */
void GimbalYaw_Output()
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();

    GimbalYaw_SetRef(boardcom->yaw_ref);
    GimbalYaw_SetAngleFdb(boardcom->yaw_pos_fdb);
    GimbalYaw_SetSpeedFdb(boardcom->yaw_spd_fdb);

    GimbalYaw_ModeProcess();
    GimbalYaw_CalcOutput();

#if IF_SYS_IDENT == SYS_IDENT
    openloop_spd_fdb = gimbalyaw->yaw_speed_fdb;
#endif
    if (boardcom->check_in == 1 || boardcom_decoded_count > BOARDCOM_TIMEOUT_VALUE)
    {
        Motor_SetOutput(&Motor_GimbalYaw, 0.0f);
    }
    else
    {
        boardcom_decoded_count++;
    }
    Motor_CAN_SendGroupOutput(&Motor_GimbalMotors);
}

/**
 * @brief      Set the target value of gimbal yaw
 * @param      yaw_ref: gimbal yaw target value
 * @retval     NULL
 */
void GimbalYaw_SetRef(float yaw_ref)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    gimbalyaw->yaw_ref = yaw_ref * 3.1415926f / 180;
}

/**
 * @brief      Setting IMU yaw position feedback
 * @param      yaw_pos_fdb: IMU Yaw Position feedback
 * @retval     NULL
 */
void GimbalYaw_SetAngleFdb(float yaw_pos_fdb)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    gimbalyaw->yaw_position_fdb = yaw_pos_fdb * 3.1415926f / 180;
}

/**
 * @brief      Setting IMU yaw speed feedback
 * @param      yaw_pos_fdb: IMU Yaw Speed feedback
 * @retval     NULL
 */
void GimbalYaw_SetSpeedFdb(float yaw_pos_fdb)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    gimbalyaw->yaw_speed_fdb = yaw_pos_fdb;
}
