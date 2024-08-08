/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-20 00:38:01
 */
 
#include "config_ctrl.h"
#include "app_gimbal.h"
#include "protocol_motor.h"
#include "lib_math.h"
#include "protocol_board.h"
#include "app_chassis.h"

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

/**
 * @brief      Control function of gimbal yaw
 * @param      NULL
 * @retval     NULL
 */
//float GetAutoFeedForward(float x) {
//	return 12.0f * x + (x > 0 ? 0.03f : -0.03f);
//}

//float lastyawref, llastyawref, refspd, lrefspd, llrefspd;
//void GimbalYaw_Output()
//{
//    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
//    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
//    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();

//    if (gimbalyaw->mode_change_flag == 1)
//    {
//        PID_Clear(&gimbalyaw->spd_no_auto);
//        PID_Clear(&gimbalyaw->pos_no_auto);
//        PID_Clear(&gimbalyaw->spd_armor);
//        PID_Clear(&gimbalyaw->pos_armor);
//        PID_Clear(&gimbalyaw->spd_big_energy);
//        PID_Clear(&gimbalyaw->pos_big_energy);
//        PID_Clear(&gimbalyaw->spd_small_energy);
//        PID_Clear(&gimbalyaw->pos_small_energy);
//        gimbalyaw->mode_change_flag = 0;
//    }
//    switch (chassis->present_mode)
//    {
//    case CHASSIS_GYRO:
//        PID_SetRef(&gimbalyaw->pos_armor, gimbalyaw->yaw_ref);
//        PID_SetFdb(&gimbalyaw->pos_armor, Math_Consequent_To_180(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
//        PID_SetRef(&gimbalyaw->spd_armor, PID_Calc(&gimbalyaw->pos_armor));
//        PID_SetFdb(&gimbalyaw->spd_armor, gimbalyaw->yaw_speed_fdb);
//        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_armor));
//        yaw_pos_error_watch = gimbalyaw->pos_armor.err_watch;
//        break;
//    default:
//		refspd = (gimbalyaw->yaw_ref - llastyawref) / 2;
//        PID_SetRef(&gimbalyaw->pos_no_auto, gimbalyaw->yaw_ref + GetAutoFeedForward((refspd + lrefspd + llrefspd) / 3));
//        PID_SetFdb(&gimbalyaw->pos_no_auto, Math_Consequent_To_180(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
//        PID_SetRef(&gimbalyaw->spd_no_auto, PID_Calc(&gimbalyaw->pos_no_auto));
//        PID_SetFdb(&gimbalyaw->spd_no_auto, gimbalyaw->yaw_speed_fdb);
//        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_no_auto));
//        yaw_pos_error_watch = gimbalyaw->pos_no_auto.err_watch;
//		llastyawref = lastyawref;
//		lastyawref = gimbalyaw->yaw_ref;
//		llrefspd = lrefspd;
//		lrefspd = refspd;
//        break;
//    }
//    if (boardcom->check_in == 1)
//    {
//        Motor_SetOutput(&Motor_GimbalYaw, 0.0f);
//    }
//    Motor_CAN_SendGroupOutput(&Motor_GimbalMotors);
//    yaw_pos_ref_watch = gimbalyaw->yaw_ref;
//    yaw_pos_fdb_watch = gimbalyaw->yaw_position_fdb;
//}
void GimbalYaw_Output()
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    if (gimbalyaw->mode_change_flag == 1)
    {
        PID_Clear(&gimbalyaw->spd_no_auto);
        PID_Clear(&gimbalyaw->pos_no_auto);
        PID_Clear(&gimbalyaw->spd_armor);
        PID_Clear(&gimbalyaw->pos_armor);
        PID_Clear(&gimbalyaw->spd_big_energy);
        PID_Clear(&gimbalyaw->pos_big_energy);
        PID_Clear(&gimbalyaw->spd_small_energy);
        PID_Clear(&gimbalyaw->pos_small_energy);
        gimbalyaw->mode_change_flag = 0;
    }
    switch (gimbalyaw->present_mode)
    {
    case GimbalYaw_NO_AUTO:
        PID_SetRef(&gimbalyaw->pos_no_auto, gimbalyaw->yaw_ref);
        PID_SetFdb(&gimbalyaw->pos_no_auto, Math_Consequent_To_180(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_no_auto, PID_Calc(&gimbalyaw->pos_no_auto));
        PID_SetFdb(&gimbalyaw->spd_no_auto, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_no_auto));
        yaw_pos_error_watch = gimbalyaw->pos_no_auto.err_watch;
        break;
    case GimbalYaw_ARMOR:
        PID_SetRef(&gimbalyaw->pos_armor, gimbalyaw->yaw_ref);
        PID_SetFdb(&gimbalyaw->pos_armor, Math_Consequent_To_180(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_armor, PID_Calc(&gimbalyaw->pos_armor));
        PID_SetFdb(&gimbalyaw->spd_armor, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_armor));
        yaw_pos_error_watch = gimbalyaw->pos_armor.err_watch;
        break;
    case GimbalYaw_BIG_ENERGY:
        PID_SetRef(&gimbalyaw->pos_big_energy, gimbalyaw->yaw_ref);
        PID_SetFdb(&gimbalyaw->pos_big_energy, Math_Consequent_To_180(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_big_energy, PID_Calc(&gimbalyaw->pos_big_energy));
        PID_SetFdb(&gimbalyaw->spd_big_energy, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_big_energy));
        yaw_pos_error_watch = gimbalyaw->pos_big_energy.err_watch;
        break;
    case GimbalYaw_SMALL_ENERGY:
        PID_SetRef(&gimbalyaw->pos_small_energy, gimbalyaw->yaw_ref);
        PID_SetFdb(&gimbalyaw->pos_small_energy, Math_Consequent_To_180(gimbalyaw->yaw_ref, gimbalyaw->yaw_position_fdb));
        PID_SetRef(&gimbalyaw->spd_small_energy, PID_Calc(&gimbalyaw->pos_small_energy));
        PID_SetFdb(&gimbalyaw->spd_small_energy, gimbalyaw->yaw_speed_fdb);
        Motor_SetOutput(&Motor_GimbalYaw, PID_Calc(&gimbalyaw->spd_small_energy));
        yaw_pos_error_watch = gimbalyaw->pos_small_energy.err_watch;
        break;
    default:
        break;
    }
    Motor_CAN_SendGroupOutput(&Motor_GimbalMotors);
    yaw_pos_ref_watch = gimbalyaw->yaw_ref;
    yaw_pos_fdb_watch = gimbalyaw->yaw_position_fdb;
}

/**
 * @brief      Set the target value of gimbal yaw
 * @param      yaw_ref: gimbal yaw target value
 * @retval     NULL
 */
void GimbalYaw_SetRef(float yaw_ref)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    gimbalyaw->yaw_ref = yaw_ref;
}

/**
 * @brief      Setting IMU yaw position feedback
 * @param      yaw_pos_fdb: IMU Yaw Position feedback
 * @retval     NULL
 */
void GimbalYaw_SetAngleFdb(float yaw_pos_fdb)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    gimbalyaw->yaw_position_fdb = yaw_pos_fdb;
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
