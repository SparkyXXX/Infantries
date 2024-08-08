/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-16 04:30:28
 */

#ifndef INIT_CTRL_H
#define INIT_CTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "periph_motor_can.h"
#include "stdint.h"

#define ROBOT_NAME BIG_TITAN

#define OLD_WHEAT 1 	//上供弹老麦轮
#define OREO_REO 2		//上供弹新麦轮
#define SWING_DANCE 3	//上供弹全向
#define BIG_TITAN 4		//半下供弹麦轮
#define NEW_TITAN 5		//半下供弹麦轮
	
#define POWER_CTRL NEW
#define NEW 1
#define OLD 2

    extern const uint16_t Gyro_Speed_Table[12][2];

    extern uint16_t chassis_power_limit_watch;
    extern uint8_t buffer_energy_watch;
    extern uint8_t rest_energy_watch;
    extern uint8_t launching_frequency_watch;
    extern float initial_speed_watch;
    extern float yaw_pos_ref_watch;
    extern float yaw_pos_fdb_watch;
    extern float yaw_pos_error_watch;

    extern Motor_GroupDataTypeDef *Motor_groupHandle[2];
    extern Motor_GroupDataTypeDef Motor_ChassisMotors;
    extern Motor_GroupDataTypeDef Motor_GimbalMotors;

    extern Motor_DataTypeDef Motor_ForwardLeft;
    extern Motor_DataTypeDef Motor_ForwardRight;
    extern Motor_DataTypeDef Motor_BackwardRight;
    extern Motor_DataTypeDef Motor_BackwardLeft;
    extern Motor_DataTypeDef Motor_GimbalYaw;

    void Chassis_ParamInit(void);
    void GimbalYaw_ParamInit(void);
    static void Init_AllMotors(void);
    void Init_All(void);

#ifdef __cplusplus
}
#endif

#endif
