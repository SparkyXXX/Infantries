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

#include "test_ctrl.h"
#include "periph_motor_can.h"
#include "stdint.h"

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
