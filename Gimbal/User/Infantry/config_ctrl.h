/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 22:02:51
 */

#ifndef INIT_CTRL_H
#define INIT_CTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_autoaim.h"
#include "app_gimbal.h"
#include "app_ins.h"
#include "app_remote.h"
#include "app_shoot.h"
#include "test_ctrl.h"

#include "periph_bmi088.h"
#include "periph_motor_can.h"
#include "periph_motor_pwm.h"
#include "periph_remote.h"
#include "periph_servo.h"
#include "protocol_minipc.h"

#include "cmsis_os.h"
#include "protocol_board.h"
#include "sys_dwt.h"
#include "usb_device.h"

    extern Servo_DataTypeDef Servo_MagServo;
    extern BMI088_DataTypeDef BMI088_Data;

    extern Motor_GroupDataTypeDef *Motor_groupHandle[3];
    extern Motor_GroupDataTypeDef Motor_gimbalMotors;
    extern Motor_GroupDataTypeDef Motor_feederMotors;
    extern MotorPWM_GroupDataTypeDef Motor_shooterMotors;

    extern Motor_DataTypeDef Motor_gimbalMotorPitch;
    extern Motor_DataTypeDef Motor_feederMotor;
    extern MotorPWM_DataTypeDef Motor_shooterMotorLeft;
    extern MotorPWM_DataTypeDef Motor_shooterMotorRight;

    void Gimbal_ParamInit(void);
    void Shoot_ParamInit(void);
    void Init_All(void);

#ifdef __cplusplus
}
#endif

#endif
