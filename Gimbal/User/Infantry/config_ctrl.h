/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-18 12:07:39
 */

#ifndef INIT_CTRL_H
#define INIT_CTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_gimbal.h"
#include "app_shoot.h"
#include "app_autoaim.h"
#include "app_remote.h"
#include "app_ins.h"

#include "periph_motor_can.h"
#include "periph_motor_pwm.h"
#include "periph_servo.h"
#include "periph_bmi088.h"
#include "periph_remote.h"
#include "periph_minipc.h"

#include "protocol_motor.h"
#include "protocol_board.h"
#include "sys_dwt.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "ext_remote_dev.h"
#include "app_monitor.h"

#define ROBOT_NAME BIG_TITAN

#define OLD_WHEAT 1 	//上供弹老麦轮
#define OREO_REO 2		//上供弹新麦轮
#define SWING_DANCE 3	//上供弹全向
#define BIG_TITAN 4		//半下供弹麦轮
#define NEW_TITAN 5		//半下供弹麦轮

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

    extern float imu_yaw_watch;
    extern float imu_pitch_watch;
    extern float imu_roll_watch;
    extern float shooter_diff_watch;
    extern float shooter_left_watch;
    extern float shooter_right_watch;
    extern float pitch_pos_ref_watch;
    extern float pitch_pos_fdb_watch;
    extern float pitch_pos_error_watch;
	extern uint8_t autoshoot_flag_watch;
	extern uint8_t aim_mode_watch;

    void Gimbal_ParamInit(void);
    void Shoot_ParamInit(void);
    void Init_All(void);

#ifdef __cplusplus
}
#endif

#endif
