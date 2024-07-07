/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-07-06 22:32:59
 */

#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "config_ctrl.h"
#include "protocol_board.h"
#include "periph_servo.h"
#include "periph_remote.h"

#define CHASSIS_STOP 0x00
#define CHASSIS_NORMAL 0x01
#define CHASSIS_GYRO 0x02

#define KEY(T) (remote->key.T)
#define KEY2(T1, T2) (remote->key.T1 && remote->key.T2)
#define KEY3(T1, T2, T3) (remote->key.T1 && remote->key.T2 && remote->key.T3)
#define KEY_UP(T) (Remote_Keylast.T && !keyboard->T)
#define KEY_DN(T) (!Remote_Keylast.T && keyboard->T)

    typedef enum
    {
        REMOTE_ARMOR = 0u,
        REMOTE_BIG_BUFF,
        REMOTE_SMALL_BUFF,
        REMOTE_GYRO,
        REMOTE_ARMOR_TEMP
    } Remote_AutoAimModeEnum;

    typedef struct
    {
        uint8_t pending;
        uint8_t on_aim;
        Remote_AutoAimModeEnum aim_mode;
    } Remote_ControlTypeDef;

    extern uint8_t Remote_Mag_State;
	extern float Remote_Acc;
	extern float Remote_Dec;
	extern float KeyMouse_NormalSpeed;
	extern float KeyMouse_UpperSpeed;
	extern float KeyMouse_FlySlopeSpeed;
    extern float Remote_Pitch_To_Ref;
    extern float Remote_Yaw_To_Ref;
    extern float Mouse_Pitch_To_Ref;
    extern float Mouse_Yaw_To_Ref;
    extern float Mouse_Pitch_To_Ref_Quiet;
    extern float Mouse_Yaw_To_Ref_Quiet;

    extern float Servo_Open;
    extern float Servo_Close;
    extern float Elevation_Angle;
    extern float Depression_Angle;
    extern float AutoShoot_Wait_ms;
    extern uint16_t AutoShootSmallEnergy_Wait_ms;
    extern uint16_t AutoShootBigEnergy_Wait_ms;
    extern uint8_t is_shoot_changed;

    Remote_ControlTypeDef *Remote_GetControlPtr(void);
    void Remote_DriveModeSet(void);
    static void Remote_Update(void);
    static void Keymouse_Update(void);
    static void Remote_AutoaimModeSet(uint8_t mode);
    static void Keymouse_AutoaimModeSet(uint8_t mode);
    static void Following_AutoaimModeSet(void);
    static void Remote_Gyro_ModeSet(void);
    static void Remote_Chassis_ModeSet(uint8_t chassis_mode);

    static void Remote_ShootModeSet(void);
    static void Keymouse_ShootModeSet(void);
	static void AutoAim_ShootModeSet(void);
#ifdef __cplusplus
}
#endif

#endif
