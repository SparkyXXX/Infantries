/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 01:18:29
 */

#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "config_ctrl.h"
#include "periph_remote.h"
#include "periph_servo.h"
#include "protocol_board.h"

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
        REMOTE_ARMOR_TEMP,
        REMOTE_BUFF_TEMP
    }
    Remote_AutoAimModeEnum;

    typedef struct
    {
        Remote_AutoAimModeEnum aim_mode;
        uint8_t pending;
        uint8_t on_aim;
        uint8_t mag_state;
        uint8_t gyro_flag;
        uint8_t quiet_flag;
        uint8_t flyslope_flag;
        uint8_t cap_speedup_flag;
        uint8_t autoshoot_flag;

        float keymouse_normal_speed;
        float keymouse_upper_speed;
        float keymouse_flyslope_speed;

        float keymouse_pitch_to_ref;
        float keymouse_yaw_to_ref;
        float keymouse_pitch_to_ref_quiet;
        float keymouse_yaw_to_ref_quiet;
        float remote_pitch_to_ref;
        float remote_yaw_to_ref;
    } Remote_ControlTypeDef;

    extern float Servo_Open;
    extern float Servo_Close;

    Remote_ControlTypeDef* Remote_GetControlPtr(void);
    void Remote_DriveModeSet(void);
    static void Remote_Update(void);
    static void Keymouse_Update(void);
    static void KeyMouse_BuffModeSet(uint8_t mode);
    static void KeyMouse_ArmorModeSet(uint8_t mode);
    static void Following_AutoaimModeSet(void);
    static void KeyMouse_GyroModeSet(void);
    static void KeyMouse_ChassisModeSet(uint8_t chassis_mode);

#ifdef __cplusplus
}
#endif

#endif
