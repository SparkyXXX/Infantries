/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-16 16:23:34
 */

#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "alg_pid.h"
#include "app_ins.h"
#include "cmsis_os.h"
#include "config_ctrl.h"

#define PITCH 0

    typedef enum
    {
        GIMBAL_NO_AUTO = 0u,
        GIMBAL_ARMOR = 1u,
        GIMBAL_BIG_ENERGY = 2u,
        GIMBAL_SMALL_ENERGY = 3u
    } Gimbal_ModeEnum;

    typedef enum
    {
        GIMBAL_YAW_NO_AUTO = 0u,
        GIMBAL_YAW_ARMOR = 1u,
        GIMBAL_YAW_BIG_ENERGY = 2u,
        GIMBAL_YAW_SMALL_ENERGY = 3u
    } GimbalYaw_ModeEnum;

    typedef struct
    {
        Gimbal_ModeEnum present_mode, last_mode;
        GimbalYaw_ModeEnum yaw_mode;
        uint8_t mode_change_flag;

        float last_pitch_spd_ref;
        float last_pitch_pos_ref;
        float yaw_position_ref;
        float pitch_position_ref;

        FuzzyPID_TypeDef pitch_spd;
        FuzzyPID_TypeDef pitch_pos;
    } Gimbal_ControlTypeDef;

    Gimbal_ControlTypeDef *Gimbal_GetControlPtr(void);
    void Gimbal_Init(void);
    void Gimbal_PitchOutput(void);
    void Gimbal_ModeSet(Gimbal_ModeEnum mode);
    void Gimbal_YawModeSet(void);

#endif

#ifdef __cplusplus
}
#endif
