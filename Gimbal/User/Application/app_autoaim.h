/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 00:58:53
 */

#ifndef APP_AUTOAIM_H
#define APP_AUTOAIM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_shoot.h"
#include "lib_filter.h"
#include "lib_math.h"
#include "protocol_board.h"
#include "protocol_minipc.h"

#define AUTOAIM_LOST_TARGET_TIME 100.0f

    typedef enum
    {
        AUTOAIM_ARMOR = 0u,
        AUTOAIM_BIG_BUFF,
        AUTOAIM_SMALL_BUFF,
        AUTOAIM_GYRO,
    } AutoAim_ModeEnum;

    typedef enum
    {
        AUTOAIM_HIT_ARMOR = 0u,
        AUTOAIM_HIT_BUFF
    } AutoAim_HitModeEnum;

    typedef enum
    {
        AUTOAIM_TARGET_LOST = 0u,
        AUTOAIM_TARGET_FOLLOWING
    } AutoAim_StateEnum;

    typedef struct
    {
        AutoAim_ModeEnum aim_mode;
        AutoAim_HitModeEnum hit_mode;
        AutoAim_StateEnum target_state;

        uint32_t get_target_time;
        uint8_t is_change_target;

        float armor_yaw;
        float armor_pitch;
        float buff_yaw;
        float buff_pitch;

        float armor_fast_freq;
        float armor_slow_freq;
        float buff_freq;
    } AutoAim_ControlTypeDef;

    AutoAim_ControlTypeDef *AutoAim_GetControlPtr(void);
    void AutoAim_Output(void);
    void AutoAim_ModeSet(AutoAim_ModeEnum mode);
    void AutoAim_UpdateTime(void);
    void AutoAim_IsLost(void);

#endif

#ifdef __cplusplus
}
#endif
