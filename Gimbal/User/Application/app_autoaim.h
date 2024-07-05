/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-16 04:55:56
 */

#ifndef APP_AUTOAIM_H
#define APP_AUTOAIM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_shoot.h"
#include "protocol_board.h"
#include "lib_math.h"
#include "lib_filter.h"
#include "periph_minipc.h"

#define AUTOAIM_LOST_TARGET_TIME 100.0f

    typedef enum
    {
        AUTOAIM_TARGET_LOST = 0u,
        AUTOAIM_TARGET_FOLLOWING
    } AutoAim_StateEnum;

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

    typedef struct
    {
        float armor_yaw;
        float armor_pitch;
        float buff_yaw;
        float buff_pitch;

        uint32_t get_target_time;
        uint8_t isChangeTarget;
        uint8_t AutoShootFlag;

        AutoAim_ModeEnum aim_mode;
        AutoAim_HitModeEnum hit_mode;
        AutoAim_StateEnum target_state;
    } AutoAim_ControlTypeDef;

    typedef struct __attribute__((packed))
    {
        uint16_t timestamp;
        uint16_t is_get;
		uint16_t is_shoot;
		int16_t yaw_ref;
		int16_t pitch_ref;
        int16_t null0, null1, null2, null3, null4, null5, null6;
    } AutoAim_ArmorPacketDataTypeDef;

    extern uint16_t bullet_speed;
	extern float Feeder_Fast_Speed;
    extern float Feeder_Slow_Speed;
	extern float Armor_Feeder_Fast_Speed;
    extern float Armor_Feeder_Slow_Speed;
	extern float Buff_Feeder_Fast_Speed;
    extern float Buff_Feeder_Slow_Speed;
	
    AutoAim_ControlTypeDef *AutoAim_GetControlPtr(void);
    void AutoAim_Init(void);
    void AutoAim_Output(void);
    void AutoAim_ModeSet(AutoAim_ModeEnum mode);
	void AutoAim_UpdateTime(void);
    void AutoAim_IsLost(void);

#endif

#ifdef __cplusplus
}
#endif
