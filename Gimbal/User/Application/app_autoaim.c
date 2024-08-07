/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Chen Zhihong
 * @LastEditTime: 2024-08-05 22:19:04
 */

#include "app_autoaim.h"

AutoAim_ControlTypeDef AutoAim_Control;
AutoAim_ControlTypeDef* AutoAim_GetControlPtr()
{
    return &AutoAim_Control;
}

void AutoAim_Output()
{
    AutoAim_ControlTypeDef* autoaim = AutoAim_GetControlPtr();
    MiniPC_DataTypeDef* minipc = MiniPC_GetDataPtr();
    Gimbal_ControlTypeDef* gimbal = Gimbal_GetControlPtr();
    Shoot_ControlTypeDef* shooter = Shoot_GetControlPtr();
	Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();

    if(gimbal->present_mode != GIMBAL_NO_AUTO)
    {
        if(autoaim->hit_mode == AUTOAIM_HIT_ARMOR)
        {
            shooter->fast_shoot_freq = autoaim->armor_fast_freq;
            shooter->slow_shoot_freq = autoaim->armor_slow_freq;
            autoaim->armor_yaw = -(float)minipc->armor_yaw_ref / 100;
            autoaim->armor_pitch = -(float)minipc->armor_pitch_ref / 100;
            LimitMaxMin(autoaim->armor_pitch, gimbal->elevation_angle, gimbal->depression_angle);
            while(autoaim->armor_yaw >= 180)
            {
                autoaim->armor_yaw -= 360;
            }
            while(autoaim->armor_yaw <= -180)
            {
                autoaim->armor_yaw += 360;
            }
            if(minipc->recieve_packet_type == MINIPC_ARMOR_PACKET && minipc->is_get_target && !MiniPC_IsLost() &&
				minipc->recieving_lock == 0 && !((minipc->armor_yaw_ref == 0) && (minipc->armor_pitch_ref == 0)))
            {
                gimbal->pitch_position_ref = autoaim->armor_pitch;
                gimbal->yaw_position_ref = autoaim->armor_yaw;
            }
        }
        else if(autoaim->hit_mode == AUTOAIM_HIT_BUFF)
        {
			autoaim->is_change_target = remote_control->autoshoot_flag;
            shooter->fast_shoot_freq = autoaim->buff_freq;
            shooter->slow_shoot_freq = autoaim->buff_freq;
            autoaim->buff_yaw = -(float)minipc->buff_yaw_ref / 100;
            autoaim->buff_pitch = -(float)minipc->buff_pitch_ref / 100;
            LimitMaxMin(autoaim->buff_pitch, gimbal->elevation_angle, gimbal->depression_angle);
            while(autoaim->buff_yaw >= 180)
            {
                autoaim->buff_yaw -= 360;
            }
            while(autoaim->buff_yaw <= -180)
            {
                autoaim->buff_yaw += 360;
            }
            if(minipc->recieve_packet_type == MINIPC_BUFF_PACKET && minipc->is_get_target && !MiniPC_IsLost() && autoaim->target_state == AUTOAIM_TARGET_FOLLOWING)
            {
                gimbal->pitch_position_ref = autoaim->buff_pitch;
                gimbal->yaw_position_ref = autoaim->buff_yaw;
            }
        }
    }
    else
    {
        shooter->fast_shoot_freq = autoaim->armor_fast_freq;
        shooter->slow_shoot_freq = autoaim->armor_slow_freq;
    }
}

void AutoAim_ModeSet(AutoAim_ModeEnum mode)
{
    AutoAim_ControlTypeDef* autoaim = AutoAim_GetControlPtr();
    MiniPC_DataTypeDef* minipc = MiniPC_GetDataPtr();
    autoaim->aim_mode = mode;
    minipc->mode = mode;
}

void AutoAim_UpdateTime()
{
    AutoAim_ControlTypeDef* autoaim = AutoAim_GetControlPtr();
    MiniPC_DataTypeDef* minipc = MiniPC_GetDataPtr();
    if(minipc->is_get_target == 0)
    {
        return;
    }
    else
    {
        autoaim->get_target_time = HAL_GetTick();
    }
}

void AutoAim_IsLost()
{
    AutoAim_ControlTypeDef* autoaim = AutoAim_GetControlPtr();
    uint32_t now = HAL_GetTick();
    if((now - autoaim->get_target_time) <= AUTOAIM_LOST_TARGET_TIME)
    {
        autoaim->target_state = AUTOAIM_TARGET_FOLLOWING;
    }
    else if((now - autoaim->get_target_time) >= AUTOAIM_LOST_TARGET_TIME)
    {
        autoaim->target_state = AUTOAIM_TARGET_LOST;
    }
}
