/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-07-06 21:50:12
 */

#include "app_autoaim.h"

AutoAim_ControlTypeDef AutoAim_Control;
AutoAim_ControlTypeDef *AutoAim_GetControlPtr()
{
    return &AutoAim_Control;
}

void AutoAim_Output()
{
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();

    if (gimbal->present_mode != GIMBAL_NO_AUTO)
    {
        if (autoaim->hit_mode == AUTOAIM_HIT_ARMOR)
        {
			Feeder_Fast_Speed = Armor_Feeder_Fast_Speed;
			Feeder_Slow_Speed = Armor_Feeder_Slow_Speed;
            autoaim->armor_yaw = -(float)minipc->yaw_ref / 100;
            autoaim->armor_pitch = -(float)minipc->pitch_ref / 100;
            LimitMaxMin(autoaim->armor_pitch, Elevation_Angle, Depression_Angle);
            while (autoaim->armor_yaw >= 180)
            {
                autoaim->armor_yaw -= 360;
            }
            while (autoaim->armor_yaw <= -180)
            {
                autoaim->armor_yaw += 360;
            }
            if (minipc->new_data_flag && minipc->is_get_target && !MiniPC_IsLost())
            {
                gimbal->pitch_position_ref = autoaim->armor_pitch;
                gimbal->yaw_position_ref = autoaim->armor_yaw;
            }
        }
        else if (autoaim->hit_mode == AUTOAIM_HIT_BUFF)
        {
			Feeder_Fast_Speed = Buff_Feeder_Fast_Speed;
			Feeder_Slow_Speed = Buff_Feeder_Slow_Speed;
            autoaim->buff_yaw = -(float)minipc->buff_yaw / 100;
            autoaim->buff_pitch = -(float)minipc->buff_pitch / 100;
            LimitMaxMin(autoaim->buff_pitch, Elevation_Angle, Depression_Angle);
            while (autoaim->buff_yaw >= 180)
            {
                autoaim->buff_yaw -= 360;
            }
            while (autoaim->buff_yaw <= -180)
            {
                autoaim->buff_yaw += 360;
            }
            if (minipc->new_data_flag && minipc->is_get_target && !MiniPC_IsLost() && autoaim->target_state == AUTOAIM_TARGET_FOLLOWING)
            {
                gimbal->pitch_position_ref = autoaim->buff_pitch;
                gimbal->yaw_position_ref = autoaim->buff_yaw;
            }
        }
    }
	else
	{
		Feeder_Fast_Speed = Armor_Feeder_Fast_Speed;
		Feeder_Slow_Speed = Armor_Feeder_Slow_Speed;
	}
}

void AutoAim_ModeSet(AutoAim_ModeEnum mode)
{
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    autoaim->aim_mode = mode;
    minipc->mode = mode;
}

void AutoAim_UpdateTime()
{
	AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
	MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
	if (minipc->is_get_target == 0)
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
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    uint32_t now = HAL_GetTick();
    if ((now - autoaim->get_target_time) <= AUTOAIM_LOST_TARGET_TIME)
    {
        autoaim->target_state = AUTOAIM_TARGET_FOLLOWING;
    }
    else if ((now - autoaim->get_target_time) >= AUTOAIM_LOST_TARGET_TIME)
    {
        autoaim->target_state = AUTOAIM_TARGET_LOST;
    }
}
