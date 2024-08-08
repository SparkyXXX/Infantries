/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-16 05:58:58
 */

#include "app_gimbal.h"

Gimbal_ControlTypeDef Gimbal_Control;

/**
 * @brief      Gimbal initialization offset and mode
 * @param      NULL
 * @retval     NULL
 */


void Gimbal_Init()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    gimbal->mode_change_flag = 0;
    gimbal->last_mode = GIMBAL_NO_AUTO;
    gimbal->present_mode = GIMBAL_NO_AUTO;
    gimbal->yaw_mode = GIMBAL_YAW_NO_AUTO;
    Motor_gimbalMotorPitch.encoder.angle = 4000;
	Gimbal_ParamInit();
		Filter_Lowpass_Init(0.1, &(gimbal->pitch_pos_ref_filter));
}

/**
 * @brief      Pitch state control
 * @param      NULL
 * @retval     NULL
 */
float last_pitch = 0.0f, now_pitch = 0.0f, feed = 0.0f, last_feed = 0.0f, feed_out = 0.0f, pitch_sig = 0.0f;
float pitch_lamb = 0.8f;
float antiG_k = 0.0f,antiG_b = 0.0f,antiG_NG = 0.0f;
int pitch_start = 0;
void Gimbal_PitchOutput()
{
		Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    INS_DataTypeDef *ins = INS_GetControlPtr();
    

    if (gimbal->mode_change_flag == 1)
    {
		// Clear PID if there is multiple PID param been used
        gimbal->mode_change_flag = 0;
    }
    gimbal->pitch_position_ref_lpf = Filter_Lowpass(gimbal->pitch_position_ref, &(gimbal->pitch_pos_ref_filter));
		float output_with_antiG;
		switch(gimbal->present_mode){
			case GIMBAL_NO_AUTO:
				PID_SetRef(&gimbal->pitch_pos_no_auto, -gimbal->pitch_position_ref);
				PID_SetFdb(&gimbal->pitch_pos_no_auto, -ins->pitch);
				PID_SetRef(&gimbal->pitch_spd_no_auto, PID_Calc(&gimbal->pitch_pos_no_auto));
				PID_SetFdb(&gimbal->pitch_spd_no_auto, -ins->gyro[PITCH]);
				//Motor_SetOutput(&Motor_gimbalMotorPitch, PID_Calc(&gimbal->pitch_spd));
				output_with_antiG = PID_Calc(&gimbal->pitch_spd_no_auto) + antiG_k * (-ins->pitch) + antiG_b + antiG_NG * cos(-ins->pitch);
				LimitMax(output_with_antiG, 30000.0f);
				Motor_SetOutput(&Motor_gimbalMotorPitch, output_with_antiG);
				
			break;
			case GIMBAL_ARMOR:
				PID_SetRef(&gimbal->pitch_pos_armor, -gimbal->pitch_position_ref);
				PID_SetFdb(&gimbal->pitch_pos_armor, -ins->pitch);
				PID_SetRef(&gimbal->pitch_spd_armor, PID_Calc(&gimbal->pitch_pos_armor));
				PID_SetFdb(&gimbal->pitch_spd_armor, -ins->gyro[PITCH]);
				//Motor_SetOutput(&Motor_gimbalMotorPitch, PID_Calc(&gimbal->pitch_spd));
				output_with_antiG = PID_Calc(&gimbal->pitch_spd_armor) + antiG_k * (-ins->pitch) + antiG_b + antiG_NG * cos(-ins->pitch);
				LimitMax(output_with_antiG, 30000.0f);
				Motor_SetOutput(&Motor_gimbalMotorPitch, output_with_antiG);
				
			break;
			case GIMBAL_BIG_ENERGY:
				PID_SetRef(&gimbal->pitch_pos_big_energy, -gimbal->pitch_position_ref_lpf);
				PID_SetFdb(&gimbal->pitch_pos_big_energy, -ins->pitch);
				PID_SetRef(&gimbal->pitch_spd_big_energy, PID_Calc(&gimbal->pitch_pos_big_energy));
				PID_SetFdb(&gimbal->pitch_spd_big_energy, -ins->gyro[PITCH]);
				//Motor_SetOutput(&Motor_gimbalMotorPitch, PID_Calc(&gimbal->pitch_spd));
				output_with_antiG = PID_Calc(&gimbal->pitch_spd_big_energy) + antiG_k * (-ins->pitch) + antiG_b + antiG_NG * cos(-ins->pitch);
				LimitMax(output_with_antiG, 30000.0f);
				Motor_SetOutput(&Motor_gimbalMotorPitch, output_with_antiG);
				
			break;
			case GIMBAL_SMALL_ENERGY:
				PID_SetRef(&gimbal->pitch_pos_small_energy, -gimbal->pitch_position_ref_lpf);
				PID_SetFdb(&gimbal->pitch_pos_small_energy, -ins->pitch);
				PID_SetRef(&gimbal->pitch_spd_small_energy, PID_Calc(&gimbal->pitch_pos_small_energy));
				PID_SetFdb(&gimbal->pitch_spd_small_energy, -ins->gyro[PITCH]);
				//Motor_SetOutput(&Motor_gimbalMotorPitch, PID_Calc(&gimbal->pitch_spd));
				output_with_antiG = PID_Calc(&gimbal->pitch_spd_small_energy) + antiG_k * (-ins->pitch) + antiG_b + antiG_NG * cos(-ins->pitch);
				LimitMax(output_with_antiG, 30000.0f);
				Motor_SetOutput(&Motor_gimbalMotorPitch, output_with_antiG);
				
			break;
			default:
				break;
		}
		
		last_pitch = now_pitch;
		now_pitch = -ins->pitch;
		
//		pitch_sig = 1 / (1 + exp(ins->gyro[PITCH]*5));
//		
//		last_feed = feed;
//		feed = (now_pitch * 408.16 + 16482)* pitch_sig  + (now_pitch * 291.47 - 6538) * (1 - pitch_sig);
//		
//		
//		feed_out = feed * pitch_lamb + last_feed * (1 - pitch_lamb);
//		LimitMaxMin(feed_out, 24000, -24000);
//		feed_out = Motor_gimbalMotorPitch.output + feed_out;
//		LimitMaxMin(feed_out, 30000, -30000);
//		Motor_SetOutput(&Motor_gimbalMotorPitch, feed_out);
		
		
    Motor_CAN_SendGroupOutput(&Motor_gimbalMotors);
    pitch_pos_ref_watch = -gimbal->pitch_position_ref;
    pitch_pos_fdb_watch = -ins->pitch;
}

/**
 * @brief      Gets the pointer to the gimbal control object
 * @param      NULL
 * @retval     The pointer points to the gimbal control object
 */
Gimbal_ControlTypeDef *Gimbal_GetControlPtr()
{
    return &Gimbal_Control;
}

/**
 * @brief      Change Gimbal mode
 * @param      mode: Gimbal mode enum
 * @retval     NULL
 */
void Gimbal_ModeSet(Gimbal_ModeEnum mode)
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    gimbal->last_mode = gimbal->present_mode;
    gimbal->present_mode = mode;
    if (gimbal->last_mode != gimbal->present_mode)
    {
        gimbal->mode_change_flag = 1;
    }
}

/**
 * @brief      Yaw state control
 * @param      NULL
 * @retval     NULL
 */
void Gimbal_YawModeSet()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    switch (gimbal->present_mode)
    {
		case GIMBAL_NO_AUTO:
            gimbal->yaw_mode = GIMBAL_YAW_NO_AUTO;
            break;
        case GIMBAL_ARMOR:
            gimbal->yaw_mode = GIMBAL_YAW_ARMOR;
            break;
        case GIMBAL_BIG_ENERGY:
            gimbal->yaw_mode = GIMBAL_YAW_BIG_ENERGY;
            break;
        case GIMBAL_SMALL_ENERGY:
            gimbal->yaw_mode = GIMBAL_YAW_SMALL_ENERGY;
            break;
    }
}
