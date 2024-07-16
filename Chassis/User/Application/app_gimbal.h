/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-20 00:34:13
 */

#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "alg_pid.h"
	
    typedef enum
    {
        GimbalYaw_NO_AUTO = 0u,
        GimbalYaw_ARMOR = 1u,
        GimbalYaw_BIG_ENERGY = 2u,
        GimbalYaw_SMALL_ENERGY = 3u,
    } GimbalYaw_ModeEnum;

    typedef struct
    {
        GimbalYaw_ModeEnum present_mode, last_mode;
        uint8_t mode_change_flag;
		
		float last_yaw_spd_ref;
		float last_yaw_pos_ref;
        float yaw_ref;
        float yaw_position_fdb;
        float yaw_speed_fdb;
		
		Filter_Lowpass_TypeDef spd_ref_filter;
        PID_TypeDef spd_no_auto;
        PID_TypeDef pos_no_auto;
        PID_TypeDef spd_armor;
        PID_TypeDef pos_armor;
        PID_TypeDef spd_big_energy;
        PID_TypeDef pos_big_energy;
        PID_TypeDef spd_small_energy;
        PID_TypeDef pos_small_energy;
    } GimbalYaw_ControlTypeDef;
	
	extern float openloop_spd_ref;
	extern float openloop_spd_fdb;

    GimbalYaw_ControlTypeDef *GimbalYaw_GetControlPtr(void);
    void GimbalYaw_Init(void);
    void GimbalYaw_ModeSet(GimbalYaw_ModeEnum mode);
    void GimbalYaw_Output(void);
    void GimbalYaw_SetRef(float yaw_ref);
    void GimbalYaw_SetAngleFdb(float yaw_pos_fdb);
    void GimbalYaw_SetSpeedFdb(float yaw_pos_fdb);
	
	void Test_Pos();
	void Test_Spd();
	void Test_Cur();

#ifdef __cplusplus
}
#endif

#endif
