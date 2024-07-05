/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-07 12:06:22
 */

#ifndef APP_POWER_H
#define APP_POWER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "alg_pid.h"
#include "lib_filter.h"
#include "periph_motor_can.h"

    typedef enum
    {
        STOP = 0,
        STARTING = 1,
        STABLE = 2,
        DOWN = 3
    } Motor_CurrentStateEnum;

    typedef enum
    {
        POWER_UNLIMITED = 0,
        POWER_LIMITED = 1,
    } PowerCtrl_ModeMnum;
	
    typedef struct
    {
        PowerCtrl_ModeMnum mode;
        PID_TypeDef powerctrl_pid;
        PID_TypeDef chassi_speed_pid;
        uint8_t starting_flag;
        uint8_t down_flag;
        float power_scale;

        Motor_GroupDataTypeDef *wheel_motor;
        Motor_CurrentStateEnum wheel_mode[4];
        PID_TypeDef wheel_current_pid[4];
        float wheel_current[4];
        Filter_Lowpass_TypeDef wheel_current_lpf[4];
        Filter_Lowpass_TypeDef wheel_speed_lpf[4];
        Filter_Window_TypeDef wheel_speed_avf[4];
    } PowerCtrl_ControlTypeDef;

    PowerCtrl_ControlTypeDef *PowerCtrl_GetDataPtr(void);
    void PowerCtrl_Init(PowerCtrl_ModeMnum Ctrl_state, Motor_GroupDataTypeDef *Wheel);
    void PowerCtrl_SpeedCalc(PID_TypeDef *pid);
    void PowerCtrl_CurrentCalc(uint8_t motor_num);
    void PowerCtrl_Calc(void);
	void Output_Control(void);
    void JudgeMotor_Mode(PID_TypeDef *speed_pid, uint8_t motor_num, float last_ref);
    void Motor_ModeControl(void);

#ifdef __cplusplus
}
#endif

#endif
