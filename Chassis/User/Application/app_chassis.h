/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-16 01:55:47
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "alg_pid.h"
#include "cmsis_os.h"
#include "lib_math.h"
#include "lib_power_ctrl.h"
#include "protocol_board.h"
#include "protocol_referee.h"

#define FORWARD_LEFT 0
#define FORWARD_RIGHT 1
#define BACKWARD_RIGHT 2
#define BACKWARD_LEFT 3

#define CW 0
#define CCW 1

    typedef enum
    {
        CHASSIS_STOP = 0u,
        CHASSIS_NORMAL,
        CHASSIS_GYRO
    } Chassis_ModeEnum;

    typedef struct
    {
        float forward_back_ref;
        float left_right_ref;
        float rotate_ref;
    } Chassis_RefDataTypeDef;

    typedef struct
    {
        Chassis_ModeEnum present_mode, last_mode;
        Chassis_RefDataTypeDef last_ref; // Last target value
        Chassis_RefDataTypeDef raw_ref;  // Original headless speed target value
        Chassis_RefDataTypeDef move_ref; // Chassis speed target value

        float wheel_ref[4];
        float last_wheel_ref[4];
        float chassis_I[4];
        float chassis_W[4];

        PID_TypeDef Chassis_MotorSpdPID[4];
        PID_TypeDef Chassis_SpdfollowPID, Chassis_AngfollowPID;
    } Chassis_ControlTypeDef;

    extern PCArgsType Power_Control_Args;
    extern float PC_Limit_K;
    extern float PC_Limit_B;
    extern float PowerUp_Coef;
    extern float Install_Angle;
    extern float FLY_ANGLE;
    extern float Vx, Vy, Wm;
    extern float wheelvel[4];

    Chassis_ControlTypeDef *Chassis_GetControlPtr(void);
    void Chassis_Init(void);
    void Chassis_ModeSet(Chassis_ModeEnum mode);
    void Chassis_SetMoveRef(float forward_back_ref, float left_right_ref);
    void MecChassis_Output(void);
    void OmniChassis_Output(void);
    static void Chassis_CalcMoveRef(void);
    static void Chassis_CalcGyroRef(void);
    static void Chassis_CalcMecFollowRef(void);
    static void Chassis_CalcOmmiFollowRef(void);
    static void Chassis_CalcWheelRef(void);
    void Calc_ChassisVel(float omega[4], float vx, float vy, float wm, float r, float R);

#ifdef __cplusplus
}
#endif

#endif
