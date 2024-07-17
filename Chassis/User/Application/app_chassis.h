/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 01:51:36
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "alg_pid.h"
#include "lib_power_ctrl.h"

#define FORWARD_LEFT 0
#define FORWARD_RIGHT 1
#define BACKWARD_RIGHT 2
#define BACKWARD_LEFT 3

#define CW -1
#define CCW 1

    typedef enum
    {
        CHASSIS_STOP = 0u,
        CHASSIS_NORMAL,
        CHASSIS_GYRO
    } Chassis_ModeEnum;

    typedef struct
    {
        float vz; // forward/backward speed, forward is positive
        float vx; // left/right speed, right is positive
        float w;  // omega, clockwise(CW) is positive
    } Chassis_SpdDataTypeDef;

    typedef struct
    {
        Chassis_ModeEnum present_mode, last_mode;

        Chassis_SpdDataTypeDef last_ref;
        Chassis_SpdDataTypeDef gimbal_coordinate_ref;  // speed reference from gimbal perspective
        Chassis_SpdDataTypeDef chassis_coordinate_ref; // speed reference from chassis perspective
        Chassis_SpdDataTypeDef real_spd;               // chassis real speed
        float wheel_ref[4];                            // calc from chassis_coordinate_ref
        float wheel_fdb[4];                            // read from encoder
        float separate_rad;                            // gimbal chassis intersection angle
        PID_TypeDef Chassis_MotorSpdPID[4];
        PID_TypeDef Chassis_SpdfollowPID, Chassis_AngfollowPID;

        PCArgsType Power_Control_Args;
        float chassis_I[4];
        float chassis_W[4];
        float powercontrol_limit_k;
        float powercontrol_limit_b;
        float powerup_coef;
        float power_limit;

        float wheel_radius;
        float center_distance; // wheel center to chassis center distance
        float install_angle;
        float fly_angle;
    } Chassis_ControlTypeDef;

    Chassis_ControlTypeDef *Chassis_GetControlPtr(void);
    void Chassis_Init(void);
    void Chassis_ModeSet(Chassis_ModeEnum mode);
    void Chassis_SetMoveRef(float forward_back_ref, float left_right_ref);
    void OmniChassis_GetInstruct(void);
    void OmniChassis_CalcOutput(void);
    void OmniChassis_PowerControl(void);
    void OmniChassis_EstimateSpeed(void);

    static void Chassis_CalcMoveRef(void);
    static void Chassis_CalcOmniFollowRef(void);
    static void InverseKinematics_Translation(float omega[4], float vx, float vy, float wz, float wm);
    static void InverseKinematics_Rotation(float omega[4], float vx, float vy, float wm);
    static void Omni_FourHeadSetPosRef(float fdb);
    static void Chassis_LowRestEnergyProtect(void);
#ifdef __cplusplus
}
#endif

#endif
