/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-16 04:16:18
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "config_ctrl.h"
#include "protocol_referee.h"
#include "protocol_board.h"
#include "alg_pid.h"
#include "lib_math.h"
#include "cmsis_os.h"

#define FORWARD_LEFT 0
#define FORWARD_RIGHT 1
#define BACKWARD_RIGHT 2
#define BACKWARD_LEFT 3
	

	
    typedef enum
    {
        CHASSIS_STOP = 0u,
        CHASSIS_NORMAL,
        CHASSIS_GYRO,
        CHASSIS_BACKGYRO
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

        float last_motor_ref[4];
        float last_yaw_ref;
		
        PID_TypeDef Chassis_MotorSpdPID[4];
        PID_TypeDef Chassis_SpdfollowPID, Chassis_AngfollowPID;
		
#if ROBOT_NAME == SWING_DANCE
		Math_SlopeParamTypeDef Chassis_Move_Slope, Chassis_Gyro_Slope, Chassis_Follow_Slope;
#endif
    } Chassis_ControlTypeDef;

		
		
//    typedef struct
//    {
//      struct {
//        uint8_t x   :2;
//        uint8_t y   :2;
//        uint8_t z   :2;
//        uint8_t None:2;
//			} sign;
//			float k, tmp;
//    } GetSpeedRefs_DataTypeDef;
		
	extern float PC_Limit_K;
	extern float PC_Limit_B;
	extern float PowerUp_Coef;
	extern float Gyro_Speed;
    extern float Install_Angle;
    extern float Forward_Left_Compensate;
    extern float Forward_Right_Compensate;
    extern float Backward_Right_Compensate;
    extern float Backward_Left_Compensate;
	
#if ROBOT_NAME == SWING_DANCE
	extern float Protect_Wheel_Speed_RPM;	
	
	extern float Chassis_Move_Acc;
	extern float Chassis_Move_Dec;
	extern float Chassis_Gyro_Acc;
	extern float Chassis_Gyro_Dec;
	extern float Chassis_Follow_Acc;
	extern float Chassis_Follow_Dec;
#endif

    Chassis_ControlTypeDef *Chassis_GetControlPtr(void);
    void Chassis_Init(void);
    void Chassis_ModeSet(Chassis_ModeEnum mode);
    void Chassis_SetMoveRef(float forward_back_ref, float left_right_ref);
    void Chassis_Output(void);
    static void Chassis_CalcMoveRef(void);
    static void Chassis_CalcFollowRef(void);
    static void Chassis_CalcGyroRef(void);
    static void Chassis_CalcWheelRef(void);

#ifdef __cplusplus
}
#endif

#endif
