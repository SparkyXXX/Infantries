/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-05-24 16:03:18
 */
//STAR DUST
#include "config_ctrl.h"
#include "test_ctrl.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_ui.h"
#include "protocol_referee.h"
#include "protocol_board.h"
#include "protocol_motor.h"
#include "lib_power_ctrl.h"
#include "periph_motor_can.h"
#include "periph_cap.h"
#include "alg_pid.h"
#include "sys_dwt.h"

/********** START OF PARAMETER SETTING **********/
uint8_t CPU_Clock = 170; //(MHz)
PCArgsType Power_Control_Args = {
    .mp = 0.01602f,
    .kp = {0.06021f,     8.018e-06f},
    .cp = 2.919f,
	.lp = 0.003365f,
    .mn = {-1.693e-06f, -0.01122f  },
    .kn = {0.1234f,     -3.899e-05f},
    .cn = 2.561f,
	.ln = 0.02422f,
};

float PC_Limit_K = 0.9559f;
float PC_Limit_B = -14.5318f;
float Wheel_Dec_Ratio = 14.0f;
float PowerUp_Coef = 2;
float Install_Angle = -150.0f;
float FLY_ANGLE = -60.0f;

void Chassis_ParamInit(void)
{
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    for (int i = 0; i < 4; i++)
    {
        PID_Init(&(chassis->Chassis_MotorSpdPID[i]), 75.0f, 10.0f, 0.0f, 0.0f, 500.0f, 10000.0f, 159.154922f, 159.154922f);
		//                                           kp    ki    kd    kf  sum_max  output_max   d_frq     kf_frq
    }
#if IF_FOLLOW == FOLLOW
    PID_Init(&(chassis->Chassis_SpdfollowPID), 8.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 1000.0f, 159.154922f, 159.154922f);
    PID_Init(&(chassis->Chassis_AngfollowPID), 2.0f, 0.0f, 0.0f, 0.0f, 200.0f, 120.0f, 159.154922f, 159.154922f);
#endif
#if IF_FOLLOW == NO_FOLLOW
    PID_Init(&(chassis->Chassis_SpdfollowPID), 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 1000.0f, 159.154922f, 159.154922f);
    PID_Init(&(chassis->Chassis_AngfollowPID), 0.0f, 0.0f, 0.0f, 0.0f, 200.0f, 120.0f, 159.154922f, 159.154922f);
#endif
	//                                          kp    ki    kd    kf  sum_max  output_max   d_frq     kf_frq
}

void GimbalYaw_ParamInit(void)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
	PID_Init(&(gimbalyaw->spd_no_auto), 12000.0f, 8000.0f, 30.0f, 100000.0f, 5000.0f, 25000.0f, 100000.0f, 159.154922f);
    PID_Init(&(gimbalyaw->pos_no_auto), 18.5f,    20.0f,   0.15f, 0.0f,   0.1f,    9.0f,     500.0f,    159.154922f);
	//                                   kp        ki        kd      kf       sum_max  output_max   d_frq     kf_frq
	PID_Init(&(gimbalyaw->spd_armor), 12000.0f, 8000.0f, 30.0f, 100000.0f, 5000.0f, 25000.0f, 100000.0f, 159.154922f);
    PID_Init(&(gimbalyaw->pos_armor), 18.5f,    20.0f,   0.15f, 0.0f,    0.1f,    9.0f,     500.0f,    159.154922f);
	//                                   kp        ki        kd      kf       sum_max  output_max   d_frq     kf_frq
	PID_Init(&(gimbalyaw->spd_big_energy), 7860.98773778112f, 24876.1388107406f, 10.9682598257439f, 0.0f,  5000.0f, 30000.0f, 32.78987f, 1.0f);
    PID_Init(&(gimbalyaw->pos_big_energy), 25.1943607025437f, 1.81996401951577f, 0.335675508f, 0.0f, 0.001f, 6.0f, 249.999985f, 159.154922f);
	//                                     kp    ki    kd    kf  sum_max  output_max   d_frq     kf_frq

	PID_Init(&(gimbalyaw->spd_small_energy), 7860.98773778112f, 24876.1388107406f, 10.9682598257439f, 0.0f,  5000.0f, 30000.0f, 32.78987f, 1.0f);
    PID_Init(&(gimbalyaw->pos_small_energy), 25.1943607025437f, 1.81996401951577f, 0.335675508f, 0.0f, 0.001f, 6.0f, 249.999985f, 159.154922f);
	//                                     	       kp    ki    kd    kf  sum_max  output_max   d_frq     kf_frq
	Filter_Lowpass_Init(200.0f, &(gimbalyaw->spd_ref_filter));
}
/********** END OF PARAMETER SETTING **********/

Motor_GroupDataTypeDef *Motor_groupHandle[2];
Motor_GroupDataTypeDef Motor_ChassisMotors;
Motor_GroupDataTypeDef Motor_GimbalMotors;

Motor_DataTypeDef Motor_ForwardLeft;
Motor_DataTypeDef Motor_ForwardRight;
Motor_DataTypeDef Motor_BackwardRight;
Motor_DataTypeDef Motor_BackwardLeft;
Motor_DataTypeDef Motor_GimbalYaw;

static void Init_AllMotors(void)
{
    Motor_groupHandle[0] = &Motor_ChassisMotors;
    Motor_InitGroup(&Motor_ChassisMotors, 4, &hfdcan1, 0x200);
    Motor_Init(&Motor_ForwardLeft, FORWARD_LEFT_CAN_ID);
    Motor_Init(&Motor_ForwardRight, FORWARD_RIGHT_CAN_ID);
    Motor_Init(&Motor_BackwardRight, BACKWARD_RIGHT_CAN_ID);
    Motor_Init(&Motor_BackwardLeft, BACKWARD_LEFT_CAN_ID);
    Motor_ChassisMotors.motor_handle[FORWARD_LEFT_CAN_ID - 0x201] = &Motor_ForwardLeft;
    Motor_ChassisMotors.motor_handle[FORWARD_RIGHT_CAN_ID - 0x201] = &Motor_ForwardRight;
    Motor_ChassisMotors.motor_handle[BACKWARD_RIGHT_CAN_ID - 0x201] = &Motor_BackwardRight;
    Motor_ChassisMotors.motor_handle[BACKWARD_LEFT_CAN_ID - 0x201] = &Motor_BackwardLeft;

    Motor_groupHandle[1] = &Motor_GimbalMotors;
    Motor_InitGroup(&Motor_GimbalMotors, 2, &hfdcan1, 0x1FF);
    Motor_Init(&Motor_GimbalYaw, YAW_CAN_ID);
    Motor_GimbalMotors.motor_handle[YAW_CAN_ID - 0x205] = &Motor_GimbalYaw;
}

void Init_All(void)
{
    DWT_Init(CPU_Clock);
    Referee_Init(&huart2);
    Init_AllMotors();
    BoardCom_Init();
    FDCAN_IntFilterAndStart(&hfdcan1);
    FDCAN_IntFilterAndStart(&hfdcan2);
    FDCAN_IntFilterAndStart(&hfdcan3);

    Cap_Init();
    Chassis_Init();
    GimbalYaw_Init();
    UI_Init();
}
