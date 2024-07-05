/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-05-07 01:14:26
 */

#include "config_ctrl.h"
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
float PC_Limit_K = 0.8826;
float PC_Limit_B = -9.8479;
float SC_Limit_K = 2.236f;
float SC_Limit_B = 120.0f;
float Wheel_Dec_Ratio = 14.0f;
float PowerUp_Coef = 2;

float Install_Angle = 90.0f;
float Forward_Left_Compensate = 1.0f;
float Forward_Right_Compensate = 1.0f;
float Backward_Right_Compensate = 1.0f;
float Backward_Left_Compensate = 1.0f;

void Chassis_ParamInit(void)
{
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    for (int i = 0; i < 4; i++)
    {
        PID_Init(&chassis->Chassis_MotorSpdPID[i], PID_POSITION, 75.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 10000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    }
    PID_Init(&chassis->Chassis_SpdfollowPID, PID_POSITION, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 1000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&chassis->Chassis_AngfollowPID, PID_POSITION, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 200.0f, 120.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    //                                          mode        p     i     d     kf1   kf2   kf3   sum_max  output_max d_fil  kf1_fil  kf2_fil  kf3_fil
}

void GimbalYaw_ParamInit(void)
{
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();

    PID_Init(&gimbalyaw->spd_no_auto, PID_POSITION, 18000.0f, 10.0f, 8000.0f, 50000.0f, 0.0f, 0.0f, 300.0f, 30000.0f, 0.1f, 0.1f, 0.1f, 0.1f);
    PID_Init(&gimbalyaw->pos_no_auto, PID_POSITION, 0.23f, 0.002f, 2.0f, 1.2f, 0.0f, 0.0f, 50.0f, 20.0f, 0.1f, 0.1f, 0.1f, 0.1f);

    PID_Init(&gimbalyaw->spd_armor, PID_POSITION, 18000.0f, 60.0f, 10000.0f, 50000.0f, 0.0f, 0.0f, 15.0f, 30000.0f, 0.1f, 0.1f, 0.1f, 0.1f);
    PID_Init(&gimbalyaw->pos_armor, PID_POSITION, 0.35f, 0.005f, 4.0f, 15.0f, 0.0f, 0.0f, 80.0f, 20.0f, 0.1f, 0.05f, 0.1f, 0.1f);

    PID_Init(&gimbalyaw->spd_big_energy, PID_POSITION, 18000.0f, 150.0f, 8000.0f, 50000.0f, 0.0f, 0.0f, 100.0f, 30000.0f, 0.1f, 0.1f, 0.1f, 0.1f);
    PID_Init(&gimbalyaw->pos_big_energy, PID_POSITION, 0.23f, 0.005f, 2.0f, 1.2f, 0.0f, 0.0f, 50.0f, 20.0f, 0.1f, 0.1f, 0.1f, 0.1f);

    PID_Init(&gimbalyaw->spd_small_energy, PID_POSITION, 18000.0f, 150.0f, 8000.0f, 50000.0f, 0.0f, 0.0f, 100.0f, 30000.0f, 0.1f, 0.1f, 0.1f, 0.1f);
    PID_Init(&gimbalyaw->pos_small_energy, PID_POSITION, 0.23f, 0.005f, 2.0f, 1.2f, 0.0f, 0.0f, 50.0f, 20.0f, 0.1f, 0.1f, 0.1f, 0.1f);
    //                                        mode          p     i     d      kf1   kf2   kf3   sum_max  output_max d_fil  kf1_fil  kf2_fil  kf3_fil
}
/********** END OF PARAMETER SETTING **********/

uint16_t referee_power_limit_watch;
uint16_t realtime_power_watch;
uint8_t buffer_energy_watch;
uint8_t rest_energy_watch;
float powerctrl_limit_watch;
float initial_speed_watch;
float yaw_pos_ref_watch;
float yaw_pos_fdb_watch;
float yaw_pos_error_watch;

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
