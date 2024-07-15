/*
 * @Project: Infantry Code
 * @Author: GDDG08
 * @Date: 2024-07-06 15:57:21
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-16 01:52:04
 */

#include "app_chassis.h"
#include "app_gimbal.h"
#include "config_ctrl.h"
#include "lib_math.h"
#include "periph_cap.h"
#include "periph_motor_can.h"
#include "protocol_motor.h"
#include "protocol_referee.h"

Chassis_ControlTypeDef Chassis_Control;

/**
 * @brief      Gets the pointer to the chassis control object
 * @param      NULL
 * @retval     The pointer points to the chassis control object
 */
Chassis_ControlTypeDef *Chassis_GetControlPtr() { return &Chassis_Control; }

/**
 * @brief      Chassis control initialization
 * @param      NULL
 * @retval     NULL
 */
void Chassis_Init()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	chassis->present_mode = CHASSIS_STOP;
	chassis->last_mode = CHASSIS_STOP;
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis->last_wheel_ref[i] = 0.0f;
	}
	chassis->last_ref.forward_back_ref = 0;
	chassis->last_ref.left_right_ref = 0;
	chassis->last_ref.rotate_ref = 0;
	chassis->raw_ref.forward_back_ref = 0;
	chassis->raw_ref.left_right_ref = 0;
	chassis->raw_ref.rotate_ref = 0;
	chassis->move_ref.forward_back_ref = 0;
	chassis->move_ref.left_right_ref = 0;
	chassis->move_ref.rotate_ref = 0;
	Chassis_ParamInit();
}

/**
 * @brief      Chassis mode setting
 * @param      mode: Chassis mode
 * @retval     NULL
 */
void Chassis_ModeSet(Chassis_ModeEnum mode)
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	chassis->last_mode = chassis->present_mode;
	chassis->present_mode = mode;
}

void Chassis_SetMoveRef(float forward_back_ref, float left_right_ref)
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	chassis->last_ref.forward_back_ref = chassis->raw_ref.forward_back_ref;
	chassis->last_ref.left_right_ref = chassis->raw_ref.left_right_ref;
	chassis->raw_ref.forward_back_ref = forward_back_ref;
	chassis->raw_ref.left_right_ref = left_right_ref;
}

/**
 * @brief      calculate move speed ref
 * @param      NULL
 * @retval     NULL
 */
static float theta_rad;
static void Chassis_CalcMoveRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	theta_rad =
		-(Motor_GimbalYaw.encoder.limited_angle - Install_Angle) * PI / 180;
	float sin_tl = (float)sin(theta_rad);
	float cos_tl = (float)cos(theta_rad);
	chassis->move_ref.forward_back_ref =
		chassis->raw_ref.forward_back_ref * cos_tl -
		chassis->raw_ref.left_right_ref * sin_tl;
	chassis->move_ref.left_right_ref =
		chassis->raw_ref.forward_back_ref * sin_tl +
		chassis->raw_ref.left_right_ref * cos_tl;
}

/**
 * @brief      Calculation of chassis small gyroscope
 * @param      NULL
 * @retval     NULL
 */

float dead_zone = 3.0f;
static void Chassis_CalcOmmiFollowRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

	chassis->move_ref.rotate_ref = chassis->raw_ref.rotate_ref;
	float fdb = Math_Consequent_To_180(Install_Angle,
									   Motor_GimbalYaw.encoder.limited_angle);
	PID_SetFdb(&(chassis->Chassis_AngfollowPID), fdb);
	PID_SetRef(&(chassis->Chassis_AngfollowPID), Install_Angle);
	PID_SetFdb(&(chassis->Chassis_SpdfollowPID), Motor_GimbalYaw.encoder.speed);
	PID_SetRef(&(chassis->Chassis_SpdfollowPID),
			   PID_Calc(&(chassis->Chassis_AngfollowPID)));
	chassis->move_ref.rotate_ref += PID_Calc(&(chassis->Chassis_SpdfollowPID));
	if (abs(chassis->Chassis_AngfollowPID.ref -
			chassis->Chassis_AngfollowPID.fdb) < dead_zone)
	{
		chassis->move_ref.rotate_ref = 0;
	}
}

/*
 * 限制线速度，平动使用这个
 * @param omega 电机角速度输出数组
 * @param vx x轴线速度
 * @param vy y轴线速度
 * @param wz 期望角速度
 * @param wm 最大角速度
 */
void ConstrainedTranslationVelocity(float omega[4], float vx, float vy,
									float wz, float wm)
{
	float k;
	char i = GET_SIGN_BIT(wz);
	wz = MIN(fabs(wz), wm * 0.7f);
	k = (fabs(vx) + fabs(vy)) / (wm - wz);
	if (k == 0)
	{
		vx = 0;
		vy = 0;
	}
	else
	{
		vx /= MAX(k, 1);
		vy /= MAX(k, 1);
	}
	if (i == 1)
	{
		wz = -wz;
	}
	omega[0] = +vx + vy + wz;
	omega[1] = +vx - vy + wz;
	omega[2] = -vx - vy + wz;
	omega[3] = -vx + vy + wz;
}

/*
 * 限制角速度，小陀螺使用这个
 * @param omega 电机角速度输出数组
 * @param vx x轴线速度
 * @param vy y轴线速度
 * @param wm 最大角速度
 */
int gyro_dir = 1;
void ConstrainedGyroVelocity(float omega[4], float vx, float vy, float wm)
{
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	float wz;
	float k;
	k = (fabs(vx) + fabs(vy)) / (0.7f * wm);
	if (k == 0)
	{
		vx = 0;
		vy = 0;
	}
	else
	{
		vx /= MAX(k, 1);
		vy /= MAX(k, 1);
	}
	if (boardcom->gyro_dir == CW)
	{
		gyro_dir = 1;
	}
	else if (boardcom->gyro_dir == CCW)
	{
		gyro_dir = -1;
	}
	wz = (wm - (fabs(vx) + fabs(vy))) * gyro_dir;
	omega[0] = +vx + vy + wz;
	omega[1] = +vx - vy + wz;
	omega[2] = -vx - vy + wz;
	omega[3] = -vx + vy + wz;
}

static void Chassis_CalcWheelRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	Referee_DataTypeDef *referee = Referee_GetDataPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	for (int i = 0; i < 4; i++)
	{
		chassis->last_wheel_ref[i] = PID_GetRef(&chassis->Chassis_MotorSpdPID[i]);
	}
	float f_b_ref = chassis->move_ref.forward_back_ref;
	float l_r_ref = chassis->move_ref.left_right_ref;
	float gyro_ref = chassis->move_ref.rotate_ref;
	switch (chassis->present_mode)
	{
	case CHASSIS_STOP:
		for (int i = 0; i < 4; i++)
		{
			chassis->wheel_ref[i] = 0;
		}
		break;
	case CHASSIS_NORMAL:
		ConstrainedTranslationVelocity(
			chassis->wheel_ref, l_r_ref, f_b_ref, gyro_ref,
			referee->chassis_power_limit * SC_Limit_K + SC_Limit_B);
		break;
	case CHASSIS_GYRO:
		// ConstrainedGyroVelocity(chassis->wheel_ref, l_r_ref, f_b_ref,
		// referee->chassis_power_limit * SC_Limit_K + SC_Limit_B);
		//  to test
		if (boardcom->power_limit_mode == POWER_UNLIMIT)
		{
			ConstrainedTranslationVelocity(chassis->wheel_ref, l_r_ref, f_b_ref,
										   200 * gyro_dir, 450);
		}
		else
		{
			if (l_r_ref == 0 && f_b_ref == 0)
			{
				ConstrainedTranslationVelocity(
					chassis->wheel_ref, 0, 0,
					(referee->chassis_power_limit * 1.9 + 180), 460);
			}
			ConstrainedGyroVelocity(chassis->wheel_ref, l_r_ref, f_b_ref,
									referee->chassis_power_limit * 1.9 + 130);
		}
		break;
	default:
		return;
	}
	for (int i = 0; i < 4; i++)
	{
		PID_SetRef(&chassis->Chassis_MotorSpdPID[i], chassis->wheel_ref[i]);
	}
}

/**
 * @brief      Calculation of chassis control quantity
 * @param      NULL
 * @retval     NULL
 */
float PC_Limit = 0.0f;
void OmmiChassis_Output()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	Referee_DataTypeDef *referee = Referee_GetDataPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	Cap_DataTypeDef *cap = Cap_GetDataPtr();

	switch (chassis->present_mode)
	{
	case CHASSIS_STOP:
		chassis->raw_ref.forward_back_ref = 0;
		chassis->raw_ref.left_right_ref = 0;
		chassis->raw_ref.rotate_ref = 0;
		chassis->move_ref.forward_back_ref = 0;
		chassis->move_ref.left_right_ref = 0;
		chassis->move_ref.rotate_ref = 0;
		break;
	case CHASSIS_NORMAL:
		Chassis_CalcMoveRef();		 // Headless translation solution
		Chassis_CalcOmmiFollowRef(); // Chassis following soGYROlution
		break;
	case CHASSIS_GYRO:
		Chassis_CalcMoveRef(); // Headless translation solution
		break;
	default:
		return;
	}

	Chassis_CalcWheelRef();
	if (cap->rest_energy < 10)
	{
		memset(chassis->wheel_ref, 0, sizeof(chassis->wheel_ref));
	}
	for (int i = 0; i < 4; i++)
	{
		PID_SetFdb(
			&chassis->Chassis_MotorSpdPID[i],
			Motor_ChassisMotors.motor_handle[i]->encoder.speed); // 速度环 pid
		chassis->chassis_I[i] = PID_Calc(&chassis->Chassis_MotorSpdPID[i]) * 20.0f /
								16384.0f; // 读取电流，单位 A
		chassis->chassis_W[i] =
			(Motor_ChassisMotors.motor_handle[i]->encoder.speed * 14.0f /
			 9.549296596425384f); // 读取转速（无减速比），单位 rad/s
	}

	PC_Limit = referee->chassis_power_limit * PC_Limit_K + PC_Limit_B;
	powerctrl_limit_watch = PC_Limit;

	if (boardcom->power_limit_mode == POWER_LIMIT &&
		boardcom->cap_speedup_flag == CAP_NORMAL)
	{
		PowerControl(Power_Control_Args, PC_Limit, chassis->chassis_I,
					 chassis->chassis_W);
	}
	else if (boardcom->power_limit_mode == POWER_LIMIT &&
			 boardcom->cap_speedup_flag == CAP_SPEEDUP)
	{
		PowerControl(Power_Control_Args, PC_Limit * PowerUp_Coef,
					 chassis->chassis_I, chassis->chassis_W);
	}
	else if (boardcom->power_limit_mode == POWER_UNLIMIT)
	{
		;
	}

	for (int i = 0; i < 4; i++)
	{
		Motor_SetOutput(
			Motor_ChassisMotors.motor_handle[i],
			chassis->chassis_I[i] / 20.0f * 16384.0f); // 设置输出，注意单位
	}
	Motor_CAN_SendGroupOutput(&Motor_ChassisMotors);
}
