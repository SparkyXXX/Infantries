/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-16 04:12:08
 */

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#include "config_ctrl.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_power.h"
#include "protocol_referee.h"
#include "protocol_motor.h"
#include "lib_math.h"
#include "periph_motor_can.h"
#include "periph_cap.h"

Chassis_ControlTypeDef Chassis_Control;

float wheel_spd[4] = {0};

/**
 * @brief      Gets the pointer to the chassis control object
 * @param      NULL
 * @retval     The pointer points to the chassis control object
 */
Chassis_ControlTypeDef *Chassis_GetControlPtr()
{
	return &Chassis_Control;
}

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
	theta_rad = -(Motor_GimbalYaw.encoder.limited_angle - Install_Angle) * PI / 180;
	float sin_tl = (float)sin(theta_rad);
	float cos_tl = (float)cos(theta_rad);
	chassis->move_ref.forward_back_ref = chassis->raw_ref.forward_back_ref * cos_tl - chassis->raw_ref.left_right_ref * sin_tl;
	chassis->move_ref.left_right_ref = chassis->raw_ref.forward_back_ref * sin_tl + chassis->raw_ref.left_right_ref * cos_tl;
}

/**
 * @brief      Calculation of chassis small gyroscope (gyro & reverse_gyro)
 * @param      NULL
 * @retval     NULL
 */
static void Chassis_CalcGyroRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	float gyro_spd = Gyro_Speed;
	if (chassis->last_mode != CHASSIS_GYRO)
	{
		gyro_spd = 0;
	}
	chassis->move_ref.rotate_ref = gyro_spd;
}

static void Chassis_CalcReverseGyroRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	float gyro_spd = -Gyro_Speed;
	if (chassis->last_mode != CHASSIS_REVERSE_GYRO)
	{
		gyro_spd = 0;
	}
	chassis->move_ref.rotate_ref = gyro_spd;
}


// zch_gyro_cal

#define WheelRadius (1.0f)
#define WheelBase (1.0f)
#define GET_SIGN_BIT(x) (((*(uint32_t *)(&(x))) >> 31) & 1)
#define SET_SIGN_POS(x) ((*(uint32_t *)(&(x))) &= 0x7FFFFFFFU)
void GetSpeedRefs(float omega[4], float vx, float vy, float wz, float omegaM)
{
	static struct
	{
		uint8_t x : 2;
		uint8_t y : 2;
		uint8_t z : 2;
		uint8_t None : 2;
	} sign;

	static float k, tmp;

		sign.x = GET_SIGN_BIT(vx);
	sign.y = GET_SIGN_BIT(vy);
	sign.z = GET_SIGN_BIT(wz);

	SET_SIGN_POS(vx);
	SET_SIGN_POS(vy);
	SET_SIGN_POS(wz);

	tmp = WheelRadius * omegaM / WheelBase * 0.8f;
	wz = MIN(wz, tmp);

	k = 1.0f;

	tmp = MAX(vx, vy) / (omegaM * WheelRadius - wz * WheelBase);
	k = MAX(k, tmp);

	tmp = (vx + vy) / (omegaM * WheelRadius);
	k = MAX(k, tmp);

	vx = vx / k * (1 - sign.x * 2);
	vy = vy / k * (1 - sign.y * 2);
	wz = wz * (1 - sign.z * 2);

	omega[0] = (vx + vy - wz * WheelBase) / WheelRadius;
	omega[1] = (vx - vy - wz * WheelBase) / WheelRadius;
	omega[2] = (-vx - vy - wz * WheelBase) / WheelRadius;
	omega[3] = (-vx + vy - wz * WheelBase) / WheelRadius;
}

void GetSpeedRefs_Gyro(float omega[4], float vx, float vy, float omegaM)
{
	static float wz = 0.0f;
	static struct
	{
		uint8_t x : 2;
		uint8_t y : 2;
		uint8_t z : 2;
		uint8_t None : 2;
	} sign;
	static float k, tmp;
	sign.x = GET_SIGN_BIT(vx);
	sign.y = GET_SIGN_BIT(vy);
	SET_SIGN_POS(vx);
	SET_SIGN_POS(vy);
	k = 1.0f;

	tmp = (vx + vy) / (omegaM * WheelRadius * 0.75);
	k = MAX(k, tmp);

	vx /= k;
	vy /= k;

	wz = WheelRadius * omegaM / WheelBase;
	tmp = (omegaM * WheelRadius - MAX(vx, vy)) / (WheelBase);
	wz = MIN(wz, tmp);

	vx *= (1 - sign.x * 2);
	vy *= (1 - sign.y * 2);
	omega[0] = (vx + vy - wz * WheelBase) / WheelRadius;
	omega[1] = (vx - vy - wz * WheelBase) / WheelRadius;
	omega[2] = (-vx - vy - wz * WheelBase) / WheelRadius;
	omega[3] = (-vx + vy - wz * WheelBase) / WheelRadius;
}



/**
 * @brief      Chassis following solution
 * @param      NULL
 * @retval     NULL
 */
static void Chassis_CalcMecFollowRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();

	chassis->move_ref.rotate_ref = chassis->raw_ref.rotate_ref;
	float fdb = Math_Consequent_To_180(Install_Angle, Motor_GimbalYaw.encoder.limited_angle);
	PID_SetFdb(&(chassis->Chassis_AngfollowPID), fdb);
	PID_SetRef(&(chassis->Chassis_AngfollowPID), Install_Angle);
	PID_SetRef(&(chassis->Chassis_SpdfollowPID), PID_Calc(&(chassis->Chassis_AngfollowPID)));
	chassis->move_ref.rotate_ref += PID_Calc(&(chassis->Chassis_SpdfollowPID));
	chassis->last_yaw_ref = gimbalyaw->yaw_ref;
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
		if (boardcom->power_limit_mode == POWER_LIMITED)
		{
			GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, -gyro_ref, referee->chassis_power_limit * 1.818 + 250);
		}
		else
		{
			GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, -gyro_ref, 500);
		}
		for (int i = 0; i < 4; i++)
		{
			PID_SetRef(&chassis->Chassis_MotorSpdPID[i], wheel_spd[i]);
		}
		break;
	case CHASSIS_GYRO:
		if (boardcom->power_limit_mode == POWER_UNLIMITED)
		{
			wheel_spd[0] = f_b_ref * 5.0f + l_r_ref * 5.0f + -Forward_Left_Compensate * gyro_ref * 1;
			wheel_spd[1] = -f_b_ref * 5.0f + l_r_ref * 5.0f + -Forward_Right_Compensate * gyro_ref * 1;
			wheel_spd[2] = -f_b_ref * 5.0f - l_r_ref * 5.0f + -Backward_Right_Compensate * gyro_ref * 1;
			wheel_spd[3] = f_b_ref * 5.0f - l_r_ref * 5.0f + -Backward_Left_Compensate * gyro_ref * 1;
			//					GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, gyro_ref * 0.2f, 600);
		}
		else if (boardcom->cap_speedup_flag == CAP_SPEEDUP)
		{
			GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, gyro_ref, referee->chassis_power_limit * PowerUp_Coef * 1.818 + 250);
		}
		else
		{
			GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, gyro_ref, referee->chassis_power_limit * PowerUp_Coef * 1.818 + 250);
		}

		for (int i = 0; i < 4; i++)
		{
			PID_SetRef(&chassis->Chassis_MotorSpdPID[i], wheel_spd[i]);
		}
		break;
	case CHASSIS_REVERSE_GYRO:
	GetSpeedRefs_Gyro(wheel_spd, -l_r_ref, -f_b_ref, referee->chassis_power_limit * 1.818 + 250);
			for(int i = 0; i < 4; i ++) {
				PID_SetRef(&chassis->Chassis_MotorSpdPID[i], -wheel_spd[i]);
			}
			break;
	default:
		return;
	}
}

/**
 * @brief      Calculation of chassis control quantity
 * @param      NULL
 * @retval     NULL
 */
float PC_Limit = 0.0f;
void MecChassis_Output()
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
		Chassis_CalcMoveRef();		// Headless translation solution
		Chassis_CalcMecFollowRef(); // Chassis following soGYROlution
		break;
	case CHASSIS_GYRO:
		Chassis_CalcMoveRef(); // Headless translation solution
		Chassis_CalcGyroRef(); // Solution of gyro
		break;
	case CHASSIS_REVERSE_GYRO:
		Chassis_CalcMoveRef();		  // Headless translation solution
		Chassis_CalcReverseGyroRef(); // Solution of Reverse gyro
		break;
	default:
		return;
	}

	Chassis_CalcWheelRef();

	for (int i = 0; i < 4; i++)
	{
		PID_SetFdb(&chassis->Chassis_MotorSpdPID[i], Motor_ChassisMotors.motor_handle[i]->encoder.speed);		   // 速度环 pid
		chassis->chassis_I[i] = PID_Calc(&chassis->Chassis_MotorSpdPID[i]) * 20.0f / 16384.0f;					   // 读取电流，单位 A
		chassis->chassis_W[i] = (Motor_ChassisMotors.motor_handle[i]->encoder.speed * 14.0f / 9.549296596425384f); // 读取转速（无减速比），单位 rad/s
	}

	PC_Limit = referee->chassis_power_limit * PC_Limit_K + PC_Limit_B;
	powerctrl_limit_watch = PC_Limit;

	if (boardcom->power_limit_mode == POWER_LIMIT && boardcom->cap_speedup_flag == CAP_NORMAL)
	{
		PowerControl(Power_Control_Args, PC_Limit, chassis->chassis_I, chassis->chassis_W);
	}
	else if (boardcom->power_limit_mode == POWER_LIMIT && boardcom->cap_speedup_flag == CAP_SPEEDUP)
	{
		PowerControl(Power_Control_Args, PC_Limit * PowerUp_Coef, chassis->chassis_I, chassis->chassis_W);
	}
	else if (boardcom->power_limit_mode == POWER_UNLIMIT)
	{
		;
	}

	for (int i = 0; i < 4; i++)
	{
		Motor_SetOutput(Motor_ChassisMotors.motor_handle[i], chassis->chassis_I[i] / 20.0f * 16384.0f); // 设置输出，注意单位
	}
	Motor_CAN_SendGroupOutput(&Motor_ChassisMotors);
}
