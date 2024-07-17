/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 01:50:44
 */

#include "app_chassis.h"
#include "app_gimbal.h"
#include "config_ctrl.h"
#include "lib_math.h"
#include "periph_cap.h"
#include "protocol_board.h"
#include "protocol_referee.h"

Chassis_ControlTypeDef Chassis_Control;

Chassis_ControlTypeDef *Chassis_GetControlPtr()
{
	return &Chassis_Control;
}

void Chassis_Init()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	chassis->present_mode = CHASSIS_STOP;
	chassis->last_mode = CHASSIS_STOP;
	chassis->last_ref.vz = 0;
	chassis->last_ref.vx = 0;
	chassis->last_ref.w = 0;
	chassis->gimbal_coordinate_ref.vz = 0;
	chassis->gimbal_coordinate_ref.vx = 0;
	chassis->gimbal_coordinate_ref.w = 0;
	chassis->chassis_coordinate_ref.vz = 0;
	chassis->chassis_coordinate_ref.vx = 0;
	chassis->chassis_coordinate_ref.w = 0;
	Chassis_ParamInit();
}

void Chassis_ModeSet(Chassis_ModeEnum mode)
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	chassis->last_mode = chassis->present_mode;
	chassis->present_mode = mode;
}

void Chassis_SetMoveRef(float vz, float vx)
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	chassis->last_ref.vz = chassis->gimbal_coordinate_ref.vz;
	chassis->last_ref.vx = chassis->gimbal_coordinate_ref.vx;
	chassis->gimbal_coordinate_ref.vz = vz;
	chassis->gimbal_coordinate_ref.vx = vx;
}

void OmniChassis_GetInstruct()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	switch (boardcom->chassis_mode)
	{
	case CHASSIS_CTRL_STOP:
	{
		Chassis_ModeSet(CHASSIS_STOP);
		Chassis_SetMoveRef(0, 0);
		break;
	}
	case CHASSIS_CTRL_NORMAL:
	{
		Chassis_ModeSet(CHASSIS_NORMAL);
		Chassis_SetMoveRef(boardcom->chassis_fb_ref, boardcom->chassis_lr_ref);
		break;
	}
	case CHASSIS_CTRL_GYRO:
	{
		Chassis_ModeSet(CHASSIS_GYRO);
		Chassis_SetMoveRef(boardcom->chassis_fb_ref, boardcom->chassis_lr_ref);
		break;
	}
	default:
		return;
	}
}

void OmniChassis_CalcOutput()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	Referee_DataTypeDef *referee = Referee_GetDataPtr();
	switch (chassis->present_mode)
	{
	case CHASSIS_STOP:
		chassis->gimbal_coordinate_ref.vz = 0;
		chassis->gimbal_coordinate_ref.vx = 0;
		chassis->gimbal_coordinate_ref.w = 0;
		chassis->chassis_coordinate_ref.vz = 0;
		chassis->chassis_coordinate_ref.vx = 0;
		chassis->chassis_coordinate_ref.w = 0;
		for (int i = 0; i < 4; i++)
		{
			chassis->wheel_ref[i] = 0;
		}
		break;
	case CHASSIS_NORMAL:
		Chassis_CalcMoveRef();		 // Headless translation solution
		Chassis_CalcOmniFollowRef(); // Chassis following soGYROlution
		if (boardcom->power_limit_mode == POWER_UNLIMIT)
		{
			InverseKinematics_Translation(chassis->wheel_ref,
										  chassis->chassis_coordinate_ref.vx, chassis->chassis_coordinate_ref.vz,
										  chassis->chassis_coordinate_ref.w, 1500);
		}
		else
		{
			InverseKinematics_Translation(chassis->wheel_ref,
										  chassis->chassis_coordinate_ref.vx, chassis->chassis_coordinate_ref.vz,
										  chassis->chassis_coordinate_ref.w, referee->chassis_power_limit * 1.9 + 130);
		}
		break;
	case CHASSIS_GYRO:
		Chassis_CalcMoveRef(); // Headless translation solution
		if (boardcom->power_limit_mode == POWER_UNLIMIT)
		{
			InverseKinematics_Translation(chassis->wheel_ref,
										  chassis->chassis_coordinate_ref.vx, chassis->chassis_coordinate_ref.vz,
										  200 * boardcom->gyro_dir, 450);
		}
		else
		{
			if (boardcom->cap_speedup_flag == CAP_NORMAL)
			{
				InverseKinematics_Rotation(chassis->wheel_ref,
										   chassis->chassis_coordinate_ref.vx, chassis->chassis_coordinate_ref.vz,
										   referee->chassis_power_limit * 1.7 + 110);
			}
			else if (boardcom->cap_speedup_flag == CAP_SPEEDUP)
			{
				InverseKinematics_Translation(chassis->wheel_ref,
											  chassis->chassis_coordinate_ref.vx, chassis->chassis_coordinate_ref.vz,
											  (referee->chassis_power_limit * 0.5 + 340) * boardcom->gyro_dir, (referee->chassis_power_limit * 0.5 + 440));
			}
		}
		break;
	default:
		break;
	}
	Chassis_LowRestEnergyProtect();
}

void OmniChassis_PowerControl()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	Referee_DataTypeDef *referee = Referee_GetDataPtr();
	for (int i = 0; i < 4; i++)
	{
		PID_SetFdb(&(chassis->Chassis_MotorSpdPID[i]), Motor_ChassisMotors.motor_handle[i]->encoder.speed);													// 速度环 pid
		chassis->chassis_I[i] = (PID_Calc(&(chassis->Chassis_MotorSpdPID[i]))) * 20.0f / 16384.0f;															// 读取电流，单位 A
		chassis->chassis_W[i] = (Motor_ChassisMotors.motor_handle[i]->encoder.speed * Motor_ChassisMotors.motor_handle[i]->dec_ratio / 9.549296596425384f); // 读取转速（无减速比），单位 rad/s
	}

	chassis->power_limit = referee->chassis_power_limit * chassis->powercontrol_limit_k + chassis->powercontrol_limit_b;

	if (boardcom->power_limit_mode == POWER_LIMIT && boardcom->cap_speedup_flag == CAP_NORMAL)
	{
		PowerControl(chassis->Power_Control_Args, chassis->power_limit, chassis->chassis_I, chassis->chassis_W);
	}
	else if (boardcom->power_limit_mode == POWER_LIMIT && boardcom->cap_speedup_flag == CAP_SPEEDUP)
	{
		PowerControl(chassis->Power_Control_Args, chassis->power_limit * chassis->powerup_coef, chassis->chassis_I, chassis->chassis_W);
	}
	else if (boardcom->power_limit_mode == POWER_UNLIMIT && boardcom->cap_speedup_flag == CAP_SPEEDUP && chassis->present_mode == CHASSIS_GYRO)
	{
		PowerControl(chassis->Power_Control_Args, chassis->power_limit * chassis->powerup_coef, chassis->chassis_I, chassis->chassis_W);
	}
	for (int i = 0; i < 4; i++)
	{
		Motor_SetOutput(Motor_ChassisMotors.motor_handle[i], chassis->chassis_I[i] / 20.0f * 16384.0f); // 设置输出，注意单�?
	}
	if (boardcom_decoded_count > BOARDCOM_TIMEOUT_VALUE)
	{
		for (int i = 0; i < 4; i++)
		{
			Motor_SetOutput(Motor_ChassisMotors.motor_handle[i], 0.0f);
		}
	}
	else
	{
		boardcom_decoded_count++;
	}
	Motor_CAN_SendGroupOutput(&Motor_ChassisMotors);
}

void OmniChassis_EstimateSpeed()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	for (int i = 0; i < 4; i++)
	{
		chassis->wheel_fdb[i] = chassis->Chassis_MotorSpdPID[i].fdb;
	}
	static float temp_vz = 0.25 * 1.414f * chassis->wheel_radius * (+chassis->wheel_fdb[0] - chassis->wheel_fdb[1] - chassis->wheel_fdb[2] + chassis->wheel_fdb[3]) * 0.10472f;					// m/s
	static float temp_vx = 0.25 * 1.414f * chassis->wheel_radius * (+chassis->wheel_fdb[0] + chassis->wheel_fdb[1] - chassis->wheel_fdb[2] - chassis->wheel_fdb[3]) * 0.10472f;					// m/s
	chassis->real_spd.w = 0.25 * chassis->wheel_radius / chassis->center_distance * (chassis->wheel_fdb[0] + chassis->wheel_fdb[1] + chassis->wheel_fdb[2] + chassis->wheel_fdb[3]) * 0.10472f; // rad/s (1 rpm = 0.10472 rad/s)
	chassis->real_spd.vz = -sin(chassis->separate_rad) * temp_vx + cos(chassis->separate_rad) * temp_vz;
	chassis->real_spd.vx = +cos(chassis->separate_rad) * temp_vx + sin(chassis->separate_rad) * temp_vz;
}

static void Chassis_CalcMoveRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	chassis->separate_rad = (Motor_GimbalYaw.encoder.limited_angle - chassis->install_angle) * PI / 180;
	float sin_tl = (float)sin(chassis->separate_rad);
	float cos_tl = (float)cos(chassis->separate_rad);
	chassis->chassis_coordinate_ref.vz = chassis->gimbal_coordinate_ref.vz * cos_tl - chassis->gimbal_coordinate_ref.vx * sin_tl;
	chassis->chassis_coordinate_ref.vx = chassis->gimbal_coordinate_ref.vz * sin_tl + chassis->gimbal_coordinate_ref.vx * cos_tl;
}

float dead_zone = 0.0f;
static void Chassis_CalcOmniFollowRef()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

	chassis->chassis_coordinate_ref.w = chassis->gimbal_coordinate_ref.w;
	float fdb = Math_Consequent_To_180(chassis->install_angle, Motor_GimbalYaw.encoder.limited_angle);
	PID_SetFdb(&(chassis->Chassis_AngfollowPID), fdb);
	Omni_FourHeadSetPosRef(fdb);
	// if (boardcom->fly_flag == 1)
	// {
	// 	PID_SetRef(&(chassis->Chassis_AngfollowPID), FLY_ANGLE);
	// }
	PID_SetFdb(&(chassis->Chassis_SpdfollowPID), Motor_GimbalYaw.encoder.speed);
	PID_SetRef(&(chassis->Chassis_SpdfollowPID), PID_Calc(&(chassis->Chassis_AngfollowPID)));
	chassis->chassis_coordinate_ref.w += PID_Calc(&(chassis->Chassis_SpdfollowPID));
	if (abs(chassis->Chassis_AngfollowPID.ref - chassis->Chassis_AngfollowPID.fdb) < dead_zone)
	{
		chassis->chassis_coordinate_ref.w = 0;
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
static void InverseKinematics_Translation(float omega[4], float vx, float vy, float wz, float wm)
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
static void InverseKinematics_Rotation(float omega[4], float vx, float vy, float wm)
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
	wz = wm - (fabs(vx) + fabs(vy));
	wz *= boardcom->gyro_dir;
	omega[0] = +vx + vy + wz;
	omega[1] = +vx - vy + wz;
	omega[2] = -vx - vy + wz;
	omega[3] = -vx + vy + wz;
}

static void Omni_FourHeadSetPosRef(float fdb)
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	if (chassis->present_mode == CHASSIS_NORMAL)
	{
		if (fdb < chassis->install_angle + 45 && fdb >= chassis->install_angle - 45)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, chassis->install_angle + 0);
		}
		else if (fdb < chassis->install_angle + 135 && fdb >= chassis->install_angle + 45)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, chassis->install_angle + 90);
		}

		else if (fdb < chassis->install_angle - 45 && fdb >= chassis->install_angle - 135)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, chassis->install_angle - 90);
		}
		else if (fdb >= chassis->install_angle + 135)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, chassis->install_angle + 180);
		}
		else if (fdb < chassis->install_angle - 135)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, chassis->install_angle - 180);
		}
	}
	else if (chassis->present_mode == CHASSIS_GYRO)
	{
		PID_SetRef(&(chassis->Chassis_AngfollowPID), chassis->install_angle);
	}
}

static void Chassis_LowRestEnergyProtect()
{
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	Cap_DataTypeDef *cap = Cap_GetDataPtr();
	if (cap->rest_energy < 25)
	{
		for (int i = 0; i < 4; i++)
		{
			PID_SetRef(&(chassis->Chassis_MotorSpdPID[i]), 0.0);
		}
	}
	else
	{
		for (int i = 0; i < 4; i++)
		{
			PID_SetRef(&(chassis->Chassis_MotorSpdPID[i]), chassis->wheel_ref[i]);
		}
	}
}
