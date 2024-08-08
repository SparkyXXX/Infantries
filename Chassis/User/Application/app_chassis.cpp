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
#include "lib_math.h"
#include "periph_motor_can.h"
#include "periph_cap.h"
#include "lib_power_ctrl.h"

float wheel_spd[4] = {0};

PCArgsType pp_args = {
    .mp = 0.3051f,
    .kp = {0.1382f,     0.001005f},
    .cp = 0.3649f,
		.lp = 0.1721f,
    .mn = {-0.000353f, -0.1052f  },
    .kn = {0.176f,     -0.003642f},
    .cn = 0.7356f,
		.ln = 0.144
};


Chassis_ControlTypeDef Chassis_Control;

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
        chassis->last_motor_ref[i] = 0.0f;
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
    PowerCtrl_Init(POWER_LIMITED, &Motor_ChassisMotors);
#if ROBOT_NAME == SWING_DANCE
	Math_InitSlopeParam(&chassis->Chassis_Move_Slope, Chassis_Move_Acc, Chassis_Move_Dec);
	Math_InitSlopeParam(&chassis->Chassis_Gyro_Slope, Chassis_Gyro_Acc, Chassis_Gyro_Dec);
	Math_InitSlopeParam(&chassis->Chassis_Follow_Slope, Chassis_Follow_Acc, Chassis_Follow_Dec);
#endif
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
    chassis->move_ref.forward_back_ref = (chassis->raw_ref.forward_back_ref * cos_tl - chassis->raw_ref.left_right_ref * sin_tl) * 0.18f;
    chassis->move_ref.left_right_ref = (chassis->raw_ref.forward_back_ref * sin_tl + chassis->raw_ref.left_right_ref * cos_tl) * 0.18f;
}

/**
 * @brief      Chassis following solution
 * @param      NULL
 * @retval     NULL
 */
static void Chassis_CalcFollowRef()
{
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();

    chassis->move_ref.rotate_ref = chassis->raw_ref.rotate_ref;
	float fdb = Math_Consequent_To_180(Install_Angle, Motor_GimbalYaw.encoder.limited_angle);
	PID_SetFdb(&(chassis->Chassis_AngfollowPID), fdb);
#if ROBOT_NAME == OLD_WHEAT
	PID_SetRef(&(chassis->Chassis_AngfollowPID), Install_Angle);
    PID_SetRef(&(chassis->Chassis_SpdfollowPID), PID_Calc(&(chassis->Chassis_AngfollowPID)));
#endif
#if ROBOT_NAME == OREO_REO
	PID_SetRef(&(chassis->Chassis_AngfollowPID), Install_Angle);
    PID_SetRef(&(chassis->Chassis_SpdfollowPID), PID_Calc(&(chassis->Chassis_AngfollowPID)));
#endif	
#if ROBOT_NAME == BIG_TITAN
		PID_SetRef(&(chassis->Chassis_AngfollowPID), Install_Angle);
    PID_SetRef(&(chassis->Chassis_SpdfollowPID), PID_Calc(&(chassis->Chassis_AngfollowPID)));
#endif
#if ROBOT_NAME == NEW_TITAN
		PID_SetRef(&(chassis->Chassis_AngfollowPID), Install_Angle);
    PID_SetRef(&(chassis->Chassis_SpdfollowPID), PID_Calc(&(chassis->Chassis_AngfollowPID)));
#endif
#if ROBOT_NAME == SWING_DANCE
	if (chassis->present_mode == CHASSIS_NORMAL)
	{
		if (fdb < -45 && fdb >= -135)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, Install_Angle);
		}
		else if (fdb < 45 && fdb >= -45)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, Install_Angle + 90);
		}

		else if (fdb < -135 && fdb >= -225)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, Install_Angle - 90);
		}
		else if (fdb >= 45)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, Install_Angle + 180);
		}
		else if (fdb < -225)
		{
			PID_SetRef(&chassis->Chassis_AngfollowPID, Install_Angle - 180);
		}
	}	
	else if (chassis->present_mode == CHASSIS_GYRO)
	{
		PID_SetRef(&chassis->Chassis_AngfollowPID, Install_Angle);
	}
	PID_SetFdb(&(chassis->Chassis_SpdfollowPID), Motor_GimbalYaw.encoder.speed);
	PID_SetRef(&chassis->Chassis_SpdfollowPID, PID_Calc(&chassis->Chassis_AngfollowPID));
#endif
    chassis->move_ref.rotate_ref = PID_Calc(&(chassis->Chassis_SpdfollowPID));
    chassis->last_yaw_ref = gimbalyaw->yaw_ref;
}

/**
 * @brief      Calculation of chassis small gyroscope
 * @param      NULL
 * @retval     NULL
 */
static void Chassis_CalcGyroRef()
{
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
    Referee_DataTypeDef *referee = Referee_GetDataPtr();
    float level_gyro_spd = Gyro_Speed_Table[1][1];
    for (int8_t i = 0; i < 12; i++)
    {
        if (referee->chassis_power_limit == Gyro_Speed_Table[i][0])
        {
            level_gyro_spd = Gyro_Speed_Table[i][1];
            break;
        }
    }
    if (chassis->last_mode != CHASSIS_GYRO)
    {
        level_gyro_spd = 0;
    }
	chassis->move_ref.rotate_ref = level_gyro_spd;
}

//zch

#define WheelRadius     (0.15f / 2)
#define WheelBase       (0.3f)
#define GET_SIGN_BIT(x) (((*(uint32_t *)(&(x))) >> 31) & 1)
#define SET_SIGN_POS(x) ((*(uint32_t *)(&(x))) &= 0x7FFFFFFFU)
void GetSpeedRefs(float omega[4], float vx, float vy, float wz, float omegaM) {
     struct {
        uint8_t x   :2;
        uint8_t y   :2;
        uint8_t z   :2;
        uint8_t None:2;
    } sign;
		
     float k, tmp;
		
    sign.x = GET_SIGN_BIT(vx);
    sign.y = GET_SIGN_BIT(vy);
    sign.z = GET_SIGN_BIT(wz);
		
    SET_SIGN_POS(vx);
    SET_SIGN_POS(vy);
    SET_SIGN_POS(wz);
		
    tmp = WheelRadius * omegaM / WheelBase * 0.8f;
    wz  = MIN(wz, tmp);
		
    k = 1.0f;
		
    tmp = MAX(vx, vy) / (omegaM * WheelRadius - wz * WheelBase);
    k   = MAX(k, tmp);
		
    tmp = (vx + vy) / (omegaM * WheelRadius);
    k   = MAX(k, tmp);
		
    vx = vx / k * (1 - sign.x * 2);
    vy = vy / k * (1 - sign.y * 2);
    wz = wz * (1 - sign.z * 2);
		
    omega[0] = (vx + vy - wz * WheelBase) / WheelRadius;
    omega[1] = (vx - vy - wz * WheelBase) / WheelRadius;
    omega[2] = (-vx - vy - wz * WheelBase) / WheelRadius;
    omega[3] = (-vx + vy - wz * WheelBase) / WheelRadius;
}

void GetSpeedRefs_Gyro(float omega[4], float vx, float vy, float omegaM) {
     float wz = 0.0f;
     struct {
        uint8_t x   :2;
        uint8_t y   :2;
        uint8_t z   :2;
        uint8_t None:2;
    } sign;
     float k, tmp;
    sign.x = GET_SIGN_BIT(vx);
    sign.y = GET_SIGN_BIT(vy);
    SET_SIGN_POS(vx);
    SET_SIGN_POS(vy);
    k = 1.0f;
		
    tmp = (vx + vy) / (omegaM * WheelRadius * 0.7);
    k   = MAX(k, tmp);
		
		vx /= k;
		vy /= k;
		
    wz = WheelRadius * omegaM / WheelBase;
    tmp = (omegaM * WheelRadius - MAX(vx, vy)) / (WheelBase);
    wz  = MIN(wz, tmp);
		
		
    vx *= (1 - sign.x * 2);
    vy *= (1 - sign.y * 2);
    omega[0] = (vx + vy - wz * WheelBase) / WheelRadius;
    omega[1] = (vx - vy - wz * WheelBase) / WheelRadius;
    omega[2] = (-vx - vy - wz * WheelBase) / WheelRadius;
    omega[3] = (-vx + vy - wz * WheelBase) / WheelRadius;
}

/**
 * @brief      Wheel round solution
 * @param      NULL
 * @retval     NULL
 */
#if ROBOT_NAME == SWING_DANCE
float Wheel_Speed_Ref[4] = {};
float Wheel_Speed_Max = 0;
float Protect_Coef = 1.0;
float temp_wheel_speed = 0.0f;
#endif
static void Chassis_CalcWheelRef()
{
	
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	Referee_DataTypeDef *referee = Referee_GetDataPtr();
	
    for (int i = 0; i < 4; i++)
    {
        chassis->last_motor_ref[i] = PID_GetRef(&chassis->Chassis_MotorSpdPID[i]);
    }

    float f_b_ref = chassis->move_ref.forward_back_ref;
    float l_r_ref = chassis->move_ref.left_right_ref;
    float gyro_ref = chassis->move_ref.rotate_ref;
		
		
		
//			float k_follow = 1.0f + sqrt(f_b_ref*f_b_ref + l_r_ref*l_r_ref)/1.0f;//基础的随动1.0f + 平动速度平方*系数作为平动时随动的偏置,400000~=700*700
//			float k_fblr = 1.0f - gyro_ref/400.0f;//跟随速度越大，越减小平动速度，150000 = 300*300*5/3
		//if(sqrt(f_b_ref*f_b_ref + l_r_ref*l_r_ref)>2.0f && gyro_ref > 2.0f){
			
		//}
	switch (chassis->present_mode)
	{
		case CHASSIS_STOP:
			for (int i = 0; i < 4; i++)
			{
				PID_SetRef(&chassis->Chassis_MotorSpdPID[i], 0);
			}
			break;
		
		case CHASSIS_NORMAL:
//			k_fblr = 1.0f;
//			k_follow = 1.0f;
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[FORWARD_LEFT], (f_b_ref + l_r_ref)*k_fblr + gyro_ref*k_follow);
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[FORWARD_RIGHT], (-f_b_ref + l_r_ref)*k_fblr + gyro_ref*k_follow);
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[BACKWARD_RIGHT], (-f_b_ref - l_r_ref)*k_fblr + gyro_ref*k_follow);
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[BACKWARD_LEFT], (f_b_ref - l_r_ref)*k_fblr + gyro_ref*k_follow);
			if(boardcom->power_limit_mode == POWER_LIMITED){
					GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, -gyro_ref, referee->chassis_power_limit * 1.818 + 250);
			}else{
					GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, -gyro_ref, 500);
			}
			for(int i = 0; i < 4; i ++) {
				PID_SetRef(&chassis->Chassis_MotorSpdPID[i], wheel_spd[i]);
			}
			break;
		case CHASSIS_GYRO:
			
			if(boardcom->power_limit_mode == POWER_UNLIMITED){
				wheel_spd[0] = f_b_ref * 5.0f + l_r_ref * 5.0f + -Forward_Left_Compensate * gyro_ref * 1.1;
				wheel_spd[1] = -f_b_ref * 5.0f + l_r_ref * 5.0f + -Forward_Right_Compensate * gyro_ref * 1.1;
				wheel_spd[2] = -f_b_ref * 5.0f - l_r_ref * 5.0f + -Backward_Right_Compensate * gyro_ref * 1.1;
				wheel_spd[3] = f_b_ref * 5.0f - l_r_ref * 5.0f + -Backward_Left_Compensate * gyro_ref * 1.1;
//					GetSpeedRefs(wheel_spd, l_r_ref, f_b_ref, gyro_ref * 0.2f, 600);
			}else if(boardcom->cap_mode_user == SUPERCAP_CTRL_ON){
				GetSpeedRefs_Gyro(wheel_spd, l_r_ref, f_b_ref, referee->chassis_power_limit * PowerUp_Coef * 1.818 + 250);
			}else{
				GetSpeedRefs_Gyro(wheel_spd, l_r_ref, f_b_ref, referee->chassis_power_limit * 1.818 + 250);
			}
		
			for(int i = 0; i < 4; i ++) {
				PID_SetRef(&chassis->Chassis_MotorSpdPID[i], wheel_spd[i]);
			}
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[FORWARD_LEFT], f_b_ref + l_r_ref + Forward_Left_Compensate * gyro_ref);
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[FORWARD_RIGHT], -f_b_ref + l_r_ref + Forward_Right_Compensate * gyro_ref);
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[BACKWARD_RIGHT], -f_b_ref - l_r_ref + Backward_Right_Compensate * gyro_ref);
//			PID_SetRef(&chassis->Chassis_MotorSpdPID[BACKWARD_LEFT], f_b_ref - l_r_ref + Backward_Left_Compensate * gyro_ref);
			break;
		case CHASSIS_BACKGYRO:
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
 * @brief      Calculation of Cap Output Power MAX
 * @param      remain_energy
 * @retval     NULL
 */
#define Max(a,b) (a>b?a:b)//lzq
#define Min(a,b) (a>b?b:a)
float Max_Power_Cal(uint8_t remain_energy)
{
	Referee_DataTypeDef *referee = Referee_GetDataPtr();
	Cap_DataTypeDef *cap = Cap_GetDataPtr();
	float VCC_Cap = sqrt((double)((remain_energy / 100.0f) * 27.0f * 27.0f));
	float Power_Input_Max = cap->BOOST_INPUT_CURRENT_MAX  * VCC_Cap;
	float VCC_Motor = Min(24.0f , (Power_Input_Max / cap->BOOST_OUTPUT_CURRENT_MAX));
	float Power_Output_Max = cap->BOOST_OUTPUT_CURRENT_MAX * VCC_Motor;
	float Cap_Charging_Power_Max = Min(VCC_Cap * cap->BUCK_OUTPUT_CURRENT_MAX, referee->chassis_power_limit);
	if(remain_energy >= 15){
	return Power_Output_Max;
	}
	else{
		float Low_Power_Output_Max = Power_Output_Max * (remain_energy * remain_energy) / 255.0f;
		return Max(Low_Power_Output_Max , Cap_Charging_Power_Max);
	}
}//lzq

/**
 * @brief      Calculation of chassis control quantity
 * @param      NULL
 * @retval     NULL
 */
uint8_t speedup_enable_flag = 1;
float ChassisI[4];
float ChassisW[4];
float overMin = 1000.0f, overMax = 0.0f;
float my_power_limit = 45.0f;
//float power_offset[10] = {50.0f,61.0f,70.0f,82.0f,92.0f,103.0f,113.0f,};//45,55,65,75,85,95,105,115

void Chassis_Output()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Referee_DataTypeDef *referee = Referee_GetDataPtr();
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
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
			Chassis_CalcMoveRef();   // Headless translation solution
			Chassis_CalcFollowRef(); // Chassis following soGYROlution
			break;
		case CHASSIS_GYRO:
		case CHASSIS_BACKGYRO:
			Chassis_CalcMoveRef(); // Headless translation solution
			if(boardcom->power_limit_mode == POWER_UNLIMITED){
				Chassis_CalcGyroRef(); // Solution of gyro
			}
			break;
		default:
			return;
	}

    // hrx: 测试保护数据
	// if (cap->rest_energy < 20)
	// {
	// 	speedup_enable_flag = 0;
	// 	LimitMax(chassis->move_ref.forward_back_ref, 90);
	// 	LimitMax(chassis->move_ref.left_right_ref, 90);
	// }
	// if (cap->rest_energy > 25 && speedup_enable_flag == 0)
	// {
	// 	speedup_enable_flag = 1;
	// }
	// else if (cap->rest_energy <= 25 && speedup_enable_flag == 0)
	// {
	// 	LimitMax(chassis->move_ref.forward_back_ref, 90);
	// 	LimitMax(chassis->move_ref.left_right_ref, 90);
	// }

    Chassis_CalcWheelRef();
	
	for (int i = 0; i < 4; i++)
	{
		PID_SetFdb(&chassis->Chassis_MotorSpdPID[i], Motor_ChassisMotors.motor_handle[i]->encoder.speed);
		ChassisI[i] = PID_Calc(&chassis->Chassis_MotorSpdPID[i]) * 20 / 16384.0f;
		ChassisW[i] = (Motor_ChassisMotors.motor_handle[i]->encoder.speed / 9.549296596425384f);
	}
	
#if POWER_CTRL == NEW//lzq
	float power_limit_temp = (boardcom->power_limit_mode ? (referee->chassis_power_limit * (boardcom->cap_mode_user ? PowerUp_Coef : 1.0)) : 300.0f);//计算功率期望
	float PC_Limit =  Min(power_limit_temp , Max_Power_Cal(cap->rest_energy))* PC_Limit_K + PC_Limit_B;

	PowerControl(pp_args, PC_Limit, ChassisI, ChassisW); // 功率控制

#endif
	/*if(referee->chassis_power > my_power_limit) {
		overMax = MAX(overMax,referee->chassis_power);
		overMin = MIN(overMin,referee->chassis_power);
	}*/
	
	//PowerControl(pp_args, (referee->chassis_power_limit * 0.0f + 55.0f), CurrentToSend, SpeedNow);
	for (int i = 0; i < 4; i++)
	{
    Motor_SetOutput(Motor_ChassisMotors.motor_handle[i], ChassisI[i]*819.2f);
	}
}
