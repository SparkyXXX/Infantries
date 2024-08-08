/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-18 12:03:32
 */

#include "config_ctrl.h"

/********** START OF PARAMETER SETTING **********/

/********** START OF BIG_TITAN **********/
#if ROBOT_NAME == BIG_TITAN
uint8_t CPU_Clock = 170; //(MHz)

float Elevation_Angle = 39.0f;
float Depression_Angle = -22.0f;

uint16_t Shooter_Freq = 500;
float Armor_Feeder_Fast_Speed = 120.0f;
float Armor_Feeder_Slow_Speed = 80.0f;
float Buff_Feeder_Fast_Speed = 20.0f;
float Buff_Feeder_Slow_Speed = 20.0f;
uint16_t AutoShoot_Wait_ms = 200;//80
uint16_t AutoShootBigEnergy_Wait_ms = 310;
uint16_t AutoShootSmallEnergy_Wait_ms = 310;

float KeyMouse_NormalSpeed = 100.0f;
float KeyMouse_CapOnSpeed = 250.0f;
float KeyMouse_FlySlopeSpeed = 500.0f;

float Feeder_Initing_Speed = 80.0f;

float Remote_Acc = 5.0f;
float Remote_Dec = 5.0f;
float Chassis_Move_Speed = 180.0f;
float SpeedUp_Coef = 3.9f;

float Remote_Pitch_To_Ref = 0.0005f;
float Remote_Yaw_To_Ref = 0.0005f;
float Mouse_Pitch_To_Ref = 0.001f;
float Mouse_Yaw_To_Ref = 0.002f;
float Mouse_Pitch_To_Ref_Quiet = 0.0001f;
float Mouse_Yaw_To_Ref_Quiet = 0.0002f;

float Servo_Open = 162.0f;
float Servo_Close = 92.0f;

const float Correction_Matrix[9] = {0, -1, 0,
                                 1, 0, 0,
                                 0, 0, 1};


float Pitch_Spd_KpSet[7] = {16000.0f, 12000.0f, 10000.0f, 8000.0f, 10000.0f, 12000.0f, 16000.0f};
float Pitch_Spd_KiSet[7] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
float Pitch_Spd_KdSet[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
Interval Pitch_Spd_ErrorRange = {-0.5f, 0.5f};
Interval Pitch_Spd_ErrorChangeRange = {-1.0f, 1.0f};

float Pitch_Pos_KpSet[7] = {0.2f, 0.2f, 0.18f, 0.16f, 0.18f, 0.2f, 0.2f};
float Pitch_Pos_KiSet[7] = {0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f};
float Pitch_Pos_KdSet[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
Interval Pitch_Pos_ErrorRange = {-1.0f, 1.0f};
Interval Pitch_Pos_ErrorChangeRange = {-1.0f, 1.0f};

void Shoot_ParamInit(void)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    PID_Init(&shooter->feed_spd, PID_POSITION, 500.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->feed_ang, PID_POSITION, 8.35f, 0.0f, 0.11f, 0.0f, 00.0f, 0.0f, 10000.0f, 10000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->shoot_left, PID_POSITION, 2.8f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 1200.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->shoot_right, PID_POSITION, 2.8f, 0.025f, 0.0f, 0.0f, 0.0f, 0.0f, 1260.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    //                             mode          p     i     d     kf1   kf2   sum_max  output_max d_fil  kf1_fil  kf2_fil
}

void Gimbal_ParamInit(void)
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    PID_Init(&gimbal->pitch_spd_no_auto, PID_POSITION, 12000.0f, 20.0f, 0.0f, 0.0f, 0.0f, -4000.0f, 1000.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_no_auto, PID_POSITION, 0.8f, 0.02f, 2.0f, 0.0f, 0.0f, 0.0f, 20.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    
		PID_Init(&gimbal->pitch_spd_armor, PID_POSITION, 10000.0f, 7.5f, 10000.0f, 20000.0f, 0.0f, -4000.0f, 950.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_armor, PID_POSITION, 0.65f, 0.03f, 3.0f, 1.5f, 0.0f, 0.0f, 3.5f, 30000.0f, 0.4f, 1.0f, 1.0f, 1.0f);
    
		PID_Init(&gimbal->pitch_spd_small_energy, PID_POSITION, 13000.0f, 30.0f, 10000.0f, 0.0f, 0.0f, -4000.0f, 300.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_small_energy, PID_POSITION, 0.9f, 0.03f, 6.0f, 0.0f, 0.0f, 0.0f, 10.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    
		PID_Init(&gimbal->pitch_spd_big_energy, PID_POSITION, 13000.0f, 30.0f, 10000.0f, 0.0f, 0.0f, -4000.0f, 300.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_big_energy, PID_POSITION, 0.9f, 0.03f, 6.0f, 0.0f, 0.0f, 0.0f, 10.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    //                             mode          p        i     d     kf1   kf2   kf3   sum_max  output_max d_fil  kf1_fil  kf2_fil  kf3_fil
}
#endif
/********** END OF BIG_TITAN **********/

/********** START OF NEW_TITAN **********/
#if ROBOT_NAME == NEW_TITAN
uint8_t CPU_Clock = 170; //(MHz)

float Elevation_Angle = 39.0f;
float Depression_Angle = -25.0f;

uint16_t Shooter_Freq = 500;
float Armor_Feeder_Fast_Speed = 120.0f;
float Armor_Feeder_Slow_Speed = 80.0f;
float Buff_Feeder_Fast_Speed = 20.0f;
float Buff_Feeder_Slow_Speed = 20.0f;
uint16_t AutoShoot_Wait_ms = 200;//80
uint16_t AutoShootBigEnergy_Wait_ms = 310;
uint16_t AutoShootSmallEnergy_Wait_ms = 310;

float KeyMouse_NormalSpeed = 100.0f;
float KeyMouse_CapOnSpeed = 250.0f;
float KeyMouse_FlySlopeSpeed = 500.0f;

float Feeder_Initing_Speed = 80.0f;

float Remote_Acc = 5.0f;
float Remote_Dec = 5.0f;
float Chassis_Move_Speed = 180.0f;
float SpeedUp_Coef = 3.9f;

float Remote_Pitch_To_Ref = 0.0005f;
float Remote_Yaw_To_Ref = 0.0005f;
float Mouse_Pitch_To_Ref = 0.001f;
float Mouse_Yaw_To_Ref = 0.002f;
float Mouse_Pitch_To_Ref_Quiet = 0.0001f;
float Mouse_Yaw_To_Ref_Quiet = 0.0002f;

float Servo_Open = 162.0f;
float Servo_Close = 92.0f;

const float Correction_Matrix[9] = {0, -1, 0,
                                 1, 0, 0,
                                 0, 0, 1};


float Pitch_Spd_KpSet[7] = {16000.0f, 12000.0f, 10000.0f, 8000.0f, 10000.0f, 12000.0f, 16000.0f};
float Pitch_Spd_KiSet[7] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
float Pitch_Spd_KdSet[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
Interval Pitch_Spd_ErrorRange = {-0.5f, 0.5f};
Interval Pitch_Spd_ErrorChangeRange = {-1.0f, 1.0f};

float Pitch_Pos_KpSet[7] = {0.2f, 0.2f, 0.18f, 0.16f, 0.18f, 0.2f, 0.2f};
float Pitch_Pos_KiSet[7] = {0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f};
float Pitch_Pos_KdSet[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
Interval Pitch_Pos_ErrorRange = {-1.0f, 1.0f};
Interval Pitch_Pos_ErrorChangeRange = {-1.0f, 1.0f};

void Shoot_ParamInit(void)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    PID_Init(&shooter->feed_spd, PID_POSITION, 500.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->feed_ang, PID_POSITION, 8.35f, 0.0f, 0.11f, 0.0f, 00.0f, 0.0f, 10000.0f, 10000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->shoot_left, PID_POSITION, 2.0f, 0.03f, 0.0f, 0.0f, 0.0f, 0.0f, 1200.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->shoot_right, PID_POSITION, 2.0f, 0.03f, 0.0f, 0.0f, 0.0f, 0.0f, 1200.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    //                             mode          p     i     d     kf1   kf2   sum_max  output_max d_fil  kf1_fil  kf2_fil
}

void Gimbal_ParamInit(void)
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    PID_Init(&gimbal->pitch_spd_no_auto, PID_POSITION, 12000.0f, 20.0f, 0.0f, 0.0f, 0.0f, -4000.0f, 1000.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_no_auto, PID_POSITION, 0.8f, 0.02f, 2.0f, 0.0f, 0.0f, 0.0f, 20.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    
		PID_Init(&gimbal->pitch_spd_armor, PID_POSITION, 10000.0f, 10.0f, 10000.0f, 20000.0f, 0.0f, -4000.0f, 1000.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_armor, PID_POSITION, 0.8f, 0.06f, 5.0f, 1.5f, 0.0f, 0.0f, 5.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    
		PID_Init(&gimbal->pitch_spd_small_energy, PID_POSITION, 13000.0f, 30.0f, 10000.0f, 0.0f, 0.0f, -4000.0f, 300.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_small_energy, PID_POSITION, 0.9f, 0.03f, 6.0f, 0.0f, 0.0f, 0.0f, 10.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    
		PID_Init(&gimbal->pitch_spd_big_energy, PID_POSITION, 13000.0f, 30.0f, 10000.0f, 0.0f, 0.0f, -4000.0f, 300.0f, 30000.0f, 0.1f, 0.1f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos_big_energy, PID_POSITION, 0.9f, 0.03f, 6.0f, 0.0f, 0.0f, 0.0f, 10.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    //                             mode          p        i     d     kf1   kf2   kf3   sum_max  output_max d_fil  kf1_fil  kf2_fil  kf3_fil
}
#endif
/********** END OF BIG_TITAN **********/

/********** END OF PARAMETER SETTING **********/

float imu_yaw_watch;
float imu_pitch_watch;
float imu_roll_watch;
float shooter_diff_watch;
float shooter_left_watch;
float shooter_right_watch;
float pitch_pos_ref_watch;
float pitch_pos_fdb_watch;
float pitch_pos_error_watch;
uint8_t autoshoot_flag_watch;
uint8_t aim_mode_watch;

Servo_DataTypeDef Servo_MagServo;

BMI088_DataTypeDef BMI088_Data;
SPI_HandleTypeDef *BMI088_SPI_HANDLER = &hspi1;
GPIO_HandleTypeDef CS_ACCEL = {GPIO_PIN_RESET, GPIOC, GPIO_PIN_15};
GPIO_HandleTypeDef CS_GYRO = {GPIO_PIN_RESET, GPIOC, GPIO_PIN_14};

Motor_GroupDataTypeDef *Motor_groupHandle[3];
Motor_GroupDataTypeDef Motor_gimbalMotors;
Motor_GroupDataTypeDef Motor_feederMotors;
MotorPWM_GroupDataTypeDef Motor_shooterMotors;

Motor_DataTypeDef Motor_gimbalMotorPitch;
Motor_DataTypeDef Motor_feederMotor;
MotorPWM_DataTypeDef Motor_shooterMotorLeft;
MotorPWM_DataTypeDef Motor_shooterMotorRight;

//float infantry_id;
//void Choose_Your_Infantry()
//{
//	infantry_id = 0x07 - (HAL_GPIO_ReadPin(SW0_GPIO_Port,SW0_Pin) * 4
//							  + HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin)*2
//							  + HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin));
//	// sw left-to-right:421, the switchPin is pull-up, and turnon the sw will pull-down the pin
//}

/**
 * @brief      Initialize all motors
 * @param      NULL
 * @retval     NULL
 */
static void Init_All_Motors()
{
    Motor_groupHandle[1] = &Motor_gimbalMotors;
    Motor_InitGroup(&Motor_gimbalMotors, 2, &hfdcan1, 0x1FF);
    Motor_Init(&Motor_gimbalMotorPitch, PITCH_CAN_ID);
    Motor_gimbalMotors.motor_handle[PITCH_CAN_ID - 0x205] = &Motor_gimbalMotorPitch;

    Motor_groupHandle[2] = &Motor_feederMotors;
    Motor_InitGroup(&Motor_feederMotors, 1, &hfdcan1, 0x200);
    Motor_Init(&Motor_feederMotor, FEEDER_CAN_ID);
    Motor_feederMotors.motor_handle[FEEDER_CAN_ID - 0x201] = &Motor_feederMotor;

    MotorPWM_InitGroup(&Motor_shooterMotors, 2);
    MotorPWM_Init(&Motor_shooterMotorLeft, &htim3, TIM_CHANNEL_4, CPU_Clock * 1000000, Shooter_Freq, &htim1);
    MotorPWM_Init(&Motor_shooterMotorRight, &htim3, TIM_CHANNEL_3, CPU_Clock * 1000000, Shooter_Freq, &htim2);
    Motor_shooterMotors.motor_handle[0] = &Motor_shooterMotorLeft;
    Motor_shooterMotors.motor_handle[1] = &Motor_shooterMotorRight;
}

/**
 * @brief      Initialize all peripherals
 * @param      NULL
 * @retval     NULL
 */
void Init_All(void)
{
    DWT_Init(CPU_Clock);

		Monitor_Init();
    Servo_Init(&Servo_MagServo, &htim15, TIM_CHANNEL_1, CPU_Clock * 1000000, Servo_Close);
    while (BMI088_Init(&BMI088_Data, BMI088_SPI_HANDLER, &CS_ACCEL, &CS_GYRO)){;}
    MiniPC_Init();
    ADC_Init();
    Init_All_Motors();
    BoardCom_Init();
    FDCAN_IntFilterAndStart(&hfdcan1);
    FDCAN_IntFilterAndStart(&hfdcan2);

    INS_Init();
    Gimbal_Init();
    Shoot_Init();
    AutoAim_Init();

    Remote_Init(&huart3);
    Remote_ControlInit();
		ext_Remote_Init(&huart2);
	beep_once(10,30);
}
