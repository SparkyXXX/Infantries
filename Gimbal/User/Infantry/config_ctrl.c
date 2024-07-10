/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-05-06 23:29:55
 */
//SWING_DANCE

#include "config_ctrl.h"
/********** START OF PARAMETER SETTING **********/
uint8_t CPU_Clock = 170; //(MHz)
uint16_t Shooter_Freq = 500;

float Armor_Feeder_Fast_Speed = 120.0f;
float Armor_Feeder_Slow_Speed = 60.0f;
float Buff_Feeder_Fast_Speed = 6.5f;
float Buff_Feeder_Slow_Speed = 6.5f;
float AutoShoot_Wait_ms = 50.0f;
uint16_t AutoShootBigEnergy_Wait_ms = 110;
uint16_t AutoShootSmallEnergy_Wait_ms = 110;

float KeyMouse_NormalSpeed = 200.0f;
float KeyMouse_UpperSpeed = 500.0f;
float KeyMouse_FlySlopeSpeed = 500.0f;

float Remote_Pitch_To_Ref = 0.0005f;
float Remote_Yaw_To_Ref = 0.0005f;
float Mouse_Pitch_To_Ref = 0.001f;
float Mouse_Yaw_To_Ref = 0.002f;
float Mouse_Pitch_To_Ref_Quiet = 0.0001f;
float Mouse_Yaw_To_Ref_Quiet = 0.0002f;

float Elevation_Angle = 23.0f;
float Depression_Angle = -35.0f;
float Servo_Open = 85.0f;
float Servo_Close = 170.0f;

const float Correction_Matrix[9] = {1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1};

void Gimbal_ParamInit(void)
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
	PID_Init(&gimbal->pitch_spd, PID_POSITION, 18000.0f, 20.0f, 8000.0f, 0.0f, 0.0f, 0.0f, 100.0f, 30000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&gimbal->pitch_pos, PID_POSITION, 0.6f, 0.001f, 3.0f, 0.0f, 0.0f, 0.0f, 100.0f, 30000.0f, 0.1f, 1.0f, 1.0f, 1.0f);
    //                                 mode        p     i     d     kf1   kf2   kf3   sum_max  output_max d_fil  kf1_fil  kf2_fil  kf3_fil
}

void Shoot_ParamInit(void)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    PID_Init(&shooter->feed_spd, PID_POSITION, 500.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->feed_ang, PID_POSITION, 8.35f, 0.0f, 0.11f, 0.0f, 0.0f, 0.0f, 10000.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->shoot_left, PID_POSITION, 2.8f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 1200.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&shooter->shoot_right, PID_POSITION, 2.8f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 1200.0f, 20000.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    //                                 mode        p     i     d     kf1   kf2   kf3   sum_max  output_max d_fil  kf1_fil  kf2_fil  kf3_fil
}
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
}
