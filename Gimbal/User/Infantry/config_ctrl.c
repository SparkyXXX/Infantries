/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-16 16:30:28
 */

#include "config_ctrl.h"
#include "lib_timer_tim1.h"

/********** START OF PARAMETER SETTING **********/
uint8_t CPU_Clock = 170; //(MHz)
uint16_t Shooter_Freq = 500;

float Armor_Feeder_Fast_Speed = 150.0f;
float Armor_Feeder_Slow_Speed = 60.0f;
float Buff_Feeder_Fast_Speed = 6.5f;
float Buff_Feeder_Slow_Speed = 6.5f;
float AutoShoot_Wait_ms = 250.0f;
uint16_t AutoShootBigEnergy_Wait_ms = 310;
uint16_t AutoShootSmallEnergy_Wait_ms = 310;

float KeyMouse_NormalSpeed = 200.0f;
float KeyMouse_UpperSpeed = 500.0f;
float KeyMouse_FlySlopeSpeed = 500.0f;

float Remote_Pitch_To_Ref = 0.0005f;
float Remote_Yaw_To_Ref = 0.0005f;
float Mouse_Pitch_To_Ref = 0.001f;
float Mouse_Yaw_To_Ref = 0.002f;
float Mouse_Pitch_To_Ref_Quiet = 0.0001f;
float Mouse_Yaw_To_Ref_Quiet = 0.0002f;

float Elevation_Angle = 35.0f;
float Depression_Angle = -27.0f;
float Servo_Open = 85.0f;
float Servo_Close = 170.0f;

const float Correction_Matrix[9] = {0, -1, 0,
                                    1, 0, 0,
                                    0, 0, 1};

float Pitch_Spd_KpSet[7] = {10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f};
float Pitch_Spd_KiSet[7] = {20000.0f, 20000.0f, 20000.0f, 20000.0f, 50000.0f, 50000.0f, 50000.0f};
float Pitch_Spd_KdSet[7] = {30.0f, 30.0f, 30.0f, 30.0f, 20.0f, 20.0f, 20.0f};
Interval Pitch_Spd_Error_Range = {-8.0f, 8.0f};
Interval Pitch_Spd_ErrorChange_Range = {-8.0f, 8.0f};

float Pitch_Pos_KpSet[7] = {80.0f, 60.0f, 40.0f, 30.0f, 40.0f, 60.0f, 80.0f};
float Pitch_Pos_KiSet[7] = {20.0f, 20.0f, 20.0f, 15.0f, 10.0f, 10.0f, 10.0f};
float Pitch_Pos_KdSet[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
Interval Pitch_Pos_Error_Range = {-0.54f, 0.48f};
Interval Pitch_Pos_ErrorChange_Range = {-0.54f, 0.48f};

void Gimbal_ParamInit(void)
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    FuzzyPID_Init(&(gimbal->pitch_spd), Pitch_Spd_KpSet, Pitch_Spd_KiSet, Pitch_Spd_KdSet,
                  &Pitch_Spd_Error_Range, &Pitch_Spd_ErrorChange_Range, 20000.0f, 8000.0f, 25000.0f, 100000000.0f, 159.154922f);
    //																	kf        sum_max  output_max d_cutoff_frq kf_cutoff_frq
    FuzzyPID_Init(&(gimbal->pitch_pos), Pitch_Pos_KpSet, Pitch_Pos_KiSet, Pitch_Pos_KdSet,
                  &Pitch_Pos_Error_Range, &Pitch_Pos_ErrorChange_Range, 0.0f, 0.15f, 15.0f, 159.154922f, 159.154922f);
}

void Shoot_ParamInit(void)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    PID_Init(&(shooter->feed_spd), 500.0f, 10.0f, 0.0f, 0.0f, 10000.0f, 20000.0f, 159.154922f, 159.154922f);
    PID_Init(&(shooter->feed_ang), 8.35f, 0.0f, 0.00011f, 0.0f, 10000.0f, 20000.0f, 159.154922f, 159.154922f);

    PID_Init(&(shooter->shoot_left), 10.0f, 20.0f, 0.0f, 0.0f, 31.0f, 35.0f, 159.154922f, 159.154922f);
    PID_Init(&(shooter->shoot_right), 10.0f, 20.0f, 0.0f, 0.0f, 31.0f, 35.0f, 159.154922f, 159.154922f);

    Filter_Lowpass_Init(200.0f, &(shooter->shooter_left_lpf));
    Filter_Lowpass_Init(200.0f, &(shooter->shooter_right_lpf));
}
/********** END OF PARAMETER SETTING **********/

Servo_DataTypeDef Servo_MagServo;

BMI088_DataTypeDef BMI088_Data;
SPI_HandleTypeDef *BMI088_SPI_HANDLER = &hspi1;
GPIO_HandleTypeDef CS_ACCEL = {GPIO_PIN_RESET, GPI OC, GPIO_PIN_15};
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
    while (BMI088_Init(&BMI088_Data, BMI088_SPI_HANDLER, &CS_ACCEL, &CS_GYRO))
    {
        ;
    }
    MiniPC_Init();
    ADC_Init();
    Init_All_Motors();
    BoardCom_Init();
    FDCAN_IntFilterAndStart(&hfdcan1);
    FDCAN_IntFilterAndStart(&hfdcan2);

    INS_Init();
    Gimbal_Init();
    Shoot_Init();
    Remote_Init(&huart3);
}
