/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-05-07 00:47:21
 */

#include "app_shoot.h"

Shoot_ControlTypeDef Shoot_Control;

/**
 * @brief      shooter control initialization
 * @param      NULL
 * @retval     NULL
 */
void Shoot_Init()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    shooter->shooter_mode = SHOOTER_STOP;
    shooter->feeder_mode = FEEDER_INITING;
    shooter->heat_ctrl.shooter_17mm_cooling_heat = 0;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = 0;
    shooter->shoot_speed.left_shoot_speed = 0;
    shooter->shoot_speed.right_shoot_speed = 0;
    shooter->shoot_speed.feeder_shoot_speed = 0;
    for (int i = 0; i < 7; i++)
    {
        Motor_shooterMotorLeft.pwm.duty = 0.1 * i;
        Motor_shooterMotorRight.pwm.duty = 0.1 * i;
        Motor_PWM_SendOutput(&Motor_shooterMotorLeft);
        Motor_PWM_SendOutput(&Motor_shooterMotorRight);
        HAL_Delay(200);
    }
    Shoot_ParamInit();
}

void Shoot_ShooterControl()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    switch (shooter->shooter_mode)
    {
    case SHOOTER_STOP:
        shooter->shoot_speed.left_shoot_speed = 0;
        shooter->shoot_speed.right_shoot_speed = 0;
        break;
    case SHOOTER_REFEREE:
        shooter->shoot_speed.left_shoot_speed = 24.5f;
        shooter->shoot_speed.right_shoot_speed = 24.5f;
        break;
    default:
        break;
    }

    PID_SetRef(&shooter->shoot_left, shooter->shoot_speed.left_shoot_speed);
    PID_SetRef(&shooter->shoot_right, shooter->shoot_speed.right_shoot_speed);
    PID_SetFdb(&shooter->shoot_left, Motor_shooterMotorLeft.encoder.speed);
    PID_SetFdb(&shooter->shoot_right, Motor_shooterMotorRight.encoder.speed);
    MotorPWM_SetOutput(&Motor_shooterMotorLeft, PID_Calc(&shooter->shoot_left));
    MotorPWM_SetOutput(&Motor_shooterMotorRight, PID_Calc(&shooter->shoot_right));
}

/**
 * @brief      Shooter feeder control
 * @param      NULL
 * @retval     NULL
 */
int shoot_mode = 0;
void Shoot_FeederControl()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();

    switch (shooter->feeder_mode)
    {
    case FEEDER_STOP:
        shoot_mode = SINGLE;
        shooter->shoot_speed.feeder_shoot_speed = 0.0f;
        break;
    case FEEDER_SINGLE:
        shoot_mode = SINGLE;
        Shoot_Single();
        break;
    case FEEDER_LOCKED:
        shoot_mode = CONTINUOUS;
        Shoot_FeederLockedHandle();
        break;
    case FEEDER_REFEREE:
        shoot_mode = CONTINUOUS;
        Shoot_HeatControl();
        shooter->shoot_speed.feeder_shoot_speed = shooter->heat_ctrl.feeder_speed;
        break;
    case FEEDER_FINISH:
        shoot_mode = CONTINUOUS;
        Shoot_HeatControl();
        break;
    default:
        break;
    }

    if (shoot_mode == CONTINUOUS)
    {
        PID_SetRef(&shooter->feed_spd, shooter->shoot_speed.feeder_shoot_speed);
        if (Motor_shooterMotorLeft.encoder.speed < 20 || Motor_shooterMotorLeft.encoder.speed < 20 || ADC_Voltage < 15.0f)
        {
            PID_SetRef(&shooter->feed_spd, 0);
        }
        PID_SetFdb(&shooter->feed_spd, Motor_feederMotor.encoder.speed);
        PID_Calc(&shooter->feed_spd);
        PID_SetRef(&shooter->feed_ang, (int)((Motor_feederMotor.encoder.consequent_angle - shooter->Initial_Feeder_Angle + 40.0f) / 45) * 45 + shooter->Initial_Feeder_Angle);
    }
    else if (shoot_mode == SINGLE)
    {
        PID_SetFdb(&shooter->feed_ang, Motor_feederMotor.encoder.consequent_angle);
        PID_SetRef(&shooter->feed_spd, PID_Calc(&shooter->feed_ang));
        if (Motor_shooterMotorLeft.encoder.speed < 20 || Motor_shooterMotorLeft.encoder.speed < 20 || ADC_Voltage < 15.0f)
        {
            PID_SetRef(&shooter->feed_spd, 0);
        }
        PID_SetFdb(&shooter->feed_spd, Motor_feederMotor.encoder.speed);
        PID_Calc(&shooter->feed_spd);
    }
    Motor_SetOutput(&Motor_feederMotor, PID_GetOutput(&shooter->feed_spd));
}

/**
 * @brief      Shooter heat control
 * @param      NULL
 * @retval     pid_num
 */
float Feeder_Fast_Speed = 0.0f;
float Feeder_Slow_Speed = 0.0f;
static void Shoot_HeatControl()
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= HEAT_FAST_LIMIT)
    {
        shooter->heat_ctrl.feeder_speed = Feeder_Fast_Speed;
    }
    else if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= HEAT_SLOW_LIMIT)
    {
        shooter->heat_ctrl.feeder_speed = Feeder_Slow_Speed;
    }
    else if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) <= HEAT_WAIT_LIMIT)
    {
        shooter->heat_ctrl.feeder_speed = 0;
    }
}

/**
 * @brief      Updata control data
 * @param      NULL
 * @retval     NULL
 */
void Shoot_Update()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    shooter->heat_ctrl.shooter_17mm_cooling_heat = (float)boardcom->heat_now;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = (float)boardcom->heat_limit;
    Motor_PWM_ReadEncoder(&Motor_shooterMotorLeft, 1);
    Motor_PWM_ReadEncoder(&Motor_shooterMotorRight, 1);
    ADC_Decode();
    shooter_left_watch = Motor_shooterMotorLeft.encoder.speed;
    shooter_right_watch = Motor_shooterMotorRight.encoder.speed;
    shooter_diff_watch = Motor_shooterMotorLeft.encoder.speed - Motor_shooterMotorRight.encoder.speed;
}

/**
 * @brief      Output shooter motor
 * @param      NULL
 * @retval     NULL
 */
uint64_t shooter_cnt = 0;
uint8_t shooter_adc_flag = 1;
uint8_t shooter_adc_flag_last = 1;
void Shoot_Output()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    if (ADC_Voltage <= 10)
    {
        Motor_shooterMotors.motor_handle[0]->output = 0;
        Motor_shooterMotors.motor_handle[1]->output = 0;
        shooter_adc_flag = 1;
    }
    else if (shooter_adc_flag == 1 && ADC_Voltage > 18)
    {
        shooter_adc_flag = 0;
        shooter_cnt = HAL_GetTick();
    }
    if ((HAL_GetTick() - shooter_cnt) < 7000)
    {
        Motor_shooterMotors.motor_handle[0]->output = 0;
        Motor_shooterMotors.motor_handle[1]->output = 0;
    }
    shooter_adc_flag_last = shooter_adc_flag;

    Motor_PWM_SendOutput(&Motor_shooterMotorLeft);
    Motor_PWM_SendOutput(&Motor_shooterMotorRight);
    Motor_CAN_SendGroupOutput(&Motor_feederMotors);
}

/**
 * @brief      Shooter feeder control: single shooting
 * @param      NULL
 * @retval     NULL
 */
void Shoot_Single()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();

    if (fabs(PID_GetFdb(&shooter->feed_ang) - PID_GetRef(&shooter->feed_ang)) > 5.0f)
    {
        return;
    }
    if (!shooter->single_shoot_done && Motor_shooterMotorLeft.encoder.speed > 20 && Motor_shooterMotorLeft.encoder.speed > 20 && ADC_Voltage > 15.0f)
    {
        PID_AddRef(&shooter->feed_ang, 45.0f);
        shooter->single_shoot_done = 1;
    }
}

/**
 * @brief      Motor locked rotor judge
 * @param      NULL
 * @retval     NULL
 */
void Shoot_FeederLockedJudge()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    static int count = 0;
    if (shooter->feeder_mode == FEEDER_INITING)
    {
        if ((fabs(Motor_feederMotor.encoder.current) >= INITDONE_CURRENT) && (fabs(Motor_feederMotor.encoder.speed) <= INITDONE_SPEED))
        {
            count++;
            if (count > INITDONE_TIME)
            {
                shooter->Initial_Feeder_Angle = Motor_feederMotor.encoder.consequent_angle - 5.0f;
                while (shooter->Initial_Feeder_Angle > 45.0f)
                {
                    shooter->Initial_Feeder_Angle -= 45.0f;
                }
                Shoot_FeederModeForceSet(FEEDER_STOP);
                PID_SetRef(&shooter->feed_ang, Motor_feederMotor.encoder.consequent_angle - 5.0f);
                count = 0;
            }
        }
        else
        {
            count = 0;
        }
    }
    else if (shooter->feeder_mode != FEEDER_LOCKED)
    {
        if ((fabs(Motor_feederMotor.encoder.current) >= LOCKED_CURRENT) && (fabs(Motor_feederMotor.encoder.speed) <= LOCKED_SPEED))
        {
            count++;
            if (count > LOCKED_TIME)
            {
                Shoot_FeederModeForceSet(FEEDER_LOCKED);
            }
        }
        else
        {
            count = 0;
        }
    }
}

/**
 * @brief      Motor locked handle
 * @param      NULL
 * @retval     NULL
 */
static void Shoot_FeederLockedHandle()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    static int count_reverse = 0;
    shooter->shoot_speed.feeder_shoot_speed = REVERSE_SPEED;
    count_reverse++;
    if (count_reverse >= RELOCKED_TIME)
    {
        count_reverse = 0;
        Shoot_FeederModeForceSet(FEEDER_STOP);
    }
}

/**
 * @brief      Gets the pointer to the shooter control data object
 * @param      NULL
 * @retval     Pointer to shooter control data object
 */
Shoot_ControlTypeDef *Shoot_GetControlPtr()
{
    return &Shoot_Control;
}

/**
 * @brief      Change shooter mode
 * @param      mode: Feeder mode
 * @retval     NULL
 */
void Shoot_FeederModeSet(Feeder_ModeEnum mode)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    if (shooter->feeder_mode == FEEDER_LOCKED)
    {
        return;
    }
    shooter->last_feeder_mode = shooter->feeder_mode;
    shooter->feeder_mode = mode;
    if ((shooter->feeder_mode != shooter->last_feeder_mode) && (shooter->last_feeder_mode == FEEDER_REFEREE))
    {
        shooter->feeder_mode = FEEDER_FINISH;
        PID_SetRef(&shooter->feed_ang, (int)((PID_GetFdb(&shooter->feed_ang) - shooter->Initial_Feeder_Angle + 40.0f) / 45) * 45 + shooter->Initial_Feeder_Angle);
    }
}

/**
 * @brief      Force change shooter mode
 * @param      mode: Feeder mode
 * @retval     NULL
 */
void Shoot_FeederModeForceSet(Feeder_ModeEnum mode)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    shooter->feeder_mode = mode;
}
