/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-06-03 01:00:07
 */

#include "app_shoot.h"

Shoot_ControlTypeDef Shoot_Control;
Shoot_ControlTypeDef *Shoot_GetControlPtr()
{
    return &Shoot_Control;
}

void Shoot_Init()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    shooter->shooter_mode = SHOOTER_STOP;
    shooter->feeder_mode = FEEDER_INITING;
    shooter->heat_ctrl.shooter_heat_now = 0;
    shooter->heat_ctrl.shooter_heat_limit = 0;
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

float heat_now = 0.0f;
void Shoot_Update()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	// shooter->heat_ctrl.shooter_heat_now = (float)boardcom->heat_now;
	shooter->heat_ctrl.shooter_heat_now = heat_now;
	shooter->heat_ctrl.shooter_heat_limit = (float)boardcom->heat_limit;
    Motor_PWM_ReadEncoder(&Motor_shooterMotorLeft, 1);
    Motor_PWM_ReadEncoder(&Motor_shooterMotorRight, 1);
    ADC_Decode();
}

float feeder_tick_last = 0.0f;
float angle_diff_sum = 0.0f;
int ammunition = 0;
uint8_t if_bullet = 0;
void Heat_Update()
{
	Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

	angle_diff_sum += angle_diff;
	if (shooter->feeder_mode == FEEDER_LOCKED)
	{
		angle_diff_sum = angle_diff_sum < 0.0f ? angle_diff_sum + 45 : angle_diff_sum;
		heat_now += -boardcom->cooling_per_second * (DWT_GetTimeline_ms() - feeder_tick_last) / 1000;
		heat_now += (heat_now < 0) ? 0 : heat_now;
	}
	else
	{
		if_bullet = (angle_diff_sum > 45.0f) ? 1 : 0;
		if (if_bullet)
		{
			angle_diff_sum = angle_diff_sum - 45.0f;
		}
		ammunition += if_bullet;
		heat_now += -boardcom->cooling_per_second *  (DWT_GetTimeline_ms() - feeder_tick_last) / 1000 + 10 * if_bullet;
		heat_now = (heat_now < 0) ? 0 : heat_now;
	}
	feeder_tick_last = DWT_GetTimeline_ms();
	if_bullet = 0;
}

uint16_t encoder_spd_l = 0;
uint16_t encoder_spd_r = 0;
float shoot_ref = 22.5f;
float open_loop_shoot_ref = 27.0f;
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
#if	IF_BULLET_SPD_TEST == NO_BULLET_SPD_TEST
        shooter->shoot_speed.left_shoot_speed = shoot_ref;
        shooter->shoot_speed.right_shoot_speed = shoot_ref;
#endif
#if	IF_BULLET_SPD_TEST == BULLET_SPD_TEST
		shooter->shoot_speed.left_shoot_speed = open_loop_shoot_ref;
        shooter->shoot_speed.right_shoot_speed = open_loop_shoot_ref;
#endif
        break;
    default:
        break;
    }

#if SHOOT_MODE == SHOOT_CLOSE_LOOP
    PID_SetRef(&(shooter->shoot_left), shooter->shoot_speed.left_shoot_speed);
    PID_SetRef(&(shooter->shoot_right), shooter->shoot_speed.right_shoot_speed);
    PID_SetFdb(&(shooter->shoot_left), Filter_Lowpass(Motor_shooterMotorLeft.encoder.speed, &shooter->shooter_left_lpf));
    PID_SetFdb(&(shooter->shoot_right), Filter_Lowpass(Motor_shooterMotorRight.encoder.speed, &shooter->shooter_right_lpf));
    MotorPWM_SetOutput(&Motor_shooterMotorLeft, PID_Calc(&(shooter->shoot_left)));
    MotorPWM_SetOutput(&Motor_shooterMotorRight, PID_Calc(&(shooter->shoot_right)));
#endif
#if SHOOT_MODE == SHOOT_OPEN_LOOP
	MotorPWM_SetOutput(&Motor_shooterMotorLeft, shooter->shoot_speed.left_shoot_speed);
	MotorPWM_SetOutput(&Motor_shooterMotorRight, shooter->shoot_speed.right_shoot_speed);
#endif
#if	IF_BULLET_SPD_TEST == BULLET_SPD_TEST
	encoder_spd_l = Motor_shooterMotorLeft.encoder.speed * 100;
	encoder_spd_r = Motor_shooterMotorRight.encoder.speed * 100;
#endif
}

int shoot_mode = 0;
int test_shoot_frq = 150;
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
#if	IF_BULLET_SPD_TEST == NO_BULLET_SPD_TEST
        shooter->shoot_speed.feeder_shoot_speed = shooter->heat_ctrl.feeder_speed;
#endif
#if	IF_BULLET_SPD_TEST == BULLET_SPD_TEST
	    shooter->shoot_speed.feeder_shoot_speed = test_shoot_frq;
#endif
        break;
    case FEEDER_FINISH:
        shoot_mode = CONTINUOUS;
        Shoot_HeatControl();
        break;
    case FEEDER_INITING:
        shoot_mode = CONTINUOUS;
#if	IF_BULLET_SPD_TEST == NO_BULLET_SPD_TEST
        shooter->shoot_speed.feeder_shoot_speed = 10.0f;
#endif
#if IF_BULLET_SPD_TEST == BULLET_SPD_TEST
	    shooter->shoot_speed.feeder_shoot_speed = test_shoot_frq;
#endif
        break;
    default:
        break;
    }

    if (shoot_mode == CONTINUOUS)
    {
        PID_SetRef(&(shooter->feed_spd), shooter->shoot_speed.feeder_shoot_speed);
#if IF_BULLET_SPD_TEST == NO_BULLET_SPD_TEST
        if (Motor_shooterMotorLeft.encoder.speed < 20 || Motor_shooterMotorLeft.encoder.speed < 20 || ADC_Voltage < 15.0f)
        {
            PID_SetRef(&(shooter->feed_spd), 0);
        }
#endif
#if IF_BULLET_SPD_TEST == BULLET_SPD_TEST
		if (Motor_shooterMotorLeft.encoder.speed < 10 || Motor_shooterMotorLeft.encoder.speed < 10 || ADC_Voltage < 15.0f)
        {
            PID_SetRef(&(shooter->feed_spd), 0);
        }
#endif
        PID_SetFdb(&(shooter->feed_spd), Motor_feederMotor.encoder.speed);
        PID_Calc(&(shooter->feed_spd));
        PID_SetRef(&(shooter->feed_ang),
                   shooter->feeder_angle_init + (int)((Motor_feederMotor.encoder.consequent_angle -
                                                           shooter->feeder_angle_init + 40.0f) / 45) * 45);
    }
    else if (shoot_mode == SINGLE)
    {
        PID_SetFdb(&(shooter->feed_ang), Motor_feederMotor.encoder.consequent_angle);
        PID_SetRef(&(shooter->feed_spd), PID_Calc(&shooter->feed_ang));
        if (Motor_shooterMotorLeft.encoder.speed < 20 || Motor_shooterMotorLeft.encoder.speed < 20 || ADC_Voltage < 15.0f)
        {
            PID_SetRef(&(shooter->feed_spd), 0);
        }
        PID_SetFdb(&(shooter->feed_spd), Motor_feederMotor.encoder.speed);
    }
    Motor_SetOutput(&Motor_feederMotor, PID_Calc(&(shooter->feed_spd)));
}

float Feeder_Fast_Speed = 0.0f;
float Feeder_Slow_Speed = 0.0f;
static void Shoot_HeatControl()
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    if ((shooter->heat_ctrl.shooter_heat_limit - shooter->heat_ctrl.shooter_heat_now) >= 80)
    {
        shooter->heat_ctrl.feeder_speed = Feeder_Fast_Speed;
    }
    else if ((shooter->heat_ctrl.shooter_heat_limit - shooter->heat_ctrl.shooter_heat_now) > 20)
    {
        shooter->heat_ctrl.feeder_speed = Feeder_Slow_Speed;
    }
    else if ((shooter->heat_ctrl.shooter_heat_limit - shooter->heat_ctrl.shooter_heat_now) <= 20)
    {
        shooter->heat_ctrl.feeder_speed = 0;
    }
}

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
    // if ((HAL_GetTick() - shooter_cnt) < 7000)
    // {
    //     Motor_shooterMotors.motor_handle[0]->output = 0;
    //     Motor_shooterMotors.motor_handle[1]->output = 0;
    // }
    shooter_adc_flag_last = shooter_adc_flag;

    Motor_PWM_SendOutput(&Motor_shooterMotorLeft);
    Motor_PWM_SendOutput(&Motor_shooterMotorRight);
    Motor_CAN_SendGroupOutput(&Motor_feederMotors);
}

void Shoot_Single()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();

    if (fabs(shooter->feed_ang.fdb - shooter->feed_ang.ref) > 5.0f)
    {
        return;
    }
    if (!shooter->single_shoot_done && Motor_shooterMotorLeft.encoder.speed > 20 && Motor_shooterMotorLeft.encoder.speed > 20 && ADC_Voltage > 15.0f)
    {
		shooter->feed_ang.ref += 45.0f;
        shooter->single_shoot_done = 1;
    }
}

float INITDONE_CURRENT = 1000.0f;
float INITDONE_SPEED = 10.0f;
float INITDONE_TIME = 5.0f;
float feeder_offset = 2.0f;
void Shoot_FeederLockedJudge()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    static int count = 0;
    static uint8_t shooter_done = 0;
    if (shooter->feeder_mode == FEEDER_INITING)
    {
        if (Motor_shooterMotorLeft.encoder.speed > 22.5 && Motor_shooterMotorLeft.encoder.speed < 25.0)
        {
            shooter_done = 1;
        }
        if ((shooter_done == 1) && (Motor_shooterMotorLeft.encoder.speed < 21.0))
        {
            shooter->feeder_angle_init = Motor_feederMotor.encoder.consequent_angle + feeder_offset;
            while (shooter->feeder_angle_init > 45.0f)
            {
                shooter->feeder_angle_init -= 45.0f;
            }
            Shoot_FeederModeForceSet(FEEDER_STOP);
            PID_SetRef(&(shooter->feed_ang), Motor_feederMotor.encoder.consequent_angle + feeder_offset);
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

void Shoot_FeederModeSet(Feeder_ModeEnum mode)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    if (shooter->feeder_mode == FEEDER_LOCKED || shooter->feeder_mode == FEEDER_INITING)
    {
        return;
    }
    shooter->last_feeder_mode = shooter->feeder_mode;
    shooter->feeder_mode = mode;
    if ((shooter->feeder_mode != shooter->last_feeder_mode) && (shooter->last_feeder_mode == FEEDER_REFEREE))
    {
        shooter->feeder_mode = FEEDER_FINISH;
        PID_SetRef(&(shooter->feed_ang),
                   shooter->feeder_angle_init + (int)((shooter->feed_ang.fdb - shooter->feeder_angle_init + 40.0f) / 45) * 45);
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
