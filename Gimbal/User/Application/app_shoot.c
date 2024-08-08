/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-20 02:10:33
 */

#include "app_shoot.h"

Shoot_ControlTypeDef Shoot_Control;

uint8_t On_FeederOneInc = 0;

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
    shooter->last_feeder_mode = FEEDER_STOP;
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
        shooter->shoot_speed.left_shoot_speed = 22.5f;
        shooter->shoot_speed.right_shoot_speed = 22.5f;
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
    case FEEDER_INITING:
        shoot_mode = CONTINUOUS;
        shooter->shoot_speed.feeder_shoot_speed = 10.0f;
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
        PID_SetRef(&shooter->feed_ang, 
					shooter->feeder_angle_init + (int)((Motor_feederMotor.encoder.consequent_angle - shooter->feeder_angle_init + 40.0f) / 45) * 45);
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
//    if ((HAL_GetTick() - shooter_cnt) < 7000)
//    {
//        Motor_shooterMotors.motor_handle[0]->output = 0;
//        Motor_shooterMotors.motor_handle[1]->output = 0;
//    }
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
void FeederOneInc()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
		
    if (Motor_shooterMotorLeft.encoder.speed > 20 && Motor_shooterMotorLeft.encoder.speed > 20 && ADC_Voltage > 15.0f)
    {
        PID_AddRef(&shooter->feed_ang, 4.0f);
			shooter->feeder_angle_init += 4.0f;
			if(shooter->feeder_angle_init > 45.0f){
				shooter->feeder_angle_init -= 45.0f;
			}
			On_FeederOneInc = 1;
    }
}
void FeederOneRemainInc()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();

    if (Motor_shooterMotorLeft.encoder.speed > 20 && Motor_shooterMotorLeft.encoder.speed > 20 && ADC_Voltage > 15.0f)
    {
        PID_AddRef(&shooter->feed_ang, 34.0f);
			shooter->feeder_angle_init += 34.0f;
			if(shooter->feeder_angle_init > 45.0f){
				shooter->feeder_angle_init -= 45.0f;
			}
			On_FeederOneInc = 1;
    }
}


/**
 * @brief      Motor locked rotor judge
 * @param      NULL
 * @retval     NULL
 */
float INITDONE_CURRENT = 3000.0f;
float INITDONE_SPEED = 10.0f;
float INITDONE_TIME = 50.0f;
float kadancurr = 9000.0f;
uint32_t feeder_init_tick = 0;
void Shoot_FeederLockedJudge()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
		if (shooter->last_feeder_mode != FEEDER_INITING || shooter->shooter_mode == SHOOTER_STOP){
					feeder_init_tick = HAL_GetTick();
		}
		if(shooter->shooter_mode == SHOOTER_STOP){return;}
    static int count = 0;
    static uint8_t shooter_done = 0;
//		if (shooter->feeder_mode == FEEDER_INITING){
//				if(HAL_GetTick() - feeder_init_tick > 2000){
//					PID_SetRef(&shooter->feed_ang, Motor_feederMotor.encoder.consequent_angle - 2.0f);
//					Shoot_FeederModeForceSet(FEEDER_STOP);
//				}
//				if ((fabs(Motor_feederMotor.encoder.current) >= INITDONE_CURRENT) && (fabs(Motor_feederMotor.encoder.speed) <= INITDONE_SPEED))
//        {
//            count++;
//            if (count > INITDONE_TIME)
//            {
//								shooter->feeder_angle_init = Motor_feederMotor.encoder.consequent_angle - 6.0f;
//								int dieWhileCnt = 0;
//								while(shooter->feeder_angle_init - 45.0f > 0){
//									shooter->feeder_angle_init -= 45.0f;
//									dieWhileCnt ++;
//									if(dieWhileCnt>100){
//										break;
//									}
//								}
//                Shoot_FeederModeForceSet(FEEDER_STOP);
//								PID_SetRef(&shooter->feed_ang, Motor_feederMotor.encoder.consequent_angle - 6.0f);
//								count = 0;
//            }
//        }
//        else
//        {
//            count = 0;
//        }
//				
//		}
	if (shooter->feeder_mode == FEEDER_INITING)
    {
        if (Motor_shooterMotorLeft.encoder.speed > 22.5 && Motor_shooterMotorLeft.encoder.speed < 25.0)
        {
            shooter_done = 1;
        }
        if ((shooter_done == 1) && (Motor_shooterMotorLeft.encoder.speed < 22.0))
        {
            shooter->feeder_angle_init = Motor_feederMotor.encoder.consequent_angle + 5.0f;
            while (shooter->feeder_angle_init > 45.0f)
            {
                shooter->feeder_angle_init -= 45.0f;
            }
            Shoot_FeederModeForceSet(FEEDER_STOP);
            PID_SetRef(&(shooter->feed_ang), Motor_feederMotor.encoder.consequent_angle + 5.0f);
        }
    }
    else if (shooter->feeder_mode != FEEDER_LOCKED)
    {
        if ((fabs(Motor_feederMotor.encoder.current) >= kadancurr) && (fabs(Motor_feederMotor.encoder.speed) <= LOCKED_SPEED))//LOCKED_CURRENT
        {
            count++;
            if (count > LOCKED_TIME)
            {
              Shoot_FeederModeForceSet(FEEDER_LOCKED);
							//Shoot_FeederLockedHandle();
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
    if (shooter->feeder_mode == FEEDER_LOCKED || shooter->feeder_mode == FEEDER_INITING)
    {
        return;
    }
    shooter->last_feeder_mode = shooter->feeder_mode;
    shooter->feeder_mode = mode;
    if ((shooter->feeder_mode != shooter->last_feeder_mode) && (shooter->last_feeder_mode == FEEDER_REFEREE))
    {
        shooter->feeder_mode = FEEDER_FINISH;
        PID_SetRef(&shooter->feed_ang, 
					shooter->feeder_angle_init + (int)((PID_GetFdb(&shooter->feed_ang) - shooter->feeder_angle_init + 40.0f) / 45) * 45); // feeder angle correct
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
