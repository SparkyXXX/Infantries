/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Chen Zhihong
 * @LastEditTime: 2024-08-08 16:49:26
 */

#include "app_shoot.h"
#include "config_ctrl.h"
#include "lib_math.h"
#include "periph_motor_pwm.h"
#include "protocol_board.h"
#include "util_adc.h"

Shoot_ControlTypeDef Shoot_Control;
Shoot_ControlTypeDef *Shoot_GetControlPtr()
{
    return &Shoot_Control;
}

void Shoot_Init()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    shooter->shooter_mode = SHOOTER_STOP;
    shooter->feeder_mode = FEEDER_INITING;
    shooter->heat_now = 0;
    shooter->heat_now_referee = 0;
    shooter->heat_limit = 0;
    shooter->shoot_speed.left_speed_ref = 0;
    shooter->shoot_speed.right_speed_ref = 0;
    shooter->shoot_freq_ref = 0;
    for (int i = 0; i < 5; i++)
    {
        shooter->shoot_speed.referee_bullet_speed[i] = 28.0f;
    }
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

uint32_t shoot_tick_start = 0;
uint32_t shoot_tick_diff = 0;
float snail_diff = 0.0f;
void ShootSpeed_Update()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();

    shooter->shoot_speed.referee_bullet_speed[4] = shooter->shoot_speed.referee_bullet_speed[3];
    shooter->shoot_speed.referee_bullet_speed[3] = shooter->shoot_speed.referee_bullet_speed[2];
    shooter->shoot_speed.referee_bullet_speed[2] = shooter->shoot_speed.referee_bullet_speed[1];
    shooter->shoot_speed.referee_bullet_speed[1] = shooter->shoot_speed.referee_bullet_speed[0];
    shooter->shoot_speed.referee_bullet_speed[0] = boardcom->shoot_spd_referee;
    float sum;
    for (int i = 0; i < 5; i++)
    {
        sum += shooter->shoot_speed.referee_bullet_speed[i];
    }

    if (shooter->shoot_speed.referee_bullet_speed[0] != shooter->shoot_speed.referee_bullet_speed[1])
    {
        shoot_tick_diff = HAL_GetTick() - shoot_tick_start;
    }
    shooter->shoot_speed.average_bullet_speed = sum / 5;
    Motor_PWM_ReadEncoder(&Motor_shooterMotorLeft);
    Motor_PWM_ReadEncoder(&Motor_shooterMotorRight);
    snail_diff = shooter->shoot_speed.left_speed_fdb - shooter->shoot_speed.right_speed_fdb;
}

float feeder_tick_last = 0.0f;
float angle_diff_sum = 0.0f;
int ammunition = 0;
uint8_t if_bullet = 0;
void Heat_Control()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    shooter->heat_now_referee = (float)boardcom->shooter_heat_referee;
    // shooter->heat_now = (float)boardcom->heat_now;

    angle_diff_sum += angle_diff;
    if (shooter->feeder_mode == FEEDER_LOCKED)
    {
        angle_diff_sum = (angle_diff_sum < 0.0f) ? angle_diff_sum + 45 : angle_diff_sum;
        shooter->heat_now += -boardcom->cooling_per_second * (DWT_GetTimeline_ms() - feeder_tick_last) / 1000;
        shooter->heat_now = (shooter->heat_now < 0) ? 0 : shooter->heat_now;
    }
    else
    {
        if_bullet = (angle_diff_sum > 45.0f) ? 1 : 0;
        if (if_bullet)
        {
            angle_diff_sum = angle_diff_sum - 45.0f;
        }
        ammunition += if_bullet;
        shooter->heat_now += -boardcom->cooling_per_second * (DWT_GetTimeline_ms() - feeder_tick_last) / 1000 + 10 * if_bullet;
        shooter->heat_now = (shooter->heat_now < 0) ? 0 : shooter->heat_now;
    }
    feeder_tick_last = DWT_GetTimeline_ms();
    if_bullet = 0;
    shooter->heat_limit = (float)boardcom->heat_limit;
    if (shooter->heat_now_referee > shooter->heat_now)
    {
        shooter->heat_now = shooter->heat_now_referee;
    }

    if ((shooter->heat_limit - shooter->heat_now) >= HEAT_LIMIT)
    {
        shooter->shoot_freq_ref = shooter->fast_shoot_freq;
    }
    else if ((shooter->heat_limit - shooter->heat_now) < HEAT_LIMIT)
    {
        shooter->shoot_freq_ref = 0;
    }
}

float shoot_ref = 25.0f;
float kf = 1.0f;
void Shoot_ShooterControl()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    switch (shooter->shooter_mode)
    {
    case SHOOTER_STOP:
        shooter->shoot_speed.left_speed_ref = 0;
        shooter->shoot_speed.right_speed_ref = 0;
        shooter->shoot_left.sum = 0;
        shooter->shoot_right.sum = 0;
        break;
    case SHOOTER_REFEREE:
        shooter->shoot_speed.left_speed_ref = shoot_ref;
        shooter->shoot_speed.right_speed_ref = shoot_ref;
        break;
    default:
        break;
    }

#if SHOOTER_MODE == CLOSELOOP_CONTROL
    PID_SetRef(&(shooter->shoot_left), shooter->shoot_speed.left_speed_ref);
    PID_SetRef(&(shooter->shoot_right), shooter->shoot_speed.right_speed_ref);
    shooter->shoot_speed.left_speed_fdb = Filter_Lowpass(Motor_shooterMotorLeft.encoder.speed, &(shooter->shooter_left_fdb_lpf));
    shooter->shoot_speed.right_speed_fdb = Filter_Lowpass(Motor_shooterMotorRight.encoder.speed, &(shooter->shooter_right_fdb_lpf));
    PID_SetFdb(&(shooter->shoot_left), shooter->shoot_speed.left_speed_fdb);
    PID_SetFdb(&(shooter->shoot_right), shooter->shoot_speed.right_speed_fdb);
    MotorPWM_SetOutput(&Motor_shooterMotorLeft, kf * shooter->shoot_speed.left_speed_ref + PID_Calc(&(shooter->shoot_left)));
    MotorPWM_SetOutput(&Motor_shooterMotorRight, kf * shooter->shoot_speed.right_speed_ref + PID_Calc(&(shooter->shoot_right)));
#endif

#if SHOOTER_MODE == OPENLOOP_TEST
    shooter->shoot_speed.left_speed_fdb = Filter_Lowpass(Motor_shooterMotorLeft.encoder.speed, &(shooter->shooter_fdb_lpf));
    shooter->shoot_speed.right_speed_fdb = Filter_Lowpass(Motor_shooterMotorRight.encoder.speed, &(shooter->shooter_fdb_lpf));
    MotorPWM_SetOutput(&Motor_shooterMotorLeft, kf * shooter->shoot_speed.left_speed_ref);
    MotorPWM_SetOutput(&Motor_shooterMotorRight, kf * shooter->shoot_speed.right_speed_ref);
#endif
}

void Shoot_FeederControl()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    switch (shooter->feeder_mode)
    {
    case FEEDER_STOP:
        shooter->shoot_mode = SINGLE;
        shooter->shoot_freq_ref = 0.0f;
        shooter->feed_spd.sum = 0;
        break;
    case FEEDER_SINGLE:
        shooter->shoot_mode = SINGLE;
        Shoot_Single();
        break;
    case FEEDER_LOCKED:
        shooter->shoot_mode = CONTINUOUS;
        Shoot_FeederLockedHandle();
        break;
    case FEEDER_REFEREE:
        shooter->shoot_mode = CONTINUOUS;
        break;
    case FEEDER_FINISH:
        shooter->shoot_mode = CONTINUOUS;
        break;
    case FEEDER_INITING:
        shooter->shoot_mode = CONTINUOUS;
        shooter->shoot_freq_ref = 10.0f;
        break;
    default:
        break;
    }
}

void Shoot_ShooterOutput()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if (boardcom->power_management_shooter_output == 0)
    {
        Motor_shooterMotors.motor_handle[0]->output = 0;
        Motor_shooterMotors.motor_handle[1]->output = 0;
    }
    Motor_PWM_SendOutput(&Motor_shooterMotorLeft);
    Motor_PWM_SendOutput(&Motor_shooterMotorRight);
    Motor_CAN_SendGroupOutput(&Motor_feederMotors);
}

void Shoot_FeederOutput()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    if (shooter->shoot_mode == CONTINUOUS)
    {
        PID_SetRef(&(shooter->feed_spd), shooter->shoot_freq_ref);
        if (shooter->shoot_speed.left_speed_fdb < 20 || shooter->shoot_speed.right_speed_fdb < 20 || boardcom->power_management_shooter_output == 0)
        {
            PID_SetRef(&(shooter->feed_spd), 0);
        }
        PID_SetFdb(&(shooter->feed_spd), Motor_feederMotor.encoder.speed);
        PID_Calc(&(shooter->feed_spd));
        PID_SetRef(&(shooter->feed_ang),
                   shooter->feeder_angle_init + (int)((Motor_feederMotor.encoder.consequent_angle -
                                                       shooter->feeder_angle_init + 40.0f) /
                                                      45) *
                                                    45);
    }
    else if (shooter->shoot_mode == SINGLE)
    {
        PID_SetFdb(&(shooter->feed_ang), Motor_feederMotor.encoder.consequent_angle);
        PID_SetRef(&(shooter->feed_spd), PID_Calc(&shooter->feed_ang));
        if (shooter->shoot_speed.left_speed_fdb < 20 || shooter->shoot_speed.right_speed_fdb < 20 || boardcom->power_management_shooter_output == 0)
        {
            PID_SetRef(&(shooter->feed_spd), 0);
        }
        PID_SetFdb(&(shooter->feed_spd), Motor_feederMotor.encoder.speed);
    }
    Motor_SetOutput(&Motor_feederMotor, PID_Calc(&(shooter->feed_spd)));
}

void Shoot_Single()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    if (fabs(shooter->feed_ang.fdb - shooter->feed_ang.ref) > 5.0f)
    {
        return;
    }
    if (!shooter->single_shoot_done && shooter->shoot_speed.left_speed_fdb > 20 && shooter->shoot_speed.right_speed_fdb > 20 && boardcom->power_management_shooter_output == 1)
    {
        shooter->feed_ang.ref += 45.0f;
        shooter->single_shoot_done = 1;
    }
}

void Shoot_FeederLockedJudge()
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    static int count = 0;
    static uint8_t shooter_done = 0;
    if (shooter->feeder_mode == FEEDER_INITING)
    {
        if (shooter->shoot_speed.left_speed_fdb > 24.0f && shooter->shoot_speed.left_speed_fdb < 25.0f)
        {
            shooter_done = 1;
        }
        if ((shooter_done == 1) && (shooter->shoot_speed.left_speed_fdb < 23.0f))
        {
            shooter->feeder_angle_init = Motor_feederMotor.encoder.consequent_angle + 2.0f;
            while (shooter->feeder_angle_init > 45.0f)
            {
                shooter->feeder_angle_init -= 45.0f;
            }
            Shoot_FeederModeForceSet(FEEDER_STOP);
            PID_SetRef(&(shooter->feed_ang), Motor_feederMotor.encoder.consequent_angle + 2.0f);
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
    shooter->shoot_freq_ref = REVERSE_SPEED;
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

void Shoot_FeederModeForceSet(Feeder_ModeEnum mode)
{
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    shooter->feeder_mode = mode;
}

/***************Shoot Mode Set***************************************************************************************/
void Remote_ShootModeSet()
{
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    static uint32_t wait_tick = 0;
    switch (remote->remote.s[SWITCH_LEFT])
    {
    case REMOTE_SWITCH_UP:
    {
        shooter->shooter_mode = SHOOTER_STOP;
        Shoot_FeederModeSet(FEEDER_STOP);
        break;
    }
    case REMOTE_SWITCH_MIDDLE:
    {
        shooter->shooter_mode = SHOOTER_REFEREE;
        Shoot_FeederModeSet(FEEDER_STOP);
        break;
    }
    case REMOTE_SWITCH_DOWN:
    {
#if SHOOT_DECIDE == AUTOAIM_DECIDE_SHOOT
        AutoAim_ShootModeSet();
#endif
#if SHOOT_DECIDE == REMOTE_DECIDE_SHOOT
        Shoot_FeederModeSet(FEEDER_REFEREE);
#endif
        break;
    }
    default:
        break;
    }
}

int press_time = 0;
void Keymouse_ShootModeSet()
{
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    if (remote->mouse.l == 1)
    {
        press_time++;
        if (press_time > 150)
        {
            Shoot_FeederModeSet(FEEDER_REFEREE);
        }
        else
        {
            Shoot_FeederModeSet(FEEDER_SINGLE);
        }
    }
    else
    {
        Shoot_FeederModeSet(FEEDER_STOP);
        shooter->single_shoot_done = 0;
        press_time = 0;
    }

    AutoAim_ShootModeSet();
}
int press_time_VTM = 0;
void Keymouse_VTM_ShootModeSet()
{
    ext_remote_control_t *remote = &(Referee_GetDataPtr()->remote_vtm_data);
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    if (remote->left_button_down == 1)
    {
        press_time_VTM++;
        if (press_time_VTM > 150)
        {
            Shoot_FeederModeSet(FEEDER_REFEREE);
        }
        else
        {
            Shoot_FeederModeSet(FEEDER_SINGLE);
        }
    }
    else
    {
        Shoot_FeederModeSet(FEEDER_STOP);
        shooter->single_shoot_done = 0;
        press_time_VTM = 0;
    }

    AutoAim_ShootModeSet();
}
extern uint32_t shoot_tick_start;
uint16_t wait_tick = 0;
uint8_t have_shooted = 0; // this is_shoot=1 time is have shooted to avoid one is_shoot time shoot twice
//  set when is_shoot==1 and shooted, and reset when is_shoot==0
void AutoAim_ShootModeSet()
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    /***************AutoShoot Start***************/
    if (remote_control->autoshoot_flag)
    {
        if (autoaim->hit_mode == AUTOAIM_HIT_ARMOR)
        {
            if ((shooter->heat_limit - shooter->heat_now) <= HEAT_LIMIT)
            {
                Shoot_FeederModeSet(FEEDER_STOP);
                return;
            }
            wait_tick++;
            if (minipc->is_get_target == 1 && minipc->is_shoot == 1 && remote_control->autoshoot_flag == 1 && wait_tick >= shooter->armor_wait_ms &&
                (shooter->heat_limit - shooter->heat_now) >= HEAT_LIMIT &&
                shooter->shoot_speed.left_speed_fdb > 20 && shooter->shoot_speed.right_speed_fdb > 20)
            {
                shoot_tick_start = HAL_GetTick();
                Shoot_FeederModeSet(FEEDER_SINGLE);
                shooter->single_shoot_done = 0;
                have_shooted = 1;
                wait_tick = 0;
            }
        }
        else if (autoaim->hit_mode == AUTOAIM_HIT_BUFF)
        {
            if (minipc->is_get_target == 1 && minipc->is_shoot == 1 && have_shooted == 0 &&
                (shooter->heat_limit - shooter->heat_now) >= HEAT_LIMIT &&
                shooter->shoot_speed.left_speed_fdb > 20 && shooter->shoot_speed.right_speed_fdb > 20)
            {
                Shoot_FeederModeForceSet(FEEDER_SINGLE);
                shooter->single_shoot_done = 0;
                have_shooted = 1;
            }
            else if (minipc->is_shoot == 0)
            {
                Shoot_FeederModeSet(FEEDER_STOP);
                shooter->single_shoot_done = 0;
                have_shooted = 0;
            }
        }
    }
    /***************AutoShoot End***************/
}
