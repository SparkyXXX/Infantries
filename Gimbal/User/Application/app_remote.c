/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-20 01:33:55
 */

#include "app_remote.h"

#define REMOTE_MODE TRAIN

#define DEBUG 1
#define TRAIN 2

Remote_ControlTypeDef Remote_Control;
Keyboard_DataTypeDef Remote_Keylast;
Keyboard_DataTypeDef Remote_KeyZero;

/**
 * @brief      Remote control command
 * @param      NULL
 * @retval     NULL
 */
void Remote_DriveModeSet()
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    remote_control->pending = 1;
    if (Remote_IsLost(remote))
    {
        Remote_Reset(remote);
    }

    switch (remote->remote.s[SWITCH_RIGHT])
    {
    case REMOTE_SWITCH_NULL:
    {
        remote_control->on_aim = 0;
        autoaim->isChangeTarget = 0;
        remote_control->aim_mode = REMOTE_ARMOR;
        Remote_Update();
        Remote_ShootModeSet();
        Servo_SetAngle(&Servo_MagServo, Servo_Close);
        Remote_Mag_State = 0;
        break;
    }
    case REMOTE_SWITCH_UP:
    {
#if REMOTE_MODE == DEBUG
        remote_control->on_aim = 1;
        autoaim->isChangeTarget = 1;
        remote_control->aim_mode = REMOTE_BIG_BUFF;
        Remote_Update();
        Remote_ShootModeSet();
        Servo_SetAngle(&Servo_MagServo, Servo_Open);
        Remote_Mag_State = 1;
#endif
#if REMOTE_MODE == TRAIN
        remote_control->on_aim = 0;
        autoaim->isChangeTarget = 0;
        remote_control->aim_mode = REMOTE_ARMOR;
        Remote_Update();
        Remote_ShootModeSet();
        Servo_SetAngle(&Servo_MagServo, Servo_Close);
        Remote_Mag_State = 0;
#endif
        break;
    }
    case REMOTE_SWITCH_MIDDLE:
    {
#if REMOTE_MODE == DEBUG
        remote_control->on_aim = 1;
        autoaim->isChangeTarget = 1;
        remote_control->aim_mode = REMOTE_SMALL_BUFF;
        Remote_Update();
        Remote_ShootModeSet();
        Servo_SetAngle(&Servo_MagServo, Servo_Open);
        Remote_Mag_State = 1;
#endif
#if REMOTE_MODE == TRAIN
        Keymouse_Update();
        Keymouse_ShootModeSet();
#endif
        break;
    }
    case REMOTE_SWITCH_DOWN:
    {
#if REMOTE_MODE == DEBUG
        remote_control->on_aim = 1;
        autoaim->isChangeTarget = 1;
		autoaim->AutoShootFlag = 1;
        remote_control->aim_mode = REMOTE_ARMOR;
        Remote_Update();
        Remote_ShootModeSet();
        Servo_SetAngle(&Servo_MagServo, Servo_Open);
        Remote_Mag_State = 0;
#endif
#if REMOTE_MODE == TRAIN
        remote_control->on_aim = 1;
        autoaim->isChangeTarget = 1;
        remote_control->aim_mode = REMOTE_ARMOR;
        Remote_Update();
        Remote_ShootModeSet();
        Servo_SetAngle(&Servo_MagServo, Servo_Open);
        Remote_Mag_State = 1;
#endif
        break;
    }
    default:
        break;
    }
    Following_AutoaimModeSet();
    remote_control->pending = 0;
}

/**
 * @brief      AutoAim_ShootModeSet
 * @param      NULL
 * @retval     NULL
 */
uint16_t wait_tick = 0;
uint8_t have_shooted = 0; // this is_shoot=1 time is have shooted to avoid one is_shoot time shoot twice
                          //  set when is_shoot==1 and shooted, and reset when is_shoot==0
void AutoAim_ShootModeSet()
{
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    uint16_t wait_ms = 1000;
    if (autoaim->aim_mode == AUTOAIM_ARMOR)
    {
        wait_ms = AutoShoot_Wait_ms;
    }
    else if (autoaim->aim_mode == AUTOAIM_SMALL_BUFF)
    {
        wait_ms = AutoShootSmallEnergy_Wait_ms;
    }
    else if (autoaim->aim_mode == AUTOAIM_BIG_BUFF)
    {
        wait_ms = AutoShootBigEnergy_Wait_ms;
    }
    if (autoaim->hit_mode == AUTOAIM_HIT_ARMOR)
    {
        wait_tick++;
        if (minipc->is_get_target == 1 && minipc->is_shoot == 1 && (autoaim->AutoShootFlag == 1 || remote->remote.s[SWITCH_RIGHT] == REMOTE_SWITCH_DOWN) && wait_tick >= wait_ms &&
            (shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= HEAT_SLOW_LIMIT && Motor_shooterMotorLeft.encoder.speed > 20 && Motor_shooterMotorRight.encoder.speed > 20)
        {
            Shoot_FeederModeForceSet(FEEDER_SINGLE);
            wait_tick = 0;
						shooter->single_shoot_done = 0;
        }
        // Shoot_FeederModeSet(FEEDER_REFEREE);//use to test the shoot, DO NOT OPEN!
    }
    else if (autoaim->hit_mode == AUTOAIM_HIT_BUFF)
    {
        if (minipc->is_get_target == 1 && minipc->is_shoot == 1 && have_shooted == 0 &&
            (shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= HEAT_SLOW_LIMIT && Motor_shooterMotorLeft.encoder.speed > 20 && Motor_shooterMotorRight.encoder.speed > 20)
        {
            Shoot_FeederModeForceSet(FEEDER_SINGLE);
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

/**
 * @brief      Remote shoot mode set
 * @param      NULL
 * @retval     NULL
 */
static void Remote_ShootModeSet()
{
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    static uint32_t wait_tick;
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
#if REMOTE_MODE == DEBUG
        AutoAim_ShootModeSet();
#endif
#if REMOTE_MODE == TRAIN
        Shoot_FeederModeSet(FEEDER_REFEREE);
#endif
        break;
    }
    default:
        break;
    }
}

uint8_t Remote_Chassis_Gyro_State = 0;
uint8_t Remote_Mag_State = 0;
uint8_t Remote_Mag_State_Last = 0;
uint8_t quiet_flag = 0;
uint8_t flyslope_flag = 0;
uint8_t cap_speedup_flag = 0;
static void Remote_Update()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();

    boardcom->chassis_fb_ref = (float)remote->remote.ch[RIGHT_Y] * 440 / 660;
    boardcom->chassis_lr_ref = (float)remote->remote.ch[RIGHT_X] * 440 / 660;

    if (remote->remote.ch[PADDLE_WHEEL] <= -500.0f)
    {
        Remote_Chassis_Gyro_State = 1;
    }
    else if (remote->remote.ch[PADDLE_WHEEL] >= 500.0f)
    {
        Remote_Chassis_Gyro_State = 2;
    }
    else
    {
        Remote_Chassis_Gyro_State = 0;
    }

    float yaw_ref, pitch_ref, temp_pitch_ref;
    yaw_ref = (float)remote->remote.ch[LEFT_X] * Remote_Yaw_To_Ref;
    pitch_ref = (float)remote->remote.ch[LEFT_Y] * Remote_Pitch_To_Ref;

    LimitMaxMin(yaw_ref, 0.5f, -0.5f);
    gimbal->yaw_position_ref = gimbal->yaw_position_ref - yaw_ref;
    temp_pitch_ref = gimbal->pitch_position_ref + pitch_ref;
    LimitMaxMin(temp_pitch_ref, Elevation_Angle, Depression_Angle);
    gimbal->pitch_position_ref = temp_pitch_ref;
}

int press_time = 0;
static void Keymouse_ShootModeSet()
{
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    AutoAim_ShootModeSet(); // autoaim is lower than NoAuto, so NoAuto set after AutoAim
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
}

/**
 * @brief      KeyMouse control process
 * @param      NULL
 * @retval     NULL
 */
static void Keymouse_Update()
{
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    Keyboard_DataTypeDef *keyboard = &(Remote_GetDataPtr()->key);
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();

    static uint8_t wait4release = 0;

    if (KEY(ctrl))
    {
        quiet_flag = 1;
        flyslope_flag = 0;
        if (wait4release <= 1)
        {
            if (KEY_DN(f))
            {
                shooter->shooter_mode = SHOOTER_STOP;
                wait4release = 1;
            }
            if (KEY_DN(q))
            {
                Servo_SetAngle(&Servo_MagServo, Servo_Close);
                Remote_Mag_State = 0;
                wait4release = 1;
            }
            if (KEY_DN(r))
            {
                autoaim->AutoShootFlag = 0;
                wait4release = 1;
            }
            if (KEY_DN(c))
            {
                cap_speedup_flag = 0;
            }
        }
    }
    else if (KEY(shift))
    {
        flyslope_flag = 1;
        if (wait4release <= 1)
        {
            if (KEY_DN(c))
            {
                boardcom->ui_cmd = !boardcom->ui_cmd;
                wait4release = 1;
            }
        }
    }
    else
    {
        quiet_flag = 0;
        flyslope_flag = 0;
        if (wait4release == 0)
        {
            if (KEY_DN(f))
            {
                shooter->shooter_mode = SHOOTER_REFEREE;
            }
            if (KEY_UP(e))
            {
                Remote_Gyro_ModeSet();
            }
            if (KEY_UP(v))
            {
                Remote_AutoaimModeSet(REMOTE_SMALL_BUFF);
            }
            if (KEY_UP(b))
            {
                Remote_AutoaimModeSet(REMOTE_BIG_BUFF);
            }
            // 保留功能，以后如果再出一个自瞄模式可以用
            //            if (KEY_UP(g))
            //            {
            //				Remote_AutoaimModeSet(REMOTE_GYRO);
            //            }
            if (KEY_DN(q))
            {
                Servo_SetAngle(&Servo_MagServo, Servo_Open);
                Remote_Mag_State = 1;
            }
            if (KEY_DN(r))
            {
                autoaim->AutoShootFlag = 1;
            }
            if (KEY_DN(c))
            {
                cap_speedup_flag = 1;
            }
        }
    }

    if (wait4release == 0)
    {
        /******** MOVE function ******/
        if (!KEY(f) && !KEY2(ctrl, shift) && !KEY(ctrl))
        {
            static float t_ws = 0.0f;
            static float t_ws_last = 0.0f;
            if (flyslope_flag == 1)
            {
                boardcom->power_limit_mode = POWER_UNLIMIT;
                boardcom->chassis_fb_ref = (KEY(w) - KEY(s)) * KeyMouse_FlySlopeSpeed;
                boardcom->chassis_lr_ref = (KEY(d) - KEY(a)) * KeyMouse_FlySlopeSpeed;
            }
            else if (flyslope_flag == 0 && cap_speedup_flag == 1)
            {
                boardcom->power_limit_mode = POWER_LIMIT;
                boardcom->cap_speedup_flag = CAP_SPEEDUP;
                boardcom->chassis_fb_ref = (KEY(w) - KEY(s)) * KeyMouse_NormalSpeed;
                boardcom->chassis_lr_ref = (KEY(d) - KEY(a)) * KeyMouse_NormalSpeed;
            }
            else if (flyslope_flag == 0 && cap_speedup_flag == 0)
            {
                boardcom->power_limit_mode = POWER_LIMIT;
                boardcom->cap_speedup_flag = CAP_NORMAL;
                boardcom->chassis_fb_ref = (KEY(w) - KEY(s)) * KeyMouse_NormalSpeed;
                boardcom->chassis_lr_ref = (KEY(d) - KEY(a)) * KeyMouse_NormalSpeed;
            }
        }
    }
    else
    {
        if (memcmp(keyboard, &Remote_KeyZero, sizeof(Remote_KeyZero)) == 0)
        {
            wait4release = 0;
        }
    }
    memcpy(&Remote_Keylast, keyboard, sizeof(Remote_Keylast));

    /*********** mouse control********/
    static uint8_t mouse_r_last = 0;

    if (!mouse_r_last && remote->mouse.r)
    {
        Keymouse_AutoaimModeSet(1);
        autoaim->isChangeTarget = 1;
    }
    else if (mouse_r_last && !remote->mouse.r)
    {
        Keymouse_AutoaimModeSet(0);
        autoaim->isChangeTarget = 0;
    }
    mouse_r_last = remote->mouse.r;

    if (gimbal->present_mode == GIMBAL_NO_AUTO || (autoaim->target_state != AUTOAIM_TARGET_FOLLOWING))
    {
        float yaw, pitch, temp_pitch_ref;
        if (quiet_flag)
        {
            yaw = (float)remote->mouse.x * Mouse_Yaw_To_Ref_Quiet;
            pitch = -(float)remote->mouse.y * Mouse_Pitch_To_Ref_Quiet;
        }
        else
        {
            yaw = (float)remote->mouse.x * Mouse_Yaw_To_Ref;
            pitch = -(float)remote->mouse.y * Mouse_Pitch_To_Ref;
        }

        LimitMaxMin(yaw, 0.5f, -0.5f);
        gimbal->yaw_position_ref = gimbal->yaw_position_ref - yaw;
        temp_pitch_ref = gimbal->pitch_position_ref + pitch;
        LimitMaxMin(temp_pitch_ref, Elevation_Angle, Depression_Angle);
        gimbal->pitch_position_ref = temp_pitch_ref;
    }
}

/**
 * @brief      switch autoaim state
 * @param      Remote_AutoAimModeEnum mode
 * @retval     NULL
 */
static void Remote_AutoaimModeSet(uint8_t mode)
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    if (remote_control->aim_mode == REMOTE_ARMOR)
    {
        remote_control->aim_mode = mode;
    }
    else if (remote_control->aim_mode == mode)
    {
        remote_control->aim_mode = REMOTE_ARMOR;
    }
}

/**
 * @brief      switch autoaim state
 * @param      state :on or off
 * @retval     NULL
 */
static void Keymouse_AutoaimModeSet(uint8_t mode)
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();

    static uint8_t temp_last_mode = REMOTE_ARMOR;
    if (mode == 1)
    {
        remote_control->on_aim = 1;
        switch (remote_control->aim_mode)
        {
        case REMOTE_ARMOR:
        case REMOTE_ARMOR_TEMP:
        case REMOTE_GYRO:
            break;
        case REMOTE_SMALL_BUFF:
            temp_last_mode = REMOTE_SMALL_BUFF;
            remote_control->aim_mode = REMOTE_ARMOR_TEMP;
            break;
        case REMOTE_BIG_BUFF:
            temp_last_mode = REMOTE_BIG_BUFF;
            remote_control->aim_mode = REMOTE_ARMOR_TEMP;
            break;
        default:
            break;
        }
    }
    else
    {
        remote_control->on_aim = 0;
        switch (remote_control->aim_mode)
        {
        default:
            break;
        case REMOTE_ARMOR:
        case REMOTE_GYRO:
            break;
        case REMOTE_ARMOR_TEMP:
            remote_control->aim_mode = temp_last_mode;
        }
    }
}

static void Following_AutoaimModeSet()
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    switch (remote_control->aim_mode)
    {
    case REMOTE_ARMOR:
    case REMOTE_ARMOR_TEMP:
        AutoAim_ModeSet(AUTOAIM_ARMOR);
        autoaim->hit_mode = AUTOAIM_HIT_ARMOR;
        if (remote_control->on_aim)
        {
            Gimbal_ModeSet(GIMBAL_ARMOR);
        }
        else
        {
            Gimbal_ModeSet(GIMBAL_NO_AUTO);
        }
        Remote_Chassis_ModeSet(CHASSIS_NORMAL);
        break;
    case REMOTE_GYRO:
        AutoAim_ModeSet(AUTOAIM_GYRO);
        autoaim->hit_mode = AUTOAIM_HIT_ARMOR;
        if (remote_control->on_aim)
        {
            Gimbal_ModeSet(GIMBAL_ARMOR);
        }
        else
        {
            Gimbal_ModeSet(GIMBAL_NO_AUTO);
        }
        Remote_Chassis_ModeSet(CHASSIS_NORMAL);
        break;
    case REMOTE_SMALL_BUFF:
        AutoAim_ModeSet(AUTOAIM_SMALL_BUFF);
        autoaim->hit_mode = AUTOAIM_HIT_BUFF;
        Gimbal_ModeSet(GIMBAL_SMALL_ENERGY);
        Remote_Chassis_ModeSet(CHASSIS_NORMAL);
        break;
    case REMOTE_BIG_BUFF:
        AutoAim_ModeSet(AUTOAIM_BIG_BUFF);
        autoaim->hit_mode = AUTOAIM_HIT_BUFF;
        Gimbal_ModeSet(GIMBAL_BIG_ENERGY);
        Remote_Chassis_ModeSet(CHASSIS_NORMAL);
        break;
    default:
        break;
    }
}

/**
 * @brief      Change chassis control state
 * @param      Chassis: control mode
 * @retval     NULL
 */
static void Remote_Chassis_ModeSet(uint8_t chassis_mode)
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if (chassis_mode == CHASSIS_NORMAL && Remote_Chassis_Gyro_State == 1)
    {
        boardcom->chassis_mode = CHASSIS_GYRO;
    }
    else if (chassis_mode == CHASSIS_NORMAL && Remote_Chassis_Gyro_State == 2)
    {
        boardcom->chassis_mode = CHASSIS_REVERSE_GYRO;
    }
    else
    {
        boardcom->chassis_mode = chassis_mode;
    }
}

/**
 * @brief      switch chassis gryo state
 * @param      NULL
 * @retval     NULL
 */
static void Remote_Gyro_ModeSet(void)
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if (boardcom->chassis_mode == CHASSIS_STOP)
    {
        return;
    }
    if (Remote_Chassis_Gyro_State == 0)
    {
        Remote_Chassis_ModeSet(CHASSIS_GYRO);
        Remote_Chassis_Gyro_State = 1;
    }
    else
    {
        Remote_Chassis_ModeSet(CHASSIS_NORMAL);
        Remote_Chassis_Gyro_State = 0;
    }
}

/**
 * @brief      Gets the pointer to the remote control remote object
 * @param      NULL
 * @retval     Pointer to remote control remote object
 */
Remote_ControlTypeDef *Remote_GetControlPtr()
{
    return &Remote_Control;
}
