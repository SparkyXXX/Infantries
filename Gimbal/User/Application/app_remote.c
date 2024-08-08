/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Chen Zhihong
 * @LastEditTime: 2024-08-08 16:49:44
 */

#include "app_remote.h"
#include "periph_ext_remote.h"

uint8_t Remote_Lost_Flag = 0;

Remote_ControlTypeDef Remote_Control;
Keyboard_DataTypeDef Remote_Keylast;
Keyboard_DataTypeDef Remote_KeyZero;
Keyboard_VTM_DataTypeDef Remote_Keylast_VTM;
Keyboard_VTM_DataTypeDef Remote_KeyZero_VTM;
Remote_ControlTypeDef *Remote_GetControlPtr()
{
    return &Remote_Control;
}

/***************Drive Mode Set***************************************************************************************/
void Remote_DriveModeSet()
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    remote_control->pending = 1;
    if (Remote_IsLost(remote))
    {
        Remote_Reset(remote);
    }
    // 断连检测由于Remote_ControlTypeDef结构体固有缺陷，不建议使用。

    if (HAL_GetTick() - remote->last_update_time > 500)
    {
        Keymouse_VTM_Update();
        Keymouse_VTM_ShootModeSet();
    }

    else
    {

        switch (remote->remote.s[SWITCH_RIGHT])
        {
        case REMOTE_SWITCH_NULL:
        {
            remote_control->on_aim = 0;
            remote_control->aim_mode = REMOTE_ARMOR;
            Remote_Update();
            Remote_ShootModeSet();
            Servo_SetAngle(&Servo_MagServo, Servo_Close);
            remote_control->mag_state = 0;
            break;
        }
        case REMOTE_SWITCH_UP:
        {
#if REMOTE_MODE == DEBUG
            remote_control->on_aim = 1;
            remote_control->aim_mode = REMOTE_BIG_BUFF;
            Remote_Update();
            Remote_ShootModeSet();
            remote_control->autoshoot_flag = 1;
            Servo_SetAngle(&Servo_MagServo, Servo_Open);
            remote_control->mag_state = 1;
#endif
#if REMOTE_MODE == TRAIN
            remote_control->on_aim = 0;
            remote_control->aim_mode = REMOTE_ARMOR;
            Remote_Update();
            Remote_ShootModeSet();
            Servo_SetAngle(&Servo_MagServo, Servo_Close);
            remote_control->mag_state = 0;
#endif
            break;
        }
        case REMOTE_SWITCH_MIDDLE:
        {
#if REMOTE_MODE == DEBUG
            remote_control->on_aim = 1;
            remote_control->aim_mode = REMOTE_SMALL_BUFF;
            Remote_Update();
            Remote_ShootModeSet();
            remote_control->autoshoot_flag = 1;
            Servo_SetAngle(&Servo_MagServo, Servo_Open);
            remote_control->mag_state = 1;
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
            remote_control->aim_mode = REMOTE_ARMOR;
            Remote_Update();
            Remote_ShootModeSet();
            remote_control->autoshoot_flag = 1;
            Servo_SetAngle(&Servo_MagServo, Servo_Close);
            remote_control->mag_state = 0;
#endif
#if REMOTE_MODE == TRAIN
            remote_control->on_aim = 1;
            remote_control->aim_mode = REMOTE_ARMOR;
            Remote_Update();
            Remote_ShootModeSet();
            remote_control->autoshoot_flag = 1;
            Servo_SetAngle(&Servo_MagServo, Servo_Open);
            remote_control->mag_state = 1;
#endif
            break;
        }
        default:
            break;
        }
    }
    Following_AutoaimModeSet();
    remote_control->pending = 0;
    // 检录磁力计校准时使用
    // #if REMOTE_MODE == TRAIN
    //    if (remote->remote.s[SWITCH_RIGHT] == REMOTE_SWITCH_DOWN)
    //        KeyMouse_ChassisModeSet(CHASSIS_STOP);
    //    boardcom->check_in = (remote->remote.s[SWITCH_RIGHT] == REMOTE_SWITCH_DOWN ? 1 : 0);
    // #endif
}

static void Remote_Update()
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();

    boardcom->chassis_fb_ref = (float)remote->remote.ch[RIGHT_Y] * 440 / 660;
    boardcom->chassis_lr_ref = (float)remote->remote.ch[RIGHT_X] * 440 / 660;
    if (remote->remote.ch[PADDLE_WHEEL] <= -500.0f)
    {
        remote_control->gyro_flag = 1;
        boardcom->gyro_dir = 0;
    }
    else if (remote->remote.ch[PADDLE_WHEEL] >= 500.0f)
    {
        remote_control->gyro_flag = 1;
        boardcom->gyro_dir = 1;
    }
    else
    {
        remote_control->gyro_flag = 0;
    }

    if ((remote->remote.ch[RIGHT_X] > 650) ||
        (remote->remote.ch[RIGHT_X] < -650) ||
        (remote->remote.ch[RIGHT_Y] > 650) ||
        (remote->remote.ch[RIGHT_Y] < -650))
    {
        boardcom->power_limit_mode = 0;
    }
    else
    {
        boardcom->power_limit_mode = 1;
    }

    float yaw_ref = (float)remote->remote.ch[LEFT_X] * remote_control->remote_yaw_to_ref;
    float pitch_ref = (float)remote->remote.ch[LEFT_Y] * remote_control->remote_pitch_to_ref;
    float temp_pitch_ref = gimbal->pitch_position_ref + pitch_ref;
    LimitMaxMin(yaw_ref, 0.5f, -0.5f);
    LimitMaxMin(temp_pitch_ref, gimbal->elevation_angle, gimbal->depression_angle);
    gimbal->yaw_position_ref = gimbal->yaw_position_ref - yaw_ref;
    gimbal->pitch_position_ref = temp_pitch_ref;
}

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
        remote_control->quiet_flag = 1;
        remote_control->flyslope_flag = 0;
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
                remote_control->mag_state = 0;
                wait4release = 1;
            }
            if (KEY_DN(r))
            {
                remote_control->autoshoot_flag = 0;
                wait4release = 1;
            }
            if (KEY_DN(c))
            {
                remote_control->cap_speedup_flag = 0;
            }
            if (KEY_DN(z))
            {
                Shoot_FeederModeSet(FEEDER_INITING);
            }
        }
    }
    else if (KEY(shift))
    {
        remote_control->flyslope_flag = 1;
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
        remote_control->quiet_flag = 0;
        remote_control->flyslope_flag = 0;
        if (wait4release == 0)
        {
            if (KEY_DN(f))
            {
                shooter->shooter_mode = SHOOTER_REFEREE;
            }
            if (KEY_UP(e))
            {
                KeyMouse_GyroModeSet();
            }
            if (KEY_UP(b))
            {
                KeyMouse_BuffModeSet(REMOTE_BUFF_TEMP);
            }
            //			  if (KEY_UP(v))
            //            {
            //                Remote_AutoaimModeSet(REMOTE_SMALL_BUFF);
            //            }
            //            if (KEY_UP(b))
            //            {
            //                Remote_AutoaimModeSet(REMOTE_BIG_BUFF);
            //            }
            if (KEY_UP(x))
            {
                gimbal->yaw_position_ref = gimbal->yaw_position_ref - 180.0f;
            }
            if (KEY_DN(z))
            {
                boardcom->fly_flag = 1;
            }
            if (KEY_UP(z))
            {
                boardcom->fly_flag = 0;
            }
            if (KEY_DN(q))
            {
                Servo_SetAngle(&Servo_MagServo, Servo_Open);
                remote_control->mag_state = 1;
            }
            if (KEY_DN(r))
            {
                remote_control->autoshoot_flag = 1;
            }
            if (KEY_DN(c))
            {
                remote_control->cap_speedup_flag = 1;
            }
        }
    }

    if (wait4release == 0)
    {
        if (!KEY(f) && !KEY2(ctrl, shift) && !KEY(ctrl))
        {
            static float t_ws = 0.0f;
            static float t_ws_last = 0.0f;
            if (remote_control->flyslope_flag)
            {
                boardcom->power_limit_mode = POWER_UNLIMIT;
                boardcom->chassis_fb_ref = (KEY(w) - KEY(s)) * remote_control->keymouse_flyslope_speed;
                boardcom->chassis_lr_ref = (KEY(d) - KEY(a)) * remote_control->keymouse_flyslope_speed;
            }
            else if (!remote_control->flyslope_flag && remote_control->cap_speedup_flag)
            {
                boardcom->power_limit_mode = POWER_LIMIT;
                boardcom->cap_speedup_flag = CAP_SPEEDUP;
                boardcom->chassis_fb_ref = (KEY(w) - KEY(s)) * remote_control->keymouse_upper_speed;
                boardcom->chassis_lr_ref = (KEY(d) - KEY(a)) * remote_control->keymouse_upper_speed;
            }
            else if (!remote_control->flyslope_flag && !remote_control->cap_speedup_flag)
            {
                boardcom->power_limit_mode = POWER_LIMIT;
                boardcom->cap_speedup_flag = CAP_NORMAL;
                boardcom->chassis_fb_ref = (KEY(w) - KEY(s)) * remote_control->keymouse_normal_speed;
                boardcom->chassis_lr_ref = (KEY(d) - KEY(a)) * remote_control->keymouse_normal_speed;
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
        KeyMouse_ArmorModeSet(1);
        autoaim->is_change_target = 1;
    }
    else if (mouse_r_last && !remote->mouse.r)
    {
        KeyMouse_ArmorModeSet(0);
        autoaim->is_change_target = 0;
    }

    mouse_r_last = remote->mouse.r;
    if (gimbal->present_mode == GIMBAL_NO_AUTO || (autoaim->target_state != AUTOAIM_TARGET_FOLLOWING))
    {
        float yaw_ref = 0.0f, pitch_ref = 0.0f, temp_pitch_ref = 0.0f;
        if (remote_control->quiet_flag)
        {
            yaw_ref = (float)remote->mouse.x * remote_control->keymouse_yaw_to_ref_quiet;
            pitch_ref = -(float)remote->mouse.y * remote_control->keymouse_pitch_to_ref_quiet;
        }
        else
        {
            yaw_ref = (float)remote->mouse.x * remote_control->keymouse_yaw_to_ref;
            pitch_ref = -(float)remote->mouse.y * remote_control->keymouse_pitch_to_ref;
        }
        temp_pitch_ref = gimbal->pitch_position_ref + pitch_ref;
        LimitMaxMin(yaw_ref, 0.5f, -0.5f);
        LimitMaxMin(temp_pitch_ref, gimbal->elevation_angle, gimbal->depression_angle);
        gimbal->yaw_position_ref = gimbal->yaw_position_ref - yaw_ref;
        gimbal->pitch_position_ref = temp_pitch_ref;
    }
}

static void KeyMouse_ArmorModeSet(uint8_t mode)
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

static void KeyMouse_BuffModeSet(uint8_t mode)
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if (remote_control->aim_mode == REMOTE_ARMOR)
    {
        remote_control->aim_mode = mode;
        if (boardcom->game_progress == 4 && boardcom->stage_remain_time <= 210)
        {
            remote_control->aim_mode = REMOTE_BIG_BUFF;
        }
        else if (boardcom->game_progress == 4 && boardcom->stage_remain_time > 210)
        {
            remote_control->aim_mode = REMOTE_SMALL_BUFF;
        }
    }
    else if ((remote_control->aim_mode == REMOTE_BUFF_TEMP) ||
             (remote_control->aim_mode == REMOTE_SMALL_BUFF) ||
             (remote_control->aim_mode == REMOTE_BIG_BUFF))
    {
        remote_control->aim_mode = REMOTE_ARMOR;
    }
}

static void Keymouse_VTM_Update(void)
{
    ext_remote_control_t *remote = &(Referee_GetDataPtr()->remote_vtm_data);
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Keyboard_VTM_DataTypeDef *keyboard_VTM = (Keyboard_VTM_DataTypeDef *)&(Referee_GetDataPtr()->remote_vtm_data.keyboard_value);
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();

    static uint8_t wait4release = 0;
    if (KEY_VTM(ctrl))
    {
        remote_control->quiet_flag = 1;
        remote_control->flyslope_flag = 0;
        if (wait4release <= 1)
        {
            if (KEY_DN_VTM(f))
            {
                shooter->shooter_mode = SHOOTER_STOP;
                wait4release = 1;
            }
            if (KEY_DN_VTM(q))
            {
                Servo_SetAngle(&Servo_MagServo, Servo_Close);
                remote_control->mag_state = 0;
                wait4release = 1;
            }
            if (KEY_DN_VTM(r))
            {
                remote_control->autoshoot_flag = 0;
                wait4release = 1;
            }
            if (KEY_DN_VTM(c))
            {
                remote_control->cap_speedup_flag = 0;
            }
            if (KEY_DN_VTM(z))
            {
                Shoot_FeederModeSet(FEEDER_INITING);
            }
        }
    }
    else if (KEY_VTM(shift))
    {
        remote_control->flyslope_flag = 1;
        if (wait4release <= 1)
        {
            if (KEY_DN_VTM(c))
            {
                boardcom->ui_cmd = !boardcom->ui_cmd;
                wait4release = 1;
            }
        }
    }
    else
    {
        remote_control->quiet_flag = 0;
        remote_control->flyslope_flag = 0;
        if (wait4release == 0)
        {
            if (KEY_DN_VTM(f))
            {
                shooter->shooter_mode = SHOOTER_REFEREE;
            }
            if (KEY_UP_VTM(e))
            {
                KeyMouse_GyroModeSet();
            }
            if (KEY_UP_VTM(b))
            {
                KeyMouse_BuffModeSet(REMOTE_BUFF_TEMP);
            }
            //			  if (KEY_UP_VTM(v))
            //            {
            //                Remote_AutoaimModeSet(REMOTE_SMALL_BUFF);
            //            }
            //            if (KEY_UP_VTM(b))
            //            {
            //                Remote_AutoaimModeSet(REMOTE_BIG_BUFF);
            //            }
            if (KEY_UP_VTM(x))
            {
                gimbal->yaw_position_ref = gimbal->yaw_position_ref - 180.0f;
            }
            if (KEY_DN_VTM(z))
            {
                boardcom->fly_flag = 1;
            }
            if (KEY_UP_VTM(z))
            {
                boardcom->fly_flag = 0;
            }
            if (KEY_DN_VTM(q))
            {
                Servo_SetAngle(&Servo_MagServo, Servo_Open);
                remote_control->mag_state = 1;
            }
            if (KEY_DN_VTM(r))
            {
                remote_control->autoshoot_flag = 1;
            }
            if (KEY_DN_VTM(c))
            {
                remote_control->cap_speedup_flag = 1;
            }
        }
    }

    if (wait4release == 0)
    {
        if (!KEY_VTM(f) && !KEY2_VTM(ctrl, shift) && !KEY_VTM(ctrl))
        {
            static float t_ws = 0.0f;
            static float t_ws_last = 0.0f;
            if (remote_control->flyslope_flag)
            {
                boardcom->power_limit_mode = POWER_UNLIMIT;
                boardcom->chassis_fb_ref = (KEY_VTM(w) - KEY_VTM(s)) * remote_control->keymouse_flyslope_speed;
                boardcom->chassis_lr_ref = (KEY_VTM(d) - KEY_VTM(a)) * remote_control->keymouse_flyslope_speed;
            }
            else if (!remote_control->flyslope_flag && remote_control->cap_speedup_flag)
            {
                boardcom->power_limit_mode = POWER_LIMIT;
                boardcom->cap_speedup_flag = CAP_SPEEDUP;
                boardcom->chassis_fb_ref = (KEY_VTM(w) - KEY_VTM(s)) * remote_control->keymouse_upper_speed;
                boardcom->chassis_lr_ref = (KEY_VTM(d) - KEY_VTM(a)) * remote_control->keymouse_upper_speed;
            }
            else if (!remote_control->flyslope_flag && !remote_control->cap_speedup_flag)
            {
                boardcom->power_limit_mode = POWER_LIMIT;
                boardcom->cap_speedup_flag = CAP_NORMAL;
                boardcom->chassis_fb_ref = (KEY_VTM(w) - KEY_VTM(s)) * remote_control->keymouse_normal_speed;
                boardcom->chassis_lr_ref = (KEY_VTM(d) - KEY_VTM(a)) * remote_control->keymouse_normal_speed;
            }
        }
    }
    else
    {
        if (memcmp(keyboard_VTM, &Remote_KeyZero_VTM, sizeof(Remote_KeyZero_VTM)) == 0)
        {
            wait4release = 0;
        }
    }
    memcpy(&Remote_Keylast_VTM, keyboard_VTM, sizeof(Remote_Keylast_VTM));

    /*********** mouse control********/
    static uint8_t mouse_r_last = 0;
    if (!mouse_r_last && remote->right_button_down)
    {
        KeyMouse_ArmorModeSet(1);
        autoaim->is_change_target = 1;
    }
    else if (mouse_r_last && !remote->right_button_down)
    {
        KeyMouse_ArmorModeSet(0);
        autoaim->is_change_target = 0;
    }

    mouse_r_last = remote->right_button_down;
    if (gimbal->present_mode == GIMBAL_NO_AUTO || (autoaim->target_state != AUTOAIM_TARGET_FOLLOWING))
    {
        float yaw_ref = 0.0f, pitch_ref = 0.0f, temp_pitch_ref = 0.0f;
        if (remote_control->quiet_flag)
        {
            yaw_ref = (float)remote->mouse_x * remote_control->keymouse_yaw_to_ref_quiet;
            pitch_ref = -(float)remote->mouse_y * remote_control->keymouse_pitch_to_ref_quiet;
        }
        else
        {
            yaw_ref = (float)remote->mouse_x * remote_control->keymouse_yaw_to_ref;
            pitch_ref = -(float)remote->mouse_y * remote_control->keymouse_pitch_to_ref;
        }
        temp_pitch_ref = gimbal->pitch_position_ref + pitch_ref;
        LimitMaxMin(yaw_ref, 0.5f, -0.5f);
        LimitMaxMin(temp_pitch_ref, gimbal->elevation_angle, gimbal->depression_angle);
        gimbal->yaw_position_ref = gimbal->yaw_position_ref - yaw_ref;
        gimbal->pitch_position_ref = temp_pitch_ref;
    }
}
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
        KeyMouse_ChassisModeSet(CHASSIS_NORMAL);
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
        KeyMouse_ChassisModeSet(CHASSIS_NORMAL);
        break;
    case REMOTE_SMALL_BUFF:
        AutoAim_ModeSet(AUTOAIM_SMALL_BUFF);
        autoaim->hit_mode = AUTOAIM_HIT_BUFF;
        Gimbal_ModeSet(GIMBAL_SMALL_ENERGY);
        KeyMouse_ChassisModeSet(CHASSIS_STOP);
        break;
    case REMOTE_BIG_BUFF:
        AutoAim_ModeSet(AUTOAIM_BIG_BUFF);
        autoaim->hit_mode = AUTOAIM_HIT_BUFF;
        Gimbal_ModeSet(GIMBAL_BIG_ENERGY);
        KeyMouse_ChassisModeSet(CHASSIS_STOP);
        break;
    default:
        break;
    }
}

static void KeyMouse_GyroModeSet(void)
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if (boardcom->chassis_mode == CHASSIS_STOP)
    {
        return;
    }
    if (remote_control->gyro_flag == 0)
    {
        KeyMouse_ChassisModeSet(CHASSIS_GYRO);
        boardcom->gyro_dir = !boardcom->gyro_dir;
        remote_control->gyro_flag = 1;
    }
    else
    {
        KeyMouse_ChassisModeSet(CHASSIS_NORMAL);
        remote_control->gyro_flag = 0;
    }
}

static void KeyMouse_ChassisModeSet(uint8_t chassis_mode)
{
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if (chassis_mode == CHASSIS_NORMAL && remote_control->gyro_flag)
    {
        boardcom->chassis_mode = CHASSIS_GYRO;
    }
    else
    {
        boardcom->chassis_mode = chassis_mode;
    }
}
