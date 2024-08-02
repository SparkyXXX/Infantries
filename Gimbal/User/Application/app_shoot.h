/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-08-02 19:06:59
 */

#ifndef APP_SHOOT_H
#define APP_SHOOT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "alg_pid.h"

#define LOCKED_CURRENT 8000.0f
#define LOCKED_SPEED 20.0f
#define LOCKED_TIME 20.0f
#define RELOCKED_TIME 100.0f
#define REVERSE_SPEED -30.0f

#define COOLING_FIRST 1
#define OUTBURST_FIRST 2

#define HEAT_LIMIT 15

#define CONTINUOUS 1
#define SINGLE 2

    typedef enum
    {
        FEEDER_STOP = 0u,
        FEEDER_SINGLE = 1u,
        FEEDER_LOCKED = 2u,
        FEEDER_REFEREE = 3u,
        FEEDER_FINISH = 4u,
        FEEDER_INITING = 5u
    } Feeder_ModeEnum;

    typedef enum
    {
        SHOOTER_STOP = 0u,
        SHOOTER_REFEREE = 1u
    } Shooter_ModeEnum;

    typedef struct
    {
        float left_speed_ref;
        float right_speed_ref;
        float left_speed_fdb;
        float right_speed_fdb;
        float referee_bullet_speed[5];
        float average_bullet_speed;
    } Shooter_SpeedDataTypeDef;

    typedef struct
    {
        Shooter_ModeEnum shooter_mode;
        Feeder_ModeEnum feeder_mode, last_feeder_mode;
        Shooter_SpeedDataTypeDef shoot_speed;

        float feeder_angle_init;
        float shoot_freq_ref;
        float shoot_freq_fdb;
        float fast_shoot_freq;
        float slow_shoot_freq;

        float heat_now;
        float heat_now_referee;
        float heat_limit;

        uint8_t shoot_mode;
        uint8_t single_shoot_done;
        float armor_wait_ms;

        PID_TypeDef shoot_left, shoot_right;
        PID_TypeDef feed_spd, feed_ang;
    } Shoot_ControlTypeDef;

    extern float angle_diff, last_consequent_angle;

    Shoot_ControlTypeDef *Shoot_GetControlPtr(void);
    void Shoot_Init(void);
    void ShootSpeed_Update(void);
    void Heat_Control(void);
    void Shoot_ShooterControl(void);
    void Shoot_FeederControl(void);
    void Shoot_ShooterOutput(void);
    void Shoot_FeederOutput(void);

    void Shoot_Single(void);
    void Shoot_FeederLockedJudge(void);
    static void Shoot_FeederLockedHandle();
    void Shoot_FeederModeSet(Feeder_ModeEnum mode);
    void Shoot_FeederModeForceSet(Feeder_ModeEnum mode);

    void Remote_ShootModeSet(void);
    void Keymouse_ShootModeSet(void);
    void AutoAim_ShootModeSet(void);
#endif

#ifdef __cplusplus
}
#endif
