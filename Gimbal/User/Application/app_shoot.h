/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-05-25 11:53:01
 */

#ifndef APP_SHOOT_H
#define APP_SHOOT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "protocol_motor.h"
#include "protocol_board.h"
#include "periph_motor_pwm.h"
#include "alg_pid.h"
#include "util_adc.h"
#include "cmsis_os.h"

#define LOCKED_CURRENT 8000.0f
#define LOCKED_SPEED 20.0f
#define LOCKED_TIME 200.0f
#define RELOCKED_TIME 100.0f
#define REVERSE_SPEED -30.0f

#define HEAT_SINGLE_COUNT 10
#define HEAT_FAST_LIMIT 120
#define HEAT_SLOW_LIMIT 80
#define HEAT_WAIT_LIMIT 10
#define HEAT_STOP_LIMIT 0

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
        float left_shoot_speed;
        float right_shoot_speed;
        float feeder_shoot_speed;
    } Shooter_SpeedDataTypeDef;

    typedef struct
    {
        float shooter_heat_now;
        float shooter_heat_limit;
        float feeder_speed;
        uint8_t current_pidnum;
    } Shooter_HeatDataTypeDef;

    typedef struct
    {
        Shooter_ModeEnum shooter_mode;
        Feeder_ModeEnum feeder_mode, last_feeder_mode;
        Shooter_SpeedDataTypeDef shoot_speed;
        Shooter_HeatDataTypeDef heat_ctrl;

        uint8_t single_shoot_done;
        float feeder_angle_init;

        PID_TypeDef shoot_left, shoot_right;
        PID_TypeDef feed_spd, feed_ang;
        Filter_Lowpass_TypeDef shooter_left_lpf;
        Filter_Lowpass_TypeDef shooter_right_lpf;
    } Shoot_ControlTypeDef;

	extern float angle_diff, last_consequent_angle;

    Shoot_ControlTypeDef *Shoot_GetControlPtr(void);
    void Shoot_Init(void);
    void Shoot_Update(void);
	void Heat_Update(void);
    void Shoot_ShooterControl(void);
    void Shoot_FeederControl(void);
    static void Shoot_HeatControl(void);
    void Shoot_Output(void);

    void Shoot_Single(void);
    void Shoot_FeederLockedJudge(void);
    static void Shoot_FeederLockedHandle();
    void Shoot_FeederModeSet(Feeder_ModeEnum mode);
    void Shoot_FeederModeForceSet(Feeder_ModeEnum mode);
    void Shoot_SetFeederSpeed(float speed);
#endif

#ifdef __cplusplus
}
#endif
