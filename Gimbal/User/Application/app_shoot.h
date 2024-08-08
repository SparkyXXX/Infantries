/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-20 02:10:14
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

#define LOCKED_CURRENT 9000.0f
#define LOCKED_SPEED 20.0f
#define LOCKED_TIME 500.0f
#define RELOCKED_TIME 100.0f
#define REVERSE_SPEED -30.0f

#define HEAT_SINGLE_COUNT 10
#define HEAT_FAST_LIMIT 120
#define HEAT_SLOW_LIMIT 20
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
        float shooter_17mm_cooling_heat;
        float shooter_17mm_cooling_rate;
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
        PID_TypeDef shoot_left, shoot_right;
        PID_TypeDef feed_spd, feed_ang;
				float feeder_angle_init;
				float feeder_angle_beginInit;
    } Shoot_ControlTypeDef;


    void Shoot_Init(void);
    void Shoot_ShooterControl(void);
    void Shoot_FeederControl(void);
    static void Shoot_HeatControl();
    void Shoot_Update(void);
    void Shoot_Output(void);

    void Shoot_Single(void);
		void FeederOneInc(void);
		void FeederOneRemainInc(void);
    void Shoot_FeederLockedJudge(void);
    static void Shoot_FeederLockedHandle();

    Shoot_ControlTypeDef *Shoot_GetControlPtr(void);
    void Shoot_FeederModeSet(Feeder_ModeEnum mode);
    void Shoot_FeederModeForceSet(Feeder_ModeEnum mode);
    void Shoot_SetFeederSpeed(float speed);

#endif

#ifdef __cplusplus
}
#endif
