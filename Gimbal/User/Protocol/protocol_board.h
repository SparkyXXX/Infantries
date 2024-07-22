/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 01:36:09
 */

#ifndef PROTOCOL_BOARDCOM_H
#define PROTOCOL_BOARDCOM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_autoaim.h"
#include "app_gimbal.h"
#include "app_ins.h"
#include "app_remote.h"
#include "app_shoot.h"
#include "callback_ctrl.h"
#include "lib_buff.h"
#include "stm32g4xx_hal.h"
#include "string.h"
#include "util_fdcan.h"

#define POWER_UNLIMIT 0x00
#define POWER_LIMIT 0x01

#define CAP_NORMAL 0x00
#define CAP_SPEEDUP 0x01

#define PITCH 0
#define ROLL 1
#define YAW 2

    typedef enum
    {
        BOARDCOM_NULL = 0,
        BOARDCOM_CONNECTED = 1,
        BOARDCOM_LOST = 2,
        BOARDCOM_ERROR = 3,
        BOARDCOM_PENDING = 4
    } BoardCom_StateEnum;

    typedef struct
    {
        BoardCom_StateEnum state;
        uint32_t last_update_time;

        // Gimbal Send to Chassis
        uint8_t yaw_mode;
        uint8_t chassis_mode;
        uint8_t power_limit_mode;
        uint8_t check_in;
        uint8_t is_get_target;
        uint8_t gyro_dir;
        uint8_t fly_flag;
        uint8_t ui_cmd;
        uint8_t cap_speedup_flag;
        float yaw_ref;
        float yaw_pos_fdb;
        float yaw_spd_fdb;
        float chassis_fb_ref;
        float chassis_lr_ref;
        float pitch_angle;
        uint8_t magazine_state;
        uint8_t shooter_state;
        uint8_t auto_shoot_state;
        float autoaim_yaw_spd_ref;

        // Gimbal Receive from Chassis
        uint8_t robot_id;
        uint8_t power_management_shooter_output;
        uint16_t heat_limit;
        float shoot_spd_referee;
        uint16_t cooling_per_second;
		uint16_t shooter_heat_referee;
        uint16_t stage_remain_time;
        uint8_t game_progress;
    } BoardCom_DataTypeDef;

    typedef struct
    {
        void (*bus_func)(uint8_t buff[]);
    } Board_SendTableEntryTypeDef;

    typedef struct
    {
        uint32_t cmd_id;
        void (*bus_func)(uint8_t buff[]);
    } Board_ReceiveTableEntryTypeDef;

    extern FDCAN_HandleTypeDef *BOARD_CAN_HANDLER;
    extern FDCAN_HandleTypeDef *CAP_CAN_HANDLER;

    BoardCom_DataTypeDef *BoardCom_GetDataPtr(void);
    void BoardCom_Init(void);
    void BoardCom_Update(void);
    void BoardComPkg1_Send(void);
    void BoardComPkg2_Send(void);
    void BoardCom_Decode(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen);
    uint8_t BoardCom_IsLost(void);

    static void _send_control(uint8_t buff[]);
    static void _send_imu_yaw(uint8_t buff[]);
    static void _send_chassis_ref(uint8_t buff[]);
    static void _send_ui_state(uint8_t buff[]);
    static void _receive_referee_data1(uint8_t buff[]);
    static void _receive_referee_data2(uint8_t buff[]);

#endif

#ifdef __cplusplus
}
#endif
