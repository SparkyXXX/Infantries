/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 01:53:19
 */

#ifndef PROTOCOL_BOARDCOM_H
#define PROTOCOL_BOARDCOM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32g4xx_hal.h"
#include "string.h"

#define CHASSIS_CTRL_STOP 0x00
#define CHASSIS_CTRL_NORMAL 0x01
#define CHASSIS_CTRL_GYRO 0x02

#define GIMBAL_YAW_NO_AUTO 0x00
#define GIMBAL_YAW_ARMOR 0x01
#define GIMBAL_YAW_BIG_ENERGY 0x02
#define GIMBAL_YAW_SMALL_ENERGY 0x03

#define POWER_UNLIMIT 0x00
#define POWER_LIMIT 0x01

#define CAP_NORMAL 0x00
#define CAP_SPEEDUP 0x01

#define BOARDCOM_TIMEOUT_VALUE 20 // 单位并非时间，而是控制电机次数（底盘+云台）

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
        uint32_t last_update_time[8];

        // Chassis Send to Gimbal
        uint8_t robot_id;
        uint16_t heat_limit;
        float shoot_spd_referee;
        uint16_t cooling_per_second;

        // Chassis Send to Cap
        uint8_t cap_mode_flag;
        uint8_t power_level_limit;
        uint8_t power_limit;
        uint8_t power_buffer;
        float chassis_power;
        uint16_t stage_remain_time;
        uint8_t game_progress;

        // Chassis Receive from Gimbal
        uint8_t yaw_mode;
        uint8_t chassis_mode;
        uint8_t power_limit_mode;
        uint8_t ui_cmd;
        uint8_t fly_flag;
        uint8_t gyro_dir;
        uint8_t is_get_target;
        uint8_t check_in;
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
        uint8_t team_color;
        float autoaim_yaw_spd_ref;

        // Chassis Receive from Cap
        float cap_power;
        uint8_t cap_rest_energy;
        uint8_t power_management_shooter_output : 1;
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

    extern uint8_t boardcom_decoded_count;

    BoardCom_DataTypeDef *BoardCom_GetDataPtr(void);
    void BoardCom_Init(void);
    void BoardCom_Update(void);
    void BoardCom_Send(void);
    void BoardCom_Decode(FDCAN_HandleTypeDef *pfdhcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
    uint8_t BoardCom_IsLost(uint8_t);

    static void _send_referee_data1(uint8_t buff[]);
    static void _send_referee_data2(uint8_t buff[]);
    static void _send_cap_data(uint8_t buff[]);
    static void _receive_control(uint8_t buff[]);
    static void _receive_imu_yaw(uint8_t buff[]);
    static void _receive_chassis_ref(uint8_t buff[]);
    static void _receive_ui_state(uint8_t buff[]);
    static void _receive_cap_data(uint8_t buff[]);

    extern uint8_t boardcom_decoded_count;
#ifdef __cplusplus
}
#endif

#endif
