/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-20 00:01:23
 */

#ifndef PROTOCOL_BOARDCOM_H
#define PROTOCOL_BOARDCOM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "string.h"
#include "stm32g4xx_hal.h"

#define BOARDCOM_TX_LEN 64
#define BOARDCOM_RX_LEN 64
#define BOARDCOM_LOST_TIME 200

#define BOARDCOM_CHASSIS_BUFF_SIZE 3
#define BOARDCOM_RECEIVE_SIZE 7
#define CAPCOM_RECEIVE_SIZE 3

#define CHASSIS_CTRL_STOP 0x00
#define CHASSIS_CTRL_NORMAL 0x01
#define CHASSIS_CTRL_GYRO 0x02

#define GIMBAL_YAW_NO_AUTO 0x00
#define GIMBAL_YAW_ARMOR 0x01
#define GIMBAL_YAW_BIG_ENERGY 0x02
#define GIMBAL_YAW_SMALL_ENERGY 0x03

//      power limit mode
#define POWER_UNLIMIT 0x00
#define POWER_LIMIT 0x01

//      cap speedup flag
#define CAP_NORMAL 0x00
#define CAP_SPEEDUP 0x01

//      cap mode
#define SUPERCAP_CTRL_OFF 0x00
#define SUPERCAP_CTRL_ON 0x01

#define SET_CONTROL 0x201
#define SET_IMU_YAW 0x203
#define SET_CHA_REF 0x204
#define SET_UI_STATE 0x205

#define SET_REFEREE_DATA_1 0x206
#define SET_REFEREE_DATA_2 0x207

#define SET_CAP_MODE 0x98
#define SET_CAP_DATA_1 0x299
#define SET_CAP_DATA_2 0x298

#define SEND_CHASSIS_DATA_1 0xA1
#define SEND_CHASSIS_DATA_2 0xA2
#define SEND_CHASSIS_DATA_3 0xA3

#define SEND_PERIOD_CONTROL 2
#define SEND_PERIOD_IMU_YAW 2
#define SEND_PERIOD_CHAREF 5
#define SEND_PERIOD_UI_STATE 10
#define SEND_PERIOD_REFEREE1 5
#define SEND_PERIOD_REFEREE2 20
#define SEND_PERIOD_CAP_MODE 5

    typedef enum
    {
        BOARDCOM_NULL = 0,
        BOARDCOM_CONNECTED = 1,
        BOARDCOM_LOST = 2,
        BOARDCOM_ERROR = 3,
        BOARDCOM_PENDING = 4
    } BoardCom_StateEnum;

    typedef enum
    {
        BOARDCOM_PKG_REFEREE1 = 0,
        BOARDCOM_PKG_REFEREE2,
        BOARDCOM_PKG_CTRL,
        BOARDCOM_PKG_IMU,
        BOARDCOM_PKG_CHA_REF,
        BOARDCOM_PKG_UI_STATE,
        BOARDCOM_PKG_CAP1,
        BOARDCOM_PKG_CAP2
    } BoardCom_PkgEnum;

    typedef struct
    {
        BoardCom_StateEnum state;
        uint32_t last_update_time[8];

        // Chassis up stream
        float yaw_consequent_angle;
        float yaw_limited_angle;
        uint8_t robot_id;
        uint8_t power_limit;
		uint8_t power_level_limit;
        uint16_t heat_now;
        uint16_t heat_limit;
        float shoot_spd_referee;
        uint8_t speed_17mm_limit;
        uint8_t cap_mode_flag;
        uint8_t boost_mode_flag;
        uint8_t power_buffer;
        float chassis_power;

        // Gimbal up stream
        uint8_t yaw_mode;
        float yaw_ref;
        float yaw_pos_fdb;
        float yaw_spd_fdb;
        uint8_t chassis_mode;
        float chassis_fb_ref;
        float chassis_lr_ref;
        uint8_t cap_mode_user;
        uint8_t cap_speedup_flag;
        uint8_t power_limit_mode;
        uint8_t ui_cmd;
        uint8_t gyro_dir;

        float pitch_angle;
        uint8_t magazine_state;
        uint8_t shooter_state;
        uint8_t auto_shoot_state;
        uint8_t minipc_aim_mode;
        //int8_t minipc_offset_horizental;
        //int8_t minipc_offset_vertical;
		uint8_t team_color;
        uint16_t ui_x;
        uint16_t ui_y;

        // Cap up stream
        uint32_t power_path_change_flag;
        uint8_t cap_state;
        uint8_t cap_rest_energy;

        float cap_power;
        float cap_voltage;
        float cap_current;
    } BoardCom_DataTypeDef;

    typedef struct
    {
        uint32_t cmd_id;
        void (*bus_func)(uint8_t buff[]);
    } Board_TableEntryTypeDef;

    BoardCom_DataTypeDef *BoardCom_GetDataPtr(void);
    void BoardCom_Init(void);
    void BoardCom_Reset(void);
    void BoardCom_Send(void);
    void BoardCom_Decode(FDCAN_HandleTypeDef *pfdhcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
    void BoardCom_DecodeBoard(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen);
    void BoardCom_DecodeCap(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen);
    void BoardCom_Update(void);
    void BoardCom_ChassisModeSet(void);
    uint8_t BoardCom_IsLost(uint8_t);

#ifdef __cplusplus
}
#endif

#endif
