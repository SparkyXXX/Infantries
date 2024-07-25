/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-25 11:09:19
 */

#ifndef PERIPH_REFEREE_H
#define PERIPH_REFEREE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "callback_ctrl.h"
#include "stdint.h"
#include "string.h"
#include "util_uart.h"

#define REFEREE_TX_BUFF_LEN 300
#define REFEREE_RX_BUFF_LEN 500
#define REFEREE_OFFLINE_TIME 1000

    /********** START OF REFEREE CMD STRUCT DEFINITION **********/
    // 0x0001
    typedef struct __attribute__((packed))
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    } ext_game_status_t;

    // 0x0002
    typedef struct __attribute__((packed))
    {
        uint8_t winner;
    } ext_game_result_t;

    // 0x0003
    typedef struct __attribute__((packed))
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    } ext_game_robot_HP_t;

    // 0x0101
    typedef struct __attribute__((packed))
    {
        uint32_t event_data;
    } ext_event_data_t;

    // 0x0102
    typedef struct __attribute__((packed))
    {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    } ext_supply_projectile_action_t;

    // 0x0104
    typedef struct __attribute__((packed))
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    } ext_referee_warning_t;

    // 0x0105
    typedef struct __attribute__((packed))
    {
        uint8_t dart_remaining_time;
        uint16_t dart_info;
    } ext_dart_info_t;

    // 0x0201
    typedef struct __attribute__((packed))
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t maximum_HP;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
    } ext_robot_status_t;

    // 0x0202
    typedef struct __attribute__((packed))
    {
        uint16_t chassis_voltage;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t buffer_energy;
        uint16_t shooter_17mm_1_barrel_heat;
        uint16_t shooter_17mm_2_barrel_heat;
        uint16_t shooter_42mm_barrel_heat;
    } ext_power_heat_data_t;

    // 0x0203
    typedef struct __attribute__((packed))
    {
        float x;
        float y;
        float angle;
    } ext_robot_pos_t;

    // 0x0204
    typedef struct __attribute__((packed))
    {
        uint8_t recovery_buff;
        uint8_t cooling_buff;
        uint8_t defence_buff;
        uint8_t vulnerability_buff;
        uint16_t attack_buff;
    } ext_buff_t;

    // 0x0205
    typedef struct __attribute__((packed))
    {
        uint8_t airforce_status;
        uint8_t time_remain;
    } ext_air_support_data_t;

    // 0x0206
    typedef struct __attribute__((packed))
    {
        uint8_t armor_id : 4;
        uint8_t HP_deduction_reason : 4;
    } ext_hurt_data_t;

    // 0x0207
    typedef struct __attribute__((packed))
    {
        uint8_t bullet_type;
        uint8_t shooter_number;
        uint8_t launching_frequency;
        float initial_speed;
    } ext_shoot_data_t;

    // 0x0208
    typedef struct __attribute__((packed))
    {
        uint16_t projectile_allowance_17mm;
        uint16_t projectile_allowance_42mm;
        uint16_t remaining_gold_coin;
    } ext_projectile_allowance_t;

    // 0x0209
    typedef struct __attribute__((packed))
    {
        uint32_t rfid_status;
    } ext_rfid_status_t;

    // 0x020A
    typedef struct __attribute__((packed))
    {
        uint8_t dart_launch_opening_status;
        uint8_t reserved;
        uint16_t target_change_time;
        uint16_t latest_launch_cmd_time;
    } ext_dart_client_cmd_t;

    typedef struct __attribute__((packed))
    {
        uint16_t data_cmd_id;
        uint16_t sender_ID;
        uint16_t receiver_ID;
    } ext_student_interactive_header_data_t;

    typedef struct __attribute__((packed))
    {
        uint8_t graphic_name[3];
        uint32_t operate_type : 3;
        uint32_t graphic_type : 3;
        uint32_t layer : 4;
        uint32_t color : 4;
        uint32_t start_angle : 9;
        uint32_t end_angle : 9;
        uint32_t width : 10;
        uint32_t start_x : 11;
        uint32_t start_y : 11;
        uint32_t radius : 10;
        uint32_t end_x : 11;
        uint32_t end_y : 11;
    } graphic_data_struct_t;

    typedef struct __attribute__((packed))
    {
        graphic_data_struct_t grapic_data_struct;
        uint8_t data[30];
    } ext_client_custom_character_t;

    /********** END OF REFEREE CMD STRUCT DEFINITION **********/

    typedef enum
    {
        REFEREE_NULL = 0,
        REFEREE_CONNECTED = 1,
        REFEREE_LOST = 2,
        REFEREE_ERROR = 3,
        REFEREE_PENDING = 4
    } Referee_RefereeStateEnum;

    typedef struct
    {
        UART_HandleTypeDef *huart;
        Referee_RefereeStateEnum state; // 裁判系统当前状态
        uint32_t last_update_time;      // 裁判系统上次更新时间
        uint16_t client_id;             // 客户端ID
		
        uint8_t game_progress;      //  当前比赛阶段,0：未开始比赛；
                                    //              1：准备阶段；
                                    //              2：自检阶段；
                                    //              3：5s倒计时；
                                    //              4：对战中；
                                    //              5：比赛结算中
		
        uint16_t stage_remain_time; //  当前阶段剩余时间，单位s

        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;

        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t maximum_HP;

        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        uint16_t chassis_power_level_limit;

        uint16_t chassis_voltage;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t buffer_energy;
        uint16_t shooter_17mm_1_barrel_heat;
        uint16_t shooter_17mm_2_barrel_heat;

        float x;
        float y;
        float angle;

        uint8_t shooter_number;
        uint8_t launching_frequency;
        float initial_speed;

        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
    } Referee_DataTypeDef;

    typedef uint8_t (*Referee_RefereeCmdParseFuncDef)(Referee_DataTypeDef *referee, void *data_ptr);

    typedef struct
    {
        uint16_t cmd_id;
        uint8_t data_length;
        Referee_RefereeCmdParseFuncDef parse_func;
    } Referee_RefereeCmdTypeDef;

    typedef struct
    {
        uint16_t robot_id;
        uint16_t client_id;
    } Referee_RobotAndClientIDTypeDef;

    Referee_DataTypeDef *Referee_GetDataPtr(void);
    void Referee_Init(UART_HandleTypeDef *huart);
    void Referee_Reset(void);
    void Referee_Decode(uint8_t *buff, uint16_t rxdatalen);
    uint8_t Referee_IsLost(void);
    uint16_t Referee_GetClientIDByRobotID(uint8_t robot_id);
    uint8_t Referee_ParseRefereeCmd(uint16_t cmd_id, uint8_t *data, uint16_t data_length);

#ifdef __cplusplus
}
#endif

#endif
