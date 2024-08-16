/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-08-16 18:06:53
 */

#include "protocol_referee.h"
#include "lib_crc.h"

Referee_DataTypeDef Referee_Data;
const uint16_t REFEREE_FRAME_HEADER_SOF = 0xA5; // 裁判系统指令帧头长度
uint8_t Referee_TxData[REFEREE_TX_BUFF_LEN];
uint8_t Referee_RxData[REFEREE_RX_BUFF_LEN];

Referee_DataTypeDef *Referee_GetDataPtr()
{
    return &Referee_Data; // 获取裁判系统数据
}

void Referee_Reset()
{
    Referee_DataTypeDef *referee = &Referee_Data;
    Referee_RefereeStateEnum state = referee->state;       // backup state
    uint32_t last_update_time = referee->last_update_time; // backup time
    memset(referee, 0, sizeof(Referee_DataTypeDef));
    referee->state = state;
    referee->last_update_time = last_update_time;
}

void Referee_Init(UART_HandleTypeDef *huart)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    Referee_Reset();
    referee->last_update_time = HAL_GetTick();
    referee->huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(referee->huart, Referee_RxData, REFEREE_RX_BUFF_LEN);
}

uint8_t Referee_IsLost()
{
    Referee_DataTypeDef *referee = &Referee_Data;
    uint32_t now = HAL_GetTick();
    if ((now - referee->last_update_time) > REFEREE_OFFLINE_TIME)
    {
        referee->state = REFEREE_LOST;
    }
    return referee->state == REFEREE_LOST;
}

void Referee_Decode(uint8_t *buff, uint16_t rxdatalen)
{
    Referee_DataTypeDef *referee = &Referee_Data;

    referee->state = REFEREE_PENDING;
    referee->last_update_time = HAL_GetTick();

    if (buff[0] != REFEREE_FRAME_HEADER_SOF)
    {
        referee->state = REFEREE_ERROR;
        return;
    }

    if (!CRC_VerifyCRC8CheckSum(buff, 5))
    {
        referee->state = REFEREE_ERROR;
        return;
    }

    uint16_t data_length = (uint16_t)buff[2] << 8 | buff[1];
    uint8_t seq = buff[3];
    if (seq == 0)
    {
        // referee->state = REFEREE_ERROR;
        // return;
    }
    if (!CRC_VerifyCRC16CheckSum(buff, data_length + 9))
    {
        referee->state = REFEREE_ERROR;
        return;
    }

    uint16_t cmd_id = (uint16_t)buff[6] << 8 | buff[5];
    if (!Referee_ParseRefereeCmd(cmd_id, buff + 7, data_length))
    {
        referee->state = REFEREE_ERROR;
        return;
    }
    referee->state = REFEREE_CONNECTED;
}

const uint8_t PARSE_FAILED = 0, PARSE_SUCCEEDED = 1;
const uint8_t level_maxpower_table[10][2] = { // 等级-功率对照表，用于校验裁判系统maxpower是否正确
    {60, 45},
    {65, 50},
    {70, 55},
    {75, 60},
    {80, 65},
    {85, 70},
    {90, 75},
    {95, 80},
    {100, 90},
    {100, 100}}; // 两行分别对应功率优先和血量优先

const Referee_RobotAndClientIDTypeDef // 机器人ID及对应客户端ID，0表示无对应客户端
    HERO_RED = {1, 0x0101},           // 英雄(红)
    ENGINEER_RED = {2, 0x0102},       // 工程(红)
    INFANTRY3_RED = {3, 0x0103},      // 步兵3(红)
    INFANTRY4_RED = {4, 0x0104},      // 步兵4(红)
    INFANTRY5_RED = {5, 0x0105},      // 步兵5(红)
    AERIAL_RED = {6, 0x0106},         // 空中(红)
    SENTRY_RED = {7, 0},              // 哨兵(红)
    HERO_BLUE = {101, 0x0165},        // 英雄(蓝)
    ENGINEER_BLUE = {102, 0x0166},    // 工程(蓝)
    INFANTRY3_BLUE = {103, 0x0167},   // 步兵3(蓝)
    INFANTRY4_BLUE = {104, 0x0168},   // 步兵4(蓝)
    INFANTRY5_BLUE = {105, 0x0169},   // 步兵5(蓝)
    AERIAL_BLUE = {106, 0x016A},      // 空中(蓝)
    SENTRY_BLUE = {107, 0};           // 哨兵(蓝)

const Referee_RefereeCmdTypeDef Const_Referee_CMD_INTERACTIVE = {0x0301, 8, NULL}; // 机器人间交互数据，发送方触发发送
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND = 0x0200;         // 机器人间交互数据内容ID下界
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND = 0x02FF;         // 机器人间交互数据内容ID上界

/********** REFEREE CMD PARSER FUNCTION **********/
uint8_t P_ext_game_status(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_game_status_t *struct_ptr = data_ptr;
    referee->game_progress = struct_ptr->game_progress;
    referee->stage_remain_time = struct_ptr->stage_remain_time;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_result(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_game_result_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_HP(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_game_robot_HP_t *struct_ptr = data_ptr;
    referee->red_7_robot_HP = struct_ptr->red_7_robot_HP;
    referee->red_outpost_HP = struct_ptr->red_outpost_HP;
    referee->red_base_HP = struct_ptr->red_base_HP;
    referee->blue_7_robot_HP = struct_ptr->blue_7_robot_HP;
    referee->blue_outpost_HP = struct_ptr->blue_outpost_HP;
    referee->blue_base_HP = struct_ptr->blue_base_HP;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_event_data(Referee_DataTypeDef *referee, void *data_ptr)
{
    //    ext_event_data_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_supply_projectile_action(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_supply_projectile_action_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_referee_warning(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_referee_warning_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}
uint8_t P_ext_dart_info(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_dart_remaining_time_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_robot_status(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_robot_status_t *struct_ptr = data_ptr;

    referee->robot_level = struct_ptr->robot_level;
    referee->current_HP = struct_ptr->current_HP;
    referee->maximum_HP = struct_ptr->maximum_HP;
    referee->shooter_barrel_cooling_value = struct_ptr->shooter_barrel_cooling_value;
    referee->shooter_barrel_heat_limit = struct_ptr->shooter_barrel_heat_limit;
    referee->chassis_power_limit = struct_ptr->chassis_power_limit;
    referee->power_management_gimbal_output = struct_ptr->power_management_gimbal_output;
    referee->power_management_chassis_output = struct_ptr->power_management_chassis_output;
    referee->power_management_shooter_output = struct_ptr->power_management_shooter_output;
    for (int8_t i = 0; i < 2; i++)
    {
        if (struct_ptr->chassis_power_limit == level_maxpower_table[referee->robot_level - 1][i])
        {
            referee->chassis_power_level_limit = struct_ptr->chassis_power_limit;
        }
    }
    if (referee->robot_id != struct_ptr->robot_id)
    {
        referee->robot_id = struct_ptr->robot_id;
        referee->client_id = Referee_GetClientIDByRobotID(referee->robot_id);
    }

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_power_heat_data(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_power_heat_data_t *struct_ptr = data_ptr;
    referee->chassis_voltage = struct_ptr->chassis_voltage;
    referee->chassis_current = struct_ptr->chassis_current;
    referee->chassis_power = struct_ptr->chassis_power;
    referee->buffer_energy = struct_ptr->buffer_energy;
    referee->shooter_17mm_1_barrel_heat = struct_ptr->shooter_17mm_1_barrel_heat;
    referee->shooter_17mm_2_barrel_heat = struct_ptr->shooter_17mm_2_barrel_heat;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_robot_pos(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_robot_pos_t *struct_ptr = data_ptr;

    referee->x = struct_ptr->x;
    referee->y = struct_ptr->y;
    referee->angle = struct_ptr->angle;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_buff(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_buff_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_air_support_data(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_air_support_data_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_hurt_data(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_hurt_data_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_shoot_data(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_shoot_data_t *struct_ptr = data_ptr;

    referee->shooter_number = struct_ptr->shooter_number;
    referee->launching_frequency = struct_ptr->launching_frequency;
    referee->initial_speed = struct_ptr->initial_speed;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_projectile_allowance(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_projectile_allowance_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_rfid_status(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_rfid_status_t *struct_ptr = data_ptr;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_dart_client_cmd(Referee_DataTypeDef *referee, void *data_ptr)
{
    // ext_dart_client_cmd_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

/********** END OF REFEREE CMD PARSER FUNCTION **********/

#define Const_Referee_CMD_NUM 20 // 裁判系统指令个数（不含交互指令）
const Referee_RefereeCmdTypeDef Const_Referee_CMD_LIST[Const_Referee_CMD_NUM] = {
    // 裁判系统消息命令ID列表
    {0x0001, 11, &P_ext_game_status},             // 比赛状态数据，1Hz 周期发送
    {0x0002, 1, &P_ext_game_result},              // 比赛结果数据，比赛结束后发送
    {0x0003, 32, &P_ext_game_robot_HP},           // 比赛机器人血量数据，1Hz 周期发送
    {0x0101, 4, &P_ext_event_data},               // 场地事件数据，事件改变后发送
    {0x0102, 4, &P_ext_supply_projectile_action}, // 场地补给站动作标识数据，动作改变后发送
    {0x0104, 3, &P_ext_referee_warning},          // 裁判警告数据，警告发生后发送
    {0x0105, 3, &P_ext_dart_info},                // 飞镖发射口倒计时，1Hz周期发送
    {0x0201, 13, &P_ext_robot_status},            // 机器人状态数据，10Hz 周期发送
    {0x0202, 16, &P_ext_power_heat_data},         // 实时功率热量数据，50Hz 周期发送
    {0x0203, 16, &P_ext_robot_pos},               // 机器人位置数据，10Hz 发送
    {0x0204, 6, &P_ext_buff},                     // 机器人增益数据，1Hz 周期发送
    {0x0205, 2, &P_ext_air_support_data},         // 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
    {0x0206, 1, &P_ext_hurt_data},                // 伤害状态数据，伤害发生后发送
    {0x0207, 7, &P_ext_shoot_data},               // 实时射击数据，子弹发射后发送
    {0x0208, 6, &P_ext_projectile_allowance},     // 弹丸剩余发射数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz周期发送
    {0x0209, 4, &P_ext_rfid_status},              // 机器人RFID状态，3Hz周期发送
    {0x020A, 6, &P_ext_dart_client_cmd},          // 飞镖机器人客户端指令书，10Hz周期发送
};

uint8_t Referee_ParseRobotCustomData(uint8_t *data, uint16_t data_length)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    ext_student_interactive_header_data_t *header_struct_ptr = (void *)data;
    if (header_struct_ptr->data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || header_struct_ptr->data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND)
    {
        return PARSE_FAILED;
    }
    if (header_struct_ptr->receiver_ID != referee->robot_id)
    {
        return PARSE_FAILED;
    }
    return PARSE_SUCCEEDED;
}

/**
 * @brief      ͨ��������ID��ȡ��Ӧ�ͻ���ID
 * @param      robot_id: ������ID
 * @retval     �ͻ���ID
 */
uint16_t Referee_GetClientIDByRobotID(uint8_t robot_id)
{
    if (robot_id == 7 || robot_id == 107)
    {
        return 0;
    }
    if ((robot_id >= 1 && robot_id <= 6) || (robot_id >= 101 && robot_id <= 106))
    {
        return robot_id + 0x100;
    }
    return 0;
}

/**
 * @brief      裁判系统数据解析函数
 * @param      cmd_id: 命令ID
 * @param      data: 数据帧
 * @param      data_length: 数据帧长度
 * @retval     解析结果（0为失败，1为成功）
 */
uint8_t Referee_ParseRefereeCmd(uint16_t cmd_id, uint8_t *data, uint16_t data_length)
{
    Referee_DataTypeDef *referee = &Referee_Data;

    if (cmd_id == Const_Referee_CMD_INTERACTIVE.cmd_id)
    {
        return Referee_ParseRobotCustomData(data, data_length); // 机器人间通信
    }
    for (int i = 0; i < Const_Referee_CMD_NUM; ++i)
    {
        if (cmd_id == Const_Referee_CMD_LIST[i].cmd_id)
        {
            // if (data_length != Const_Referee_CMD_LIST[i].data_length) return PARSE_FAILED;  // wrong data length
            if (Const_Referee_CMD_LIST[i].parse_func == NULL)
            {
                return PARSE_FAILED; // unsupported cmd
            }
            return (*(Const_Referee_CMD_LIST[i].parse_func))(referee, data); // parse cmd
        }
    }
    return PARSE_FAILED; // unknown cmd
}
