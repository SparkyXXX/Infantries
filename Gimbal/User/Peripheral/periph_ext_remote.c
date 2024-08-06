/*
 * @Author: Chen Zhihong
 * @LastEditors: Chen Zhihong
 */
/**
 * DreamChaser Frame Source File
 *
 * File:        ext_remote_dev.c
 * Brief:       This document contains the data receiving and sending of the referee system in referee_dev.c
 * Author:      Qu Jiuhe
 * Modified:    2023/5/7      11:37
 *
 */
#include "periph_ext_remote.h"
#include "lib_crc.h"

Referee_DataTypeDef Referee_Data;

const uint16_t REFEREE_FRAME_HEADER_SOF = 0xA5; // 裁判系统指令帧头长度
uint8_t Referee_TxData[REFEREE_TX_BUFF_LEN];
uint8_t Referee_RxData[REFEREE_RX_BUFF_LEN];

Referee_DataTypeDef *Referee_GetDataPtr()
{
    return &Referee_Data; // 获取裁判系统数据
}

void Referee_Init(UART_HandleTypeDef *huart)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    Referee_Reset();
    referee->last_update_time = HAL_GetTick();
    referee->huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(referee->huart, Referee_RxData, REFEREE_RX_BUFF_LEN);
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
/**
 * @brief      解码函数
 * @param
 * @retval
 */
uint8_t P_ext_remote_control(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_remote_control_t *struct_ptr = data_ptr;
    referee->remote_vtm_data.mouse_x = struct_ptr->mouse_x;
    referee->remote_vtm_data.mouse_y = struct_ptr->mouse_y;
    referee->remote_vtm_data.mouse_z = struct_ptr->mouse_z;
    referee->remote_vtm_data.left_button_down = struct_ptr->left_button_down;
    referee->remote_vtm_data.right_button_down = struct_ptr->right_button_down;
    referee->remote_vtm_data.keyboard_value = struct_ptr->keyboard_value;
    return PARSE_SUCCEEDED;
}
#define Const_Referee_CMD_NUM 2 // 裁判系统指令个数（不含交互指令）
const Referee_RefereeCmdTypeDef Const_Referee_CMD_LIST[Const_Referee_CMD_NUM] = {
    {0x0304, 12, &P_ext_remote_control}};

uint8_t Referee_ParseRobotCustomData(uint8_t *data, uint16_t data_length)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    ext_student_interactive_header_data_t *header_struct_ptr = (void *)data;
    if (header_struct_ptr->data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || header_struct_ptr->data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND)
    {
        return PARSE_FAILED;
    }
    //    if (header_struct_ptr->receiver_ID != referee->robot_id)
    //    {
    //        return PARSE_FAILED;
    //    }
    return PARSE_SUCCEEDED;
}

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
