/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-16 01:43:10
 */

#include "protocol_referee.h"
#include "periph_cap.h"
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
const uint16_t Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH = 113 - 1;           // 机器人间交互数据内容最大长度
const uint16_t Const_Referee_GRAPHIC_BUFFER_MAX_LENGTH = 21;                       // 图形缓冲区最大长度
const Referee_RefereeCmdTypeDef Const_Referee_DATA_CMD_ID_LIST[6] = {
    // 裁判系统交互数据内容ID
    {0x0100, 2, NULL},   // 客户端删除图形
    {0x0101, 15, NULL},  // 客户端绘制一个图形
    {0x0102, 30, NULL},  // 客户端绘制二个图形
    {0x0103, 75, NULL},  // 客户端绘制五个图形
    {0x0104, 105, NULL}, // 客户端绘制七个图形
    {0x0110, 45, NULL}   // 客户端绘制字符图形
};

/********** REFEREE CMD PARSER FUNCTION **********/
uint8_t P_ext_game_status(Referee_DataTypeDef *referee, void *data_ptr)
{
    ext_game_status_t *struct_ptr = data_ptr;

    referee->game_type = struct_ptr->game_type;
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
    ext_event_data_t *struct_ptr = data_ptr;

    referee->event_data = struct_ptr->event_data;

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
    Cap_DataTypeDef *cap = Cap_GetDataPtr();
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
    // referee->rfid_status = struct_ptr->rfid_status;
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

    // if (data_length != Const_Referee_CMD_INTERACTIVE.data_length) return PARSE_FAILED;      // wrong data length

    ext_student_interactive_header_data_t *header_struct_ptr = (void *)data;
    if (header_struct_ptr->data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || header_struct_ptr->data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND)
    {
        return PARSE_FAILED; // wrong data cmd id
    }
    if (header_struct_ptr->receiver_ID != referee->robot_id)
    {
        return PARSE_FAILED; // wrong receiver id
    }

    // uint8_t interactive_data_ptr = data + Const_Referee_CMD_INTERACTIVE.data_length;

    // Interactive Data Recieve Callback

    return PARSE_SUCCEEDED;
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

graphic_data_struct_t Referee_dummyGraphicCmd = {{0x00, 0x00, 0x00}, Draw_OPERATE_NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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
 * @brief      ����ϵͳ�������ݷ��ͺ�����������
 * @param      data_cmd_id: ��������ID
 * @param      receiver_ID: ������ID
 * @param      interactive_data: ��������֡
 * @param      interactive_data_length: ��������֡����
 * @retval     ��
 */
void Referee_SendInteractiveData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *interactive_data, uint16_t interactive_data_length)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    static uint8_t bag_count = 0;
    uint8_t *buf = Referee_TxData;
    buf[0] = REFEREE_FRAME_HEADER_SOF;

    uint16_t *data_length_ptr = (void *)(buf + 1);
    *data_length_ptr = interactive_data_length + Const_Referee_CMD_INTERACTIVE.data_length;

    uint8_t *seq_ptr = (void *)(buf + 3);

    *seq_ptr = bag_count; // not obvious in doc

    if (bag_count != 255)
    {
        bag_count++;
    }
    else
    {
        bag_count = 0;
    }

    uint8_t *crc8_ptr = (void *)(buf + 4);
    *crc8_ptr = CRC_GetCRC8CheckSum(buf, 4, CRC8_INIT);

    buf[5] = 0x01;
    buf[6] = 0x03;

    ext_student_interactive_header_data_t *header = (void *)(buf + 7);
    header->data_cmd_id = data_cmd_id;
    header->receiver_ID = receiver_ID;
    header->sender_ID = (uint16_t)referee->robot_id; // 开头老三样输入

    memcpy(buf + 5 + Const_Referee_CMD_INTERACTIVE.data_length, interactive_data, interactive_data_length);

    uint16_t *crc16_ptr = (void *)(buf + 5 + 2 + *data_length_ptr);
    *crc16_ptr = CRC_GetCRC16CheckSum(buf, 5 + 2 + *data_length_ptr, CRC16_INIT);

    uint16_t tx_size = 5 + 2 + *data_length_ptr + 2;
    UART_Send(referee->huart, buf, tx_size, 15);
}

/**
 * @brief      �����˼佻�����ݷ��ͺ���
 * @param      data_cmd_id: 数据内容ID
 * @param      receiver_ID: 接受者ID
 * @param      data: 数据帧
 * @param      data_length: 数据帧长度
 * @retval     无
 */
void Referee_SendRobotCustomData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *data, uint16_t data_length)
{
    if (data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND ||
        data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND)
        return; // wrong data cmd id
    if (receiver_ID == 0 || (receiver_ID > 10 && receiver_ID < 110) || receiver_ID > 107)
        return; // wrong receiver id
    if (data_length > Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH)
        return; // interactive data too long
    Referee_SendInteractiveData(data_cmd_id, receiver_ID, data, data_length);
}

/**
 * @brief      ���Ϳͻ����Զ���ͼ��������
 * @param      graph: �������ָ��������ͼ������
 * @param      mode: ����ģʽ��1��2��3��4��Ӧ1��2��5��7��һ��
 * @retval     ��
 */
void Referee_SendDrawingCmd(graphic_data_struct_t graph[], uint8_t mode)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    if (mode == 0 || mode >= 5)
    {
        return;
    }

    uint8_t buf[120], cellsize = sizeof(graphic_data_struct_t);
    if (mode >= 1)
    {
        memcpy(buf, graph, cellsize);
    }
    if (mode >= 2)
    {
        memcpy(buf + cellsize, graph + 1, cellsize);
    }
    if (mode >= 3)
    {
        memcpy(buf + cellsize * 2, graph + 2, cellsize);
        memcpy(buf + cellsize * 3, graph + 3, cellsize);
        memcpy(buf + cellsize * 4, graph + 4, cellsize);
    }
    if (mode >= 4)
    {
        memcpy(buf + cellsize * 5, graph + 5, cellsize);
        memcpy(buf + cellsize * 6, graph + 6, cellsize);
    }

    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[mode].cmd_id, referee->client_id,
                                buf, Const_Referee_DATA_CMD_ID_LIST[mode].data_length);
}

/**
 * @brief      ���Ϳͻ����Զ���ͼ����ʾ�ַ�������
 * @param      pgraph: ָ��ָ����ʾ�ַ���ͼ������
 * @param      str: �ַ���������Ϊ30��
 * @retval     ��
 */
void Referee_SendDrawingStringCmd(graphic_data_struct_t *pgraph, const uint8_t str[], uint8_t len)
{
    Referee_DataTypeDef *referee = &Referee_Data;

    ext_client_custom_character_t struct_data;
    memcpy(&struct_data.grapic_data_struct, pgraph, sizeof(graphic_data_struct_t));
    memcpy(&struct_data.data, str, Const_Referee_DATA_CMD_ID_LIST[5].data_length - sizeof(graphic_data_struct_t));
    memcpy(&struct_data.data, str, len);
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[5].cmd_id, referee->client_id, (void *)&struct_data, Const_Referee_DATA_CMD_ID_LIST[5].data_length);
}

/**
 * @brief      �ͻ����Զ���ͼ�λ������Ƿ�Ϊ��
 * @param      ��
 * @retval     1Ϊ�գ�0Ϊ�ǿ�
 */
uint8_t Referee_IsDrawingBufferEmpty()
{
    Referee_DataTypeDef *referee = &Referee_Data;
    return referee->graphic_buf_len == 0;
}

/**
 * @brief      �ͻ����Զ���ͼ�λ�����ˢд����
 * @param      ��
 * @retval     ��
 */
void Referee_DrawingBufferFlush()
{
    Referee_DataTypeDef *referee = &Referee_Data;
    if (Referee_IsDrawingBufferEmpty())
    {
        return;
    }
    uint8_t cur = 0;
    while (cur + 7 < referee->graphic_buf_len)
    {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 4);
        cur += 7;
    }
    uint8_t remain = referee->graphic_buf_len - cur;
    if (remain > 5)
    {
        for (int i = remain; i < 7; ++i)
        {
            Referee_DrawingBufferPushDummy();
        }
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 4);
    }
    else if (remain > 2)
    {
        for (int i = remain; i < 5; ++i)
        {
            Referee_DrawingBufferPushDummy();
        }
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 3);
    }
    else if (remain == 2)
    {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 2);
    }
    else if (remain == 1)
    {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 1);
    }
    referee->graphic_buf_len = 0;
}

/**
 * @brief      ����ͼ���������ͻ����Զ���ͼ�λ�������ռλ�ã�
 * @param      ��
 * @retval     ��
 */
void Referee_DrawingBufferPushDummy()
{
    Referee_DataTypeDef *referee = &Referee_Data;
    memcpy(referee->graphic_buf + referee->graphic_buf_len, &Referee_dummyGraphicCmd, sizeof(graphic_data_struct_t));
    ++referee->graphic_buf_len;
}

/**
 * @brief      ��ͼ���������ͻ����Զ���ͼ�λ�����
 * @param      pgraph: ָ��ָ��ͼ������
 * @retval     ��
 */
void Referee_DrawingBufferPush(graphic_data_struct_t *pgraph)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    memcpy(referee->graphic_buf + referee->graphic_buf_len, pgraph, sizeof(graphic_data_struct_t));
    ++referee->graphic_buf_len;
    if (referee->graphic_buf_len >= 7)
    {
        Referee_DrawingBufferFlush();
    }
}

/**
 * @brief      �ͻ����Զ���ͼ��ʱ������
 * @param      ��
 * @retval     ��
 */
void Referee_DrawingTimeBaseCallback()
{
    static uint8_t tick = 0;
    ++tick;
    if (tick == 2)
    {
        tick = 0;
        Referee_DrawingBufferFlush();
    }
}

/**
 * @brief      ���ͼ������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
 */
uint32_t Referee_PackGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, Draw_OperateType operate_type, Draw_GraphicType graphic_type, uint8_t layer, Draw_Color color, uint16_t start_angle, uint16_t end_angle, uint8_t width, uint16_t start_x, uint16_t start_y, uint16_t radius, uint16_t end_x, uint16_t end_y)
{
    if (graph_id > 0xffffff)
    {
        return PARSE_FAILED;
    }
    pgraph->graphic_name[0] = graph_id & 0xff;
    pgraph->graphic_name[1] = (graph_id >> 8) & 0xff;
    pgraph->graphic_name[2] = (graph_id >> 16) & 0xff;

    pgraph->operate_type = (uint8_t)operate_type;
    pgraph->graphic_type = (uint8_t)graphic_type;

    if (layer > 9)
    {
        return PARSE_FAILED;
    }
    pgraph->layer = layer;

    pgraph->color = (uint8_t)color;

    if (start_angle > 0x7ff || end_angle > 0x7ff)
    {
        return PARSE_FAILED;
    }
    pgraph->start_angle = start_angle;
    pgraph->end_angle = end_angle;

    pgraph->width = width;

    if (start_x > 0x7ff || start_x > 0x7ff || radius > 0x3ff || end_x > 0x7ff || end_y > 0x7ff)
    {
        return PARSE_FAILED;
    }
    pgraph->start_x = start_x;
    pgraph->start_y = start_y;
    pgraph->radius = radius;
    pgraph->end_x = end_x;
    pgraph->end_y = end_y;

    return PARSE_SUCCEEDED;
}

/**
 * @brief      �����ʾ������ͼ������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
 */
uint32_t Referee_PackFloatGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, Draw_OperateType operate_type, uint8_t layer, Draw_Color color, uint16_t font_size, uint16_t decimal_digit, uint8_t width, uint16_t start_x, uint16_t start_y, float value)
{
    Referee_GraphicDataConverterUnion conv;
    conv.int_data = (int32_t)(value * 1000.0f);
    uint16_t radius = (conv.ui32_data) & 0x3ff;
    uint16_t end_x = (conv.ui32_data >> 10) & 0x7ff;
    uint16_t end_y = (conv.ui32_data >> 21) & 0x7ff;
    return Referee_PackGraphicData(pgraph, graph_id, operate_type, Draw_TYPE_FLOAT, layer, color, font_size, decimal_digit, width, start_x, start_y, radius, end_x, end_y);
}

/**
 * @brief      �����ʾ����ͼ������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
 */
uint32_t Referee_PackIntGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, Draw_OperateType operate_type, uint8_t layer, Draw_Color color, uint16_t font_size, uint8_t width, uint16_t start_x, uint16_t start_y, int value)
{
    Referee_GraphicDataConverterUnion conv;
    conv.int_data = value;
    return Referee_PackGraphicData(pgraph, graph_id, operate_type, Draw_TYPE_INT, layer, color, font_size,
                                   0, width, start_x, start_y, conv.graphic_data.radius,
                                   conv.graphic_data.end_x, conv.graphic_data.end_y);
}

/**
 * @brief      �����ʾ�ַ���ͼ������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
 */
uint32_t Referee_PackStringGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, Draw_OperateType operate_type, uint8_t layer, Draw_Color color, uint16_t font_size, uint8_t length, uint8_t width, uint16_t start_x, uint16_t start_y)
{
    if (length > Const_Referee_DATA_CMD_ID_LIST[5].data_length - sizeof(graphic_data_struct_t))
    {
        return PARSE_FAILED;
    }
    return Referee_PackGraphicData(pgraph, graph_id, operate_type, Draw_TYPE_STRING, layer, color, font_size,
                                   length, width, start_x, start_y, 0, 0, 0);
}

/********** REFEREE CUSTOM GRAPHIC DRAWING FUNCTION **********/
/**
 * @brief      ��ͼ���������ָ��ͼ��
 * @param      layer: ͼ��ţ�0~9��
 * @retval     ��
 */
void Draw_ClearLayer(uint8_t layer)
{
    Referee_DataTypeDef *referee = &Referee_Data;
    Referee_DrawingBufferFlush();
    uint8_t buf[2];
    buf[0] = 1;
    buf[1] = layer;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[0].cmd_id, referee->client_id,
                                buf, Const_Referee_DATA_CMD_ID_LIST[0].data_length);
}

/**
 * @brief      ��ͼ���������ȫ��
 * @param      ��
 * @retval     ��
 */
void Draw_ClearAll()
{
    Referee_DataTypeDef *referee = &Referee_Data;
    // Referee_DrawingBufferFlush();
    referee->graphic_buf_len = 0; // ֱ�������������еĻ�ͼָ��
    uint8_t buf[2];
    buf[0] = 2;
    buf[1] = 0;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[0].cmd_id, referee->client_id,
                                buf, Const_Referee_DATA_CMD_ID_LIST[0].data_length);
}

/**
 * @brief      ��ͼ���������ָ��ͼ��
 * @param      graph_id: ͼ��ID
 * @retval     ��
 */
void Draw_Delete(uint32_t graph_id)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_DELETE, (Draw_GraphicType)0, 0,
                                (Draw_Color)0, 0, 0, 0, 0, 0, 0, 0, 0) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������ֱ�ߣ�������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddLine(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_LINE, layer, color,
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������ֱ�ߣ��޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyLine(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_LINE, layer, color,
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ�����������Σ�������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddRectangle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_RECTANGLE, layer, color,
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ�����������Σ��޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyRectangle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_RECTANGLE, layer, color,
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������Բ��������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddCircle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_CIRCLE, layer, color,
                                0, 0, width, center_x, center_y, radius, 0, 0) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������Բ���޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyCircle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_CIRCLE, layer, color,
                                0, 0, width, center_x, center_y, radius, 0, 0) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ����������Բ��������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddEllipse(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_ELLIPSE, layer, color,
                                0, 0, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ����������Բ���޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyEllipse(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_ELLIPSE, layer, color,
                                0, 0, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������Բ����������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddArc(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t start_angle, uint16_t end_angle, uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_ARC, layer, color,
                                start_angle, end_angle, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������Բ�����޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyArc(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t start_angle, uint16_t end_angle, uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y)
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_ARC, layer, color,
                                start_angle, end_angle, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������ʾ��������������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddFloat(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint16_t decimal_digit, uint8_t width, uint16_t start_x, uint16_t start_y, float value)
{
    graphic_data_struct_t graph;
    if (Referee_PackFloatGraphicData(&graph, graph_id, Draw_OPERATE_ADD, layer, color,
                                     font_size, decimal_digit, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������ʾ���������޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyFloat(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint16_t decimal_digit, uint8_t width, uint16_t start_x, uint16_t start_y, float value)
{
    graphic_data_struct_t graph;
    if (Referee_PackFloatGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, layer, color,
                                     font_size, decimal_digit, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������ʾ������������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddInt(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint8_t width, uint16_t start_x, uint16_t start_y, int value)
{
    graphic_data_struct_t graph;
    if (Referee_PackIntGraphicData(&graph, graph_id, Draw_OPERATE_ADD, layer, color,
                                   font_size, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������ʾ�������޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyInt(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint8_t width, uint16_t start_x, uint16_t start_y, int value)
{
    graphic_data_struct_t graph;
    if (Referee_PackIntGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, layer, color,
                                   font_size, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}

/**
 * @brief      ��ͼ��������ʾ�ַ�����������
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_AddString(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint8_t width, uint16_t start_x, uint16_t start_y, const char str[])
{
    graphic_data_struct_t graph;
    //    Referee_DrawingBufferFlush();
    uint8_t len = strlen(str);
    if (Referee_PackStringGraphicData(&graph, graph_id, Draw_OPERATE_ADD, layer, color,
                                      font_size, len, width, start_x, start_y) != PARSE_SUCCEEDED)
        return;
    uint8_t buf[35];
    memcpy(buf, str, len);
    Referee_SendDrawingStringCmd(&graph, buf, len + 1);
}

/**
 * @brief      ��ͼ��������ʾ�ַ������޸ģ�
 * @param      ���Э�鼰ͷ�ļ�����
 * @retval     ��
 */
void Draw_ModifyString(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint8_t width, uint16_t start_x, uint16_t start_y, const char str[])
{
    graphic_data_struct_t graph;
    uint8_t len = strlen(str);
    if (Referee_PackStringGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, layer, color,
                                      font_size, len, width, start_x, start_y) != PARSE_SUCCEEDED)
        return;
    uint8_t buf[35];
    memcpy(buf, str, len);
    Referee_SendDrawingStringCmd(&graph, buf, len + 1);
    //		Referee_DrawingBufferFlush();
}

/********** END OF REFEREE CUSTOM GRAPHIC DRAWING FUNCTION **********/
