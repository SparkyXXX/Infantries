/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-08-16 18:52:21
 */

#include "protocol_board.h"
#include "lib_buff.h"
#include "protocol_referee.h"
#include "util_fdcan.h"

#define BOARDCOM_TX_LEN 64
#define BOARDCOM_RX_LEN 64
#define BOARDCOM_LOST_TIME 200

#define SEND_REFEREE_DATA1_TIMEOUT 5
#define SEND_REFEREE_DATA2_TIMEOUT 5
#define SEND_CAP_DATA_TIMEOUT 5

#define CHASSIS_SEND_TO_GIMBAL_PKG_AMOUNT 3
#define CHASSIS_RECEIVE_FROM_GIMBAL_PKG_AMOUNT 4
#define CHASSIS_RECEIVE_FROM_CAP_PKG_AMOUNT 1

#define ID_SEND_REFEREE_DATA1 0x205
#define ID_SEND_REFEREE_DATA2 0x206
#define ID_SEND_CAP_DATA 0x98
#define ID_RECEIVE_CONTROL 0x201
#define ID_RECEIVE_IMU_YAW 0x202
#define ID_RECEIVE_CHASSIS_REF 0x203
#define ID_RECEIVE_UI_STATE 0x204
#define ID_RECEIVE_CAP_DATA 0x299

BoardCom_DataTypeDef BoardCom_Data;

uint8_t BoardCom_TxData[BOARDCOM_TX_LEN];
uint8_t BoardCom_RxData[BOARDCOM_RX_LEN];
uint8_t boardcom_decoded_count = 0;

Board_SendTableEntryTypeDef Chassis_Send_to_Gimbal[CHASSIS_SEND_TO_GIMBAL_PKG_AMOUNT] =
    {&_send_referee_data1,
     &_send_referee_data2,
     &_send_cap_data};

Board_ReceiveTableEntryTypeDef Chassis_Receive_from_Gimbal[CHASSIS_RECEIVE_FROM_GIMBAL_PKG_AMOUNT] =
    {{ID_RECEIVE_CONTROL, &_receive_control},
     {ID_RECEIVE_IMU_YAW, &_receive_imu_yaw},
     {ID_RECEIVE_CHASSIS_REF, &_receive_chassis_ref},
     {ID_RECEIVE_UI_STATE, &_receive_ui_state}};

Board_ReceiveTableEntryTypeDef Chassis_Receive_from_Cap[CHASSIS_RECEIVE_FROM_CAP_PKG_AMOUNT] =
    {{ID_RECEIVE_CAP_DATA, &_receive_cap_data}};

FDCAN_HandleTypeDef *BOARD_CAN_HANDLER = &hfdcan2;
FDCAN_HandleTypeDef *CAP_CAN_HANDLER = &hfdcan3;

FDCAN_TxHeaderTypeDef TxHeader_Chassis_to_Gimbal1;
FDCAN_TxHeaderTypeDef TxHeader_Chassis_to_Gimbal2;
FDCAN_TxHeaderTypeDef TxHeader_Chassis_to_Cap;

BoardCom_DataTypeDef *BoardCom_GetDataPtr()
{
    return &BoardCom_Data;
}

void BoardCom_Init()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->cap_rest_energy = 40;
    FDCAN_InitTxHeader(&TxHeader_Chassis_to_Gimbal1, ID_SEND_REFEREE_DATA1);
    FDCAN_InitTxHeader(&TxHeader_Chassis_to_Gimbal2, ID_SEND_REFEREE_DATA2);
    FDCAN_InitTxHeader(&TxHeader_Chassis_to_Cap, ID_SEND_CAP_DATA);
}

void BoardCom_Update()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Referee_DataTypeDef *referee = Referee_GetDataPtr();

    boardcom->robot_id = referee->robot_id;
    boardcom->cooling_per_second = referee->shooter_barrel_cooling_value;
    boardcom->shoot_spd_referee = referee->initial_speed;
    boardcom->heat_limit = referee->shooter_barrel_heat_limit;
    boardcom->power_limit = referee->chassis_power_limit;
    boardcom->power_level_limit = referee->chassis_power_level_limit;
    boardcom->shooter_heat_referee = referee->shooter_17mm_1_barrel_heat;
    boardcom->power_buffer = referee->buffer_energy;
    boardcom->chassis_power = referee->chassis_power;
    boardcom->power_management_shooter_output = referee->power_management_shooter_output;
    boardcom->game_progress = referee->game_progress;
    if (referee->stage_remain_time != 0)
    {
        boardcom->stage_remain_time = referee->stage_remain_time;
    }
}

void BoardCom_Send()
{
    BoardCom_Update();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->state = BOARDCOM_PENDING;

    uint32_t out_time = HAL_GetTick();
    for (int i = 0; i < CHASSIS_SEND_TO_GIMBAL_PKG_AMOUNT; i++)
    {
        if (Chassis_Send_to_Gimbal[i].bus_func != NULL)
        {
            Chassis_Send_to_Gimbal[i].bus_func(BoardCom_TxData);
        }
    }
    boardcom->state = BOARDCOM_CONNECTED;
}

void BoardCom_Decode(FDCAN_HandleTypeDef *pfdhcan, uint32_t stdid, uint8_t rxdata[], uint32_t len)
{
    if (pfdhcan == BOARD_CAN_HANDLER)
    {
        boardcom_decoded_count = 0;
        memcpy(BoardCom_RxData, rxdata, len);
        for (int i = 0; i < CHASSIS_RECEIVE_FROM_GIMBAL_PKG_AMOUNT; i++)
        {
            if ((stdid == Chassis_Receive_from_Gimbal[i].cmd_id) && (Chassis_Receive_from_Gimbal[i].bus_func != NULL))
            {
                Chassis_Receive_from_Gimbal[i].bus_func(BoardCom_RxData);
                return;
            }
        }
    }
    else if (pfdhcan == CAP_CAN_HANDLER)
    {
        memcpy(BoardCom_RxData, rxdata, len);

        for (int i = 0; i < CHASSIS_RECEIVE_FROM_CAP_PKG_AMOUNT; i++)
        {
            if ((stdid == Chassis_Receive_from_Cap[i].cmd_id) && (Chassis_Receive_from_Cap[i].bus_func != NULL))
            {
                Chassis_Receive_from_Cap[i].bus_func(BoardCom_RxData);
                return;
            }
        }
    }
}

uint8_t BoardCom_IsLost(uint8_t index)
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if (HAL_GetTick() - boardcom->last_update_time[index] > BOARDCOM_LOST_TIME)
    {
        while (1)
        {
            ;
        }
    }
    return 0;
}

/*************** SEND *****************/
int counta[3];
float ratea[3];
static void _send_referee_data1(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_REFEREE_DATA1_TIMEOUT)
    {
        return;
    }
    last_send_time = HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &TxHeader_Chassis_to_Gimbal1;
    counta[0]++;
    ratea[0] = 1000 * counta[0] / HAL_GetTick();
    memset(buff, 0, 8);

    buff[0] = (1 << 7) + boardcom->robot_id;
    buff[1] = boardcom->power_management_shooter_output << 7;
    i162buff(boardcom->heat_limit, buff + 2);
    ui162buff((uint16_t)(boardcom->shoot_spd_referee * 100), buff + 4);
    ui162buff(boardcom->cooling_per_second, buff + 6);
    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_referee_data2(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_REFEREE_DATA2_TIMEOUT)
    {
        return;
    }
    last_send_time = HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &TxHeader_Chassis_to_Gimbal2;
    counta[1]++;
    ratea[1] = 1000 * counta[1] / HAL_GetTick();
    memset(buff, 0, 8);

    ui162buff(boardcom->stage_remain_time, buff);
    buff[2] = boardcom->game_progress;
    ui162buff(boardcom->shooter_heat_referee, buff + 3);
    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

// 与超电的通信协议，与硬件协商前禁止改动
static void _send_cap_data(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_CAP_DATA_TIMEOUT)
    {
        return;
    }
    last_send_time = HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &TxHeader_Chassis_to_Cap;
    counta[2]++;
    ratea[2] = 1000 * counta[2] / HAL_GetTick();
    memset(buff, 0, 8);

    buff[0] = boardcom->cap_mode_flag | (0x77 << 1);
    buff[1] = boardcom->power_level_limit;
    buff[2] = boardcom->power_limit;
    buff[3] = boardcom->power_buffer;
    float2buff(boardcom->chassis_power, buff + 4);
    FDCAN_Send(CAP_CAN_HANDLER, pheader, buff);
}

/*************** RECEIVE *****************/
int countb[5];
float rateb[5];
static void _receive_control(uint8_t buff[])
{
    countb[0]++;
    rateb[0] = 1000 * countb[0] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->yaw_mode = buff[0] >> 4;
    boardcom->chassis_mode = (buff[0] & 0x0E) >> 1;
    boardcom->power_limit_mode = buff[0] & 0x01;
    boardcom->ui_cmd = (buff[1] & 0x04) >> 2;
    boardcom->fly_flag = (buff[1] & 0x08) >> 3;
    boardcom->gyro_dir = (buff[1] & 0x10) >> 4;
    boardcom->is_get_target = (buff[1] & 0x20) >> 5;
    boardcom->check_in = (buff[1] & 0x40) >> 6;
    boardcom->cap_speedup_flag = buff[1] & 0x01;
    boardcom->yaw_ref = buff2float(buff + 2);
    boardcom->last_update_time[0] = HAL_GetTick();
}

static void _receive_imu_yaw(uint8_t buff[])
{
    countb[1]++;
    rateb[1] = 1000 * countb[1] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->yaw_spd_fdb = buff2float(buff);
    boardcom->yaw_pos_fdb = buff2float(buff + 4);
    boardcom->last_update_time[1] = HAL_GetTick();
}

static void _receive_chassis_ref(uint8_t buff[])
{
    countb[2]++;
    rateb[2] = 1000 * countb[2] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->chassis_fb_ref = (float)(buff2i16(buff) / 10.0f);
    boardcom->chassis_lr_ref = (float)(buff2i16(buff + 2) / 10.0f);
    boardcom->last_update_time[2] = HAL_GetTick();
}

static void _receive_ui_state(uint8_t buff[])
{
    countb[3]++;
    rateb[3] = 1000 * countb[3] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->pitch_angle = ((float)buff2i16(buff)) / 100;
    boardcom->magazine_state = (buff[2] & 0x80) >> 7;
    boardcom->shooter_state = (buff[2] & 0x40) >> 6;
    boardcom->auto_shoot_state = (buff[2] & 0x30) >> 4;
    boardcom->team_color = (buff[2] & 0x0F);
    boardcom->autoaim_yaw_spd_ref = buff2float(buff + 3);
    boardcom->last_update_time[3] = HAL_GetTick();
}

// 与超电的通信协议，与硬件协商前禁止改动
static void _receive_cap_data(uint8_t buff[])
{
    countb[4]++;
    rateb[4] = 1000 * countb[4] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->cap_power = buff2float(buff);
    boardcom->cap_rest_energy = buff[4];
    boardcom->buck_version = buff[5] & 0x0F;
    boardcom->boost_version = (buff[5] & 0xF0) >> 4;
    boardcom->last_update_time[4] = HAL_GetTick();
}
