/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-19 23:55:56
 */

#include "config_ctrl.h"
#include "protocol_board.h"
#include "protocol_referee.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "lib_buff.h"
#include "util_fdcan.h"

BoardCom_DataTypeDef BoardCom_Data;

/**
 * @brief      Gets the pointer to the bus communication data object
 * @param      NULL
 * @retval     Pointer to bus communication data object
 */
BoardCom_DataTypeDef *BoardCom_GetDataPtr()
{
    return &BoardCom_Data;
}

uint8_t BoardCom_TxData[BOARDCOM_TX_LEN];
uint8_t BoardCom_RxData[BOARDCOM_RX_LEN];

static void _send_referee_data_1(uint8_t buff[]);
static void _send_referee_data_2(uint8_t buff[]);
static void _send_cap_mode(uint8_t buff[]);

static void _set_referee_data_1(uint8_t buff[]);
static void _set_referee_data_2(uint8_t buff[]);
static void _set_control(uint8_t buff[]);
static void _set_imu_yaw(uint8_t buff[]);
static void _set_cha_ref(uint8_t buff[]);
static void _set_ui_state(uint8_t buff[]);

static void _set_cap_data_1(uint8_t buff[]);
static void _set_cap_data_2(uint8_t buff[]);

Board_TableEntryTypeDef Cha_Send[BOARDCOM_CHASSIS_BUFF_SIZE] = {
    {SEND_CHASSIS_DATA_1, &_send_referee_data_1},
    {SEND_CHASSIS_DATA_2, &_send_referee_data_2},
    {SEND_CHASSIS_DATA_3, &_send_cap_mode}};

Board_TableEntryTypeDef Cha_Receive[BOARDCOM_RECEIVE_SIZE] = {
    {0xff, NULL},
    {SET_REFEREE_DATA_1, &_set_referee_data_1},
    {SET_REFEREE_DATA_2, &_set_referee_data_2},
    {SET_CONTROL, &_set_control},
    {SET_IMU_YAW, &_set_imu_yaw},
    {SET_CHA_REF, &_set_cha_ref},
    {SET_UI_STATE, &_set_ui_state}};

Board_TableEntryTypeDef Cap_Receive[CAPCOM_RECEIVE_SIZE] = {
    {0xff, NULL},
    {SET_CAP_DATA_1, &_set_cap_data_1},
    {SET_CAP_DATA_2, &_set_cap_data_2}};

FDCAN_HandleTypeDef *BOARD_CAN_HANDLER = &hfdcan2;
FDCAN_HandleTypeDef *CAP_CAN_HANDLER = &hfdcan3;

FDCAN_TxHeaderTypeDef BoardCom_GimControl;
FDCAN_TxHeaderTypeDef BoardCom_GimImuYaw;
FDCAN_TxHeaderTypeDef BoardCom_GimChassisRef;
FDCAN_TxHeaderTypeDef BoardCom_GimUIState;
FDCAN_TxHeaderTypeDef BoardCom_ChaRefereeData1;
FDCAN_TxHeaderTypeDef BoardCom_ChaRefereeData2;
FDCAN_TxHeaderTypeDef BoardCom_CapMode;
FDCAN_TxHeaderTypeDef BoardCom_CapState1;
FDCAN_TxHeaderTypeDef BoardCom_CapState2;

/**
 * @brief      Inter bus communication initialization
 * @param      NULL
 * @retval     NULL
 */
void BoardCom_Init()
{
    BoardCom_Reset();
    FDCAN_InitTxHander(&BoardCom_GimControl, SET_CONTROL);
    FDCAN_InitTxHander(&BoardCom_GimImuYaw, SET_IMU_YAW);
    FDCAN_InitTxHander(&BoardCom_GimChassisRef, SET_CHA_REF);
    FDCAN_InitTxHander(&BoardCom_GimUIState, SET_UI_STATE);
    FDCAN_InitTxHander(&BoardCom_ChaRefereeData1, SET_REFEREE_DATA_1);
    FDCAN_InitTxHander(&BoardCom_ChaRefereeData2, SET_REFEREE_DATA_2);
    FDCAN_InitTxHander(&BoardCom_CapMode, SET_CAP_MODE);
    FDCAN_InitTxHander(&BoardCom_CapState1, SET_CAP_DATA_1);
    FDCAN_InitTxHander(&BoardCom_CapState2, SET_CAP_DATA_2);
}

/**
 * @brief      Reset inter bus communication data object
 * @param      NULL
 * @retval     NULL
 */
void BoardCom_Reset()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->last_update_time[1] = HAL_GetTick();
    boardcom->yaw_limited_angle = 0;
    boardcom->yaw_consequent_angle = 0;
    boardcom->robot_id = 0;
    boardcom->cooling_per_second = 0;
    boardcom->power_limit = 45;
    boardcom->heat_limit = 0;
    boardcom->speed_17mm_limit = 0;
}

/**
 * @brief      Data sending function of serial port in inter bus communication
 * @param      NULL
 * @retval     NULL
 */
void BoardCom_Send()
{
    BoardCom_Update();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->state = BOARDCOM_PENDING;

    uint32_t out_time = HAL_GetTick();
    for (int i = 0; i < BOARDCOM_CHASSIS_BUFF_SIZE; i++)
    {
        if (Cha_Send[i].bus_func != NULL)
        {
            Cha_Send[i].bus_func(BoardCom_TxData);
        }
    }
    boardcom->state = BOARDCOM_CONNECTED;
}

/**
 * @brief      Interrupt callback function of can in inter Bus communication
 * @param
 * @retval     NULL
 */
void BoardCom_Decode(FDCAN_HandleTypeDef *pfdhcan, uint32_t stdid, uint8_t rxdata[], uint32_t len)
{
    if (pfdhcan == BOARD_CAN_HANDLER)
    {
        BoardCom_DecodeBoard(rxdata, stdid, len);
    }
    else if (pfdhcan == CAP_CAN_HANDLER)
    {
        BoardCom_DecodeCap(rxdata, stdid, len);
    }
}

/**
 * @brief      Data decoding function of serial port in inter bus communication
 * @param      buff: Data buffer
 * @param      rxdatalen: data length
 * @retval     NULL
 */
uint32_t decode_cont = 0;
float decode_rate;
void BoardCom_DecodeBoard(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen)
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    decode_cont++;
    decode_rate = decode_cont / HAL_GetTick();
    memcpy(BoardCom_RxData, buff, rxdatalen);

    for (int i = 1; i < (BOARDCOM_RECEIVE_SIZE); i++)
    {
        if ((stdid == Cha_Receive[i].cmd_id) && (Cha_Receive[i].bus_func != NULL))
        {
            Cha_Receive[i].bus_func(BoardCom_RxData);
            return;
        }
    }
}

/**
 * @brief      Data decoding function of serial port in cap communication
 * @param      buff: Data buffer
 * @param      rxdatalen: data length
 * @retval     NULL
 */
uint32_t decode_cont2 = 0;
float decode_rate2;
void BoardCom_DecodeCap(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen)
{
    decode_cont2++;
    decode_rate2 = decode_cont2 / HAL_GetTick();
    memcpy(BoardCom_RxData, buff, rxdatalen);

    for (int i = 1; i < (CAPCOM_RECEIVE_SIZE); i++)
    {
        if ((stdid == Cap_Receive[i].cmd_id) && (Cap_Receive[i].bus_func != NULL))
        {
            Cap_Receive[i].bus_func(BoardCom_RxData);
            return;
        }
    }
}

/**
 * @brief      Assignment of inter bus communication structure
 * @param      NULL
 * @retval     NULL
 */
void BoardCom_Update()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
    Referee_DataTypeDef *referee = Referee_GetDataPtr();

    boardcom->yaw_consequent_angle = Motor_GimbalYaw.encoder.consequent_angle;
    boardcom->yaw_limited_angle = Motor_GimbalYaw.encoder.limited_angle;
    boardcom->robot_id = referee->robot_id;
    boardcom->power_limit = referee->chassis_power_limit;
//    boardcom->heat_now = referee->shooter_17mm_1_barrel_heat;
	boardcom->cooling_per_second = referee->shooter_barrel_cooling_value;
    boardcom->shoot_spd_referee = referee->initial_speed;
    boardcom->heat_limit = referee->shooter_barrel_heat_limit;
}

void BoardCom_ChassisModeSet()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	
    GimbalYaw_SetRef(boardcom->yaw_ref);
    GimbalYaw_SetAngleFdb(boardcom->yaw_pos_fdb);
    GimbalYaw_SetSpeedFdb(boardcom->yaw_spd_fdb);

    switch (boardcom->yaw_mode)
    {
    case GIMBAL_YAW_NO_AUTO:
    {
        GimbalYaw_ModeSet(GimbalYaw_NO_AUTO);
        break;
    }
    case GIMBAL_YAW_ARMOR:
    {
        GimbalYaw_ModeSet(GimbalYaw_ARMOR);
        break;
    }
    case GIMBAL_YAW_BIG_ENERGY:
    {
        GimbalYaw_ModeSet(GimbalYaw_BIG_ENERGY);
        break;
    }
    case GIMBAL_YAW_SMALL_ENERGY:
    {
        GimbalYaw_ModeSet(GimbalYaw_SMALL_ENERGY);
        break;
    }
    default:
        return;
    }

    switch (boardcom->chassis_mode)
    {
    case CHASSIS_CTRL_STOP:
    {
        Chassis_ModeSet(CHASSIS_STOP);
        Chassis_SetMoveRef(0, 0);
        break;
    }
    case CHASSIS_CTRL_NORMAL:
    {
        Chassis_ModeSet(CHASSIS_NORMAL);
        Chassis_SetMoveRef(boardcom->chassis_fb_ref, boardcom->chassis_lr_ref);
        break;
    }
    case CHASSIS_CTRL_GYRO:
    {
        Chassis_ModeSet(CHASSIS_GYRO);
        Chassis_SetMoveRef(boardcom->chassis_fb_ref, boardcom->chassis_lr_ref);
        break;
    }
    default:
        return;
    }
}

/**
 * @brief      Check whether the dual bus communication is offline
 * @param      NULL
 * @retval     NULL
 */
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
int counta[7];
float ratea[7];
static void _send_referee_data_1(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_PERIOD_REFEREE1)
    {
        return;
    }
    last_send_time = HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &BoardCom_ChaRefereeData1;
    counta[0]++;
    ratea[0] = 1000 * counta[0] / HAL_GetTick();
    memset(buff, 0, 8);

    float2buff(boardcom->yaw_consequent_angle, buff);
    i162buff((int16_t)(boardcom->shoot_spd_referee * 100), buff + 4);
    ui162buff(boardcom->cooling_per_second, buff + 6);

    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_referee_data_2(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_PERIOD_REFEREE2)
    {
        return;
    }
    last_send_time = HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &BoardCom_ChaRefereeData2;
    counta[1]++;
    ratea[1] = 1000 * counta[1] / HAL_GetTick();
    memset(buff, 0, 8);
    buff[0] = (1 << 7) + boardcom->robot_id;
    buff[1] = boardcom->speed_17mm_limit << 6;
    i162buff(boardcom->heat_limit, buff + 2);

    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_cap_mode(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_PERIOD_CAP_MODE)
    {
        return;
    }
    last_send_time = HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &BoardCom_CapMode;
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
int countb[8];
float rateb[8];
static void _set_referee_data_1(uint8_t buff[])
{
    countb[BOARDCOM_PKG_REFEREE1]++;
    rateb[BOARDCOM_PKG_REFEREE1] = 1000 * countb[BOARDCOM_PKG_REFEREE1] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    static float yaw_position;
    yaw_position = buff2float(buff);
    boardcom->yaw_consequent_angle = yaw_position;
    while (yaw_position > 180)
    {
        yaw_position -= 360;
    }
    while (yaw_position < -180)
    {
        yaw_position += 360;
    }
    boardcom->yaw_limited_angle = yaw_position;
    boardcom->shoot_spd_referee = ((float)buff2i16(buff + 4)) / 100;
    boardcom->cooling_per_second = buff2ui16(buff + 6);
    boardcom->last_update_time[BOARDCOM_PKG_REFEREE1] = HAL_GetTick();
}

static void _set_referee_data_2(uint8_t buff[])
{
    countb[BOARDCOM_PKG_REFEREE2]++;
    rateb[BOARDCOM_PKG_REFEREE2] = 1000 * countb[BOARDCOM_PKG_REFEREE2] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->robot_id = buff[0] & 0x7F;
    boardcom->speed_17mm_limit = (buff[1] & 0xC0) >> 6;
    boardcom->heat_limit = buff2i16(buff + 2);
    boardcom->last_update_time[BOARDCOM_PKG_REFEREE2] = HAL_GetTick();
}

static void _set_control(uint8_t buff[])
{
    countb[BOARDCOM_PKG_CTRL]++;
    rateb[BOARDCOM_PKG_CTRL] = 1000 * countb[BOARDCOM_PKG_CTRL] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    if ((boardcom->cap_mode_user == SUPERCAP_CTRL_ON) && (buff[3] == SUPERCAP_CTRL_OFF))
    {
        boardcom->power_path_change_flag = HAL_GetTick();
    }
    boardcom->yaw_mode = buff[0] >> 4;
    boardcom->chassis_mode = (buff[0] & 0x0E) >> 1;
    boardcom->power_limit_mode = buff[0] & 0x01;
    boardcom->cap_mode_user = (buff[1] & 0x02) >> 1;
    boardcom->ui_cmd = (buff[1] & 0x04) >> 2;
    boardcom->fly_flag = (buff[1] & 0x08) >> 3;
    boardcom->gyro_dir = (buff[1] & 0x10)>> 4;
    boardcom->is_get_target = (buff[1] & 0x20) >> 5;
    boardcom->check_in = (buff[1] & 0x40) >> 6;
    boardcom->cap_speedup_flag = buff[1] & 0x01;
    boardcom->yaw_ref = buff2float(buff + 2);
    BoardCom_ChassisModeSet();
    boardcom->last_update_time[BOARDCOM_PKG_CTRL] = HAL_GetTick();
}

static void _set_imu_yaw(uint8_t buff[])
{
    countb[BOARDCOM_PKG_IMU]++;
    rateb[BOARDCOM_PKG_IMU] = 1000 * countb[BOARDCOM_PKG_IMU] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->yaw_spd_fdb = buff2float(buff);
    boardcom->yaw_pos_fdb = buff2float(buff + 4);
    boardcom->last_update_time[BOARDCOM_PKG_IMU] = HAL_GetTick();
}

static void _set_cha_ref(uint8_t buff[])
{
    countb[BOARDCOM_PKG_CHA_REF]++;
    rateb[BOARDCOM_PKG_CHA_REF] = 1000 * countb[BOARDCOM_PKG_CHA_REF] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->chassis_fb_ref = (float)(buff2i16(buff) / 10.0f);
    boardcom->chassis_lr_ref = (float)(buff2i16(buff + 2) / 10.0f);
    boardcom->last_update_time[BOARDCOM_PKG_CHA_REF] = HAL_GetTick();
}

static void _set_ui_state(uint8_t buff[])
{
    countb[BOARDCOM_PKG_UI_STATE]++;
    rateb[BOARDCOM_PKG_UI_STATE] = 1000 * countb[BOARDCOM_PKG_UI_STATE] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    boardcom->pitch_angle = ((float)buff2i16(buff)) / 100;
    boardcom->magazine_state = (buff[2] & 0x80) >> 7;
    boardcom->shooter_state = (buff[2] & 0x40) >> 6;
    boardcom->auto_shoot_state = (buff[2] & 0x30) >> 4;
// 留出通讯接口，需要时使用
    boardcom->team_color = (buff[2] & 0x0F);
    //    boardcom->minipc_offset_horizental = buff[3];
    //    boardcom->minipc_offset_vertical = buff[4];
	boardcom->ui_x = buff[5];
	boardcom->ui_y = buff[6];
    boardcom->last_update_time[BOARDCOM_PKG_UI_STATE] = HAL_GetTick();
}

static void _set_cap_data_1(uint8_t buff[])
{
    countb[BOARDCOM_PKG_CAP1]++;
    rateb[BOARDCOM_PKG_CAP1] = 1000 * countb[BOARDCOM_PKG_CAP1] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    boardcom->cap_power = buff2float(buff);
    boardcom->cap_rest_energy = buff[4];
    boardcom->last_update_time[BOARDCOM_PKG_CAP1] = HAL_GetTick();
}

static void _set_cap_data_2(uint8_t buff[])
{
    countb[BOARDCOM_PKG_CAP2]++;
    rateb[BOARDCOM_PKG_CAP2] = 1000 * countb[BOARDCOM_PKG_CAP2] / HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    boardcom->cap_voltage = buff2float(buff);
    boardcom->cap_current = buff2float(buff + 4);
    boardcom->last_update_time[BOARDCOM_PKG_CAP2] = HAL_GetTick();
}
