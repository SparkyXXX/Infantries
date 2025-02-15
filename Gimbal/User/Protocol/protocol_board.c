/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-26 19:46:43
 */

#include "protocol_board.h"

#define BOARDCOM_TX_LEN 64
#define BOARDCOM_RX_LEN 64
#define BOARDCOM_LOST_TIME 200

#define SEND_CONTROL_TIMEOUT 2
#define SEND_IMU_YAW_TIMEOUT 2
#define SEND_CHAREF_TIMEOUT 5
#define SEND_UI_STATE_TIMEOUT 10

#define GIMBAL_SEND_TOCHASSIS_PKG1_AMOUNT 2
#define GIMBAL_SEND_TOCHASSIS_PKG2_AMOUNT 2
#define GIMBAL_RECEIVE_FROM_CHASSIS_PKG_AMOUNT 2

#define ID_SEND_CONTROL 0x201
#define ID_SEND_IMU_YAW 0x202
#define ID_SEND_CHASSIS_REF 0x203
#define ID_SEND_UI_STATE 0x204
#define ID_RECEIVE_REFEREE_DATA1 0x205
#define ID_RECEIVE_REFEREE_DATA2 0x206

BoardCom_DataTypeDef BoardCom_Data;

uint8_t BoardCom_TxData1[BOARDCOM_TX_LEN];
uint8_t BoardCom_TxData2[BOARDCOM_TX_LEN];
uint8_t BoardCom_RxData[BOARDCOM_RX_LEN];

Board_SendTableEntryTypeDef Gimbal_Send_to_Chassis_Pkg1[GIMBAL_SEND_TOCHASSIS_PKG1_AMOUNT] =
    {
        {&_send_control},
        {&_send_imu_yaw}};

Board_SendTableEntryTypeDef Gimbal_Send_to_Chassis_Pkg2[GIMBAL_SEND_TOCHASSIS_PKG2_AMOUNT] =
    {
        {&_send_chassis_ref},
        {&_send_ui_state}};

Board_ReceiveTableEntryTypeDef Gimbal_Receive_from_Chassis[GIMBAL_RECEIVE_FROM_CHASSIS_PKG_AMOUNT] =
    {{ID_RECEIVE_REFEREE_DATA1, &_receive_referee_data1},
     {ID_RECEIVE_REFEREE_DATA2, &_receive_referee_data2}};

FDCAN_HandleTypeDef *BOARD_CAN_HANDLER = &hfdcan2;

FDCAN_TxHeaderTypeDef TxHeader_Gimbal_Control;
FDCAN_TxHeaderTypeDef TxHeader_Gimbal_ImuYaw;
FDCAN_TxHeaderTypeDef TxHeader_Gimbal_ChassisRef;
FDCAN_TxHeaderTypeDef TxHeader_Gimbal_UIState;

BoardCom_DataTypeDef *BoardCom_GetDataPtr()
{
    return &BoardCom_Data;
}

void BoardCom_Init()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_InitTxHeader(&TxHeader_Gimbal_Control, ID_SEND_CONTROL);
    FDCAN_InitTxHeader(&TxHeader_Gimbal_ImuYaw, ID_SEND_IMU_YAW);
    FDCAN_InitTxHeader(&TxHeader_Gimbal_ChassisRef, ID_SEND_CHASSIS_REF);
    FDCAN_InitTxHeader(&TxHeader_Gimbal_UIState, ID_SEND_UI_STATE);
    boardcom->power_limit_mode = 1;
}

void BoardCom_Update()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    Remote_ControlTypeDef *remote_control = Remote_GetControlPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    INS_DataTypeDef *ins = INS_GetControlPtr();

    boardcom->yaw_mode = gimbal->yaw_mode;
    boardcom->yaw_ref = gimbal->yaw_position_ref;
    boardcom->yaw_pos_fdb = ins->yaw_consequent;
    boardcom->yaw_spd_fdb = ins->gyro[YAW];
    boardcom->pitch_angle = -ins->pitch;
    boardcom->autoaim_yaw_spd_ref = minipc->armor_yaw_spd_ref / 100 / 57.3f;
    boardcom->magazine_state = remote_control->mag_state;
    boardcom->shooter_state = (shooter->shoot_left.fdb > 10) && (shooter->shoot_right.fdb > 10);
    boardcom->auto_shoot_state = remote_control->autoshoot_flag;
}

uint32_t can_free_level1 = 0;
uint32_t can_free_level2 = 0;
void BoardComPkg1_Send()
{
    BoardCom_Update();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->state = BOARDCOM_PENDING;
    for (int i = 0; i < GIMBAL_SEND_TOCHASSIS_PKG1_AMOUNT; i++)
    {
        if (Gimbal_Send_to_Chassis_Pkg1[i].bus_func != NULL)
        {
            if (BOARD_CAN_HANDLER->ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL)
            {
                HAL_FDCAN_Stop(BOARD_CAN_HANDLER);
                BOARD_CAN_HANDLER->Instance->TXFQS = 0x03;
                HAL_FDCAN_Start(BOARD_CAN_HANDLER);
            }
            can_free_level1 = HAL_FDCAN_GetTxFifoFreeLevel(BOARD_CAN_HANDLER);
            Gimbal_Send_to_Chassis_Pkg1[i].bus_func(BoardCom_TxData1);
        }
    }
    boardcom->state = BOARDCOM_CONNECTED;
}

void BoardComPkg2_Send()
{
    BoardCom_Update();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->state = BOARDCOM_PENDING;
    for (int i = 0; i < GIMBAL_SEND_TOCHASSIS_PKG2_AMOUNT; i++)
    {
        if (Gimbal_Send_to_Chassis_Pkg2[i].bus_func != NULL)
        {
            if (BOARD_CAN_HANDLER->ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL)
            {
                HAL_FDCAN_Stop(BOARD_CAN_HANDLER);
                BOARD_CAN_HANDLER->Instance->TXFQS = 0x03;
                HAL_FDCAN_Start(BOARD_CAN_HANDLER);
            }
            can_free_level2 = HAL_FDCAN_GetTxFifoFreeLevel(BOARD_CAN_HANDLER);
            Gimbal_Send_to_Chassis_Pkg2[i].bus_func(BoardCom_TxData1);
        }
    }
    boardcom->state = BOARDCOM_CONNECTED;
}

void BoardCom_Decode(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen)
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    memcpy(BoardCom_RxData, buff, rxdatalen);
    for (int i = 0; i < (GIMBAL_RECEIVE_FROM_CHASSIS_PKG_AMOUNT); i++)
    {
        if ((stdid == Gimbal_Receive_from_Chassis[i].cmd_id) && (Gimbal_Receive_from_Chassis[i].bus_func != NULL))
        {
            Gimbal_Receive_from_Chassis[i].bus_func(buff);
            return;
        }
    }
}

uint8_t BoardCom_IsLost()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();

    if (HAL_GetTick() - boardcom->last_update_time > BOARDCOM_LOST_TIME)
    {
        while (1)
        {
            ;
        }
    }
    return 0;
}

/*************** SEND *****************/
int counta[4];
float ratea[4];
static void _send_control(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_CONTROL_TIMEOUT)
    {
        return;
    }
    last_send_time = HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &TxHeader_Gimbal_Control;
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    counta[0]++;
    ratea[0] = 1000 * counta[0] / HAL_GetTick();
    memset(buff, 0, 8);

    buff[0] = (boardcom->yaw_mode << 4) + (boardcom->chassis_mode << 1) + boardcom->power_limit_mode;
    buff[1] = (boardcom->check_in << 6) + (minipc->is_get_target << 5) + (boardcom->gyro_dir << 4) +
              (boardcom->fly_flag << 3) + (boardcom->ui_cmd << 2) + (0 << 1) + boardcom->cap_speedup_flag;
    float2buff(boardcom->yaw_ref, buff + 2);
    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_imu_yaw(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_IMU_YAW_TIMEOUT)
    {
        return;
    }
    last_send_time = HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &TxHeader_Gimbal_ImuYaw;
    counta[1]++;
    ratea[1] = 1000 * counta[1] / HAL_GetTick();
    memset(buff, 0, 8);

    float2buff(boardcom->yaw_spd_fdb, buff);
    float2buff(boardcom->yaw_pos_fdb, buff + 4);
    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_chassis_ref(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_CHAREF_TIMEOUT)
    {
        return;
    }
    last_send_time = HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &TxHeader_Gimbal_ChassisRef;
    counta[2]++;
    ratea[2] = 1000 * counta[2] / HAL_GetTick();
    memset(buff, 0, 8);

    i162buff((int16_t)(boardcom->chassis_fb_ref * 10), buff);
    i162buff((int16_t)(boardcom->chassis_lr_ref * 10), buff + 2);
    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_ui_state(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_UI_STATE_TIMEOUT)
    {
        return;
    }
    last_send_time = HAL_GetTick();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &TxHeader_Gimbal_UIState;
    counta[3]++;
    ratea[3] = 1000 * counta[3] / HAL_GetTick();
    memset(buff, 0, 8);

    i162buff((int16_t)(boardcom->pitch_angle * 100), buff);
    buff[2] = ((boardcom->magazine_state & 0x01) << 7) + ((boardcom->shooter_state & 0x01) << 6) +
              ((boardcom->auto_shoot_state & 0x03) << 4) + (minipc->team_color & 0x0F);
    float2buff(boardcom->autoaim_yaw_spd_ref, buff + 3);
    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

/*************** RECEIVE *****************/
int countb[2];
float rateb[2];
// uint16_t temp_heat_limit = 0;
// uint16_t temp_cool = 0;
static void _receive_referee_data1(uint8_t buff[])
{
    countb[0]++;
    rateb[0] = 1000 * countb[0] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->robot_id = buff[0] & 0x7F;
    boardcom->power_management_shooter_output = buff[1] >> 7;
    boardcom->heat_limit = buff2i16(buff + 2);
    //	boardcom->heat_limit = temp_heat_limit;
    boardcom->shoot_spd_referee = ((float)buff2ui16(buff + 4)) / 100;
    boardcom->cooling_per_second = buff2ui16(buff + 6);
    //	boardcom->cooling_per_second = temp_cool;
    boardcom->last_update_time = HAL_GetTick();
}

static void _receive_referee_data2(uint8_t buff[])
{
    countb[1]++;
    rateb[1] = 1000 * countb[1] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->stage_remain_time = buff2ui16(buff);
    boardcom->game_progress = buff[2];
    boardcom->shooter_heat_referee = buff2ui16(buff + 3);
    boardcom->last_update_time = HAL_GetTick();
}
