/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-02-29 01:54:12
 */

#include "protocol_board.h"

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

uint8_t BoardCom_TxData1[BOARDCOM_TX_LEN];
uint8_t BoardCom_TxData2[BOARDCOM_TX_LEN];
uint8_t BoardCom_RxData[BOARDCOM_RX_LEN];

FDCAN_HandleTypeDef *BOARD_CAN_HANDLER = &hfdcan2;

FDCAN_TxHeaderTypeDef BoardCom_GimControl;
FDCAN_TxHeaderTypeDef BoardCom_GimImuYaw;
FDCAN_TxHeaderTypeDef BoardCom_GimChassisRef;
FDCAN_TxHeaderTypeDef BoardCom_GimUIState;
FDCAN_TxHeaderTypeDef BoardCom_ChaRefereeData1;
FDCAN_TxHeaderTypeDef BoardCom_ChaRefereeData2;

static void _send_control(uint8_t buff[]);
static void _send_imu_yaw(uint8_t buff[]);
static void _send_chassis_ref(uint8_t buff[]);
static void _send_ui_state(uint8_t buff[]);
static void _set_referee_data_1(uint8_t buff[]);
static void _set_referee_data_2(uint8_t buff[]);
static void _set_control(uint8_t buff[]);
static void _set_imu_yaw(uint8_t buff[]);
static void _set_cha_ref(uint8_t buff[]);
static void _set_ui_state(uint8_t buff[]);

Board_TableEntryTypeDef Gim_Send1[BOARDCOM_GIMBAL_BUFF_SIZE_1] = {
    {SEND_GIMBAL_DATA_1, &_send_control},
    {SEND_GIMBAL_DATA_2, &_send_imu_yaw}};

Board_TableEntryTypeDef Gim_Send2[BOARDCOM_GIMBAL_BUFF_SIZE_2] = {
    {SEND_GIMBAL_DATA_3, &_send_chassis_ref},
    {SEND_GIMBAL_DATA_4, &_send_ui_state}};

Board_TableEntryTypeDef Cha_Receive[BOARDCOM_RECEIVE_SIZE] = {
    {0xff, NULL},
    {SET_REFEREE_DATA_1, &_set_referee_data_1},
    {SET_REFEREE_DATA_2, &_set_referee_data_2},
    {SET_CONTROL, &_set_control},
    {SET_IMU_YAW, &_set_imu_yaw},
    {SET_CHA_REF, &_set_cha_ref},
    {SET_UI_STATE, &_set_ui_state}};

/**
 * @brief      Inter bus communication initialization
 * @param      NULL
 * @retval     NULL
 */
void BoardCom_Init()
{
    BoardCom_Reset();
    FDCAN_InitTxHeader(&BoardCom_GimControl, SET_CONTROL, FDCAN_DLC_BYTES_8, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHeader(&BoardCom_GimImuYaw, SET_IMU_YAW, FDCAN_DLC_BYTES_8, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHeader(&BoardCom_GimChassisRef, SET_CHA_REF, FDCAN_DLC_BYTES_8, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHeader(&BoardCom_GimUIState, SET_UI_STATE, FDCAN_DLC_BYTES_8, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHeader(&BoardCom_ChaRefereeData1, SET_REFEREE_DATA_1, FDCAN_DLC_BYTES_8, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHeader(&BoardCom_ChaRefereeData2, SET_REFEREE_DATA_2, FDCAN_DLC_BYTES_8, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
}

/**
 * @brief      Reset inter bus communication data object
 * @param      NULL
 * @retval     NULL
 */
void BoardCom_Reset()
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->yaw_mode = 0;
    boardcom->yaw_ref = 0;
    boardcom->yaw_spd_fdb = 0.0f;
    boardcom->yaw_pos_fdb = 0.0f;
    boardcom->chassis_mode = 0;
    boardcom->chassis_fb_ref = 0.0f;
    boardcom->chassis_lr_ref = 0.0f;
    boardcom->cap_mode_user = SUPERCAP_CTRL_OFF;
    boardcom->power_limit_mode = POWER_LIMIT;
    //boardcom->cap_speedup_flag = CAP_NORMAL;
    boardcom->ui_cmd = 0;
    boardcom->cap_rest_energy_display = 0;
    boardcom->pitch_position = 0.0f;
    boardcom->magazine_state = 0;
    boardcom->shooter_state = 0;
    boardcom->auto_shoot_state = 0;
}

/**
 * @brief      Data sending function of serial port in inter bus communication
 * @param      NULL
 * @retval     NULL
 */
uint32_t can_free_level1 = 0;
uint32_t can_free_level2 = 0;
void BoardComPkg1_Send()
{
    BoardCom_Update();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->state = BOARDCOM_PENDING;
    for (int i = 0; i < BOARDCOM_GIMBAL_BUFF_SIZE_1; i++)
    {
        if (Gim_Send1[i].bus_func != NULL)
        {
			if (BOARD_CAN_HANDLER->ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL)
            {
                HAL_FDCAN_Stop(BOARD_CAN_HANDLER);
                BOARD_CAN_HANDLER->Instance->TXFQS = 0x03;
                HAL_FDCAN_Start(BOARD_CAN_HANDLER);
            }
            can_free_level1 = HAL_FDCAN_GetTxFifoFreeLevel(BOARD_CAN_HANDLER);
            Gim_Send1[i].bus_func(BoardCom_TxData1);
        }
    }
    boardcom->state = BOARDCOM_CONNECTED;
}

void BoardComPkg2_Send()
{
    BoardCom_Update();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->state = BOARDCOM_PENDING;
    for (int i = 0; i < BOARDCOM_GIMBAL_BUFF_SIZE_2; i++)
    {
        if (Gim_Send2[i].bus_func != NULL)
        {
			if (BOARD_CAN_HANDLER->ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL)
            {
                HAL_FDCAN_Stop(BOARD_CAN_HANDLER);
                BOARD_CAN_HANDLER->Instance->TXFQS = 0x03;
                HAL_FDCAN_Start(BOARD_CAN_HANDLER);
            }
            can_free_level2 = HAL_FDCAN_GetTxFifoFreeLevel(BOARD_CAN_HANDLER);
            Gim_Send2[i].bus_func(BoardCom_TxData1);
        }
    }
    boardcom->state = BOARDCOM_CONNECTED;
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
 * @brief      Assignment of inter bus communication structure
 * @param      NULL
 * @retval     NULL
 */
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
	boardcom->yaw_spd_fdb = ins->gyro[YAW];
    boardcom->yaw_pos_fdb = ins->yaw_consequent;
    boardcom->pitch_position = -ins->pitch;

	// for UI
    boardcom->magazine_state = Remote_Mag_State;
    boardcom->shooter_state = shooter->shooter_mode > 0;
	boardcom->auto_shoot_state = autoaim->AutoShootFlag;
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
        while (1)
        {
            ;
        }
    return 0;
}

/*************** SEND *****************/
int counta[7];
float ratea[7];

static void _send_control(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_PERIOD_CONTROL)
        return;
    last_send_time = HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &BoardCom_GimControl;
    counta[3]++;
    ratea[3] = 1000 * counta[3] / HAL_GetTick();
    memset(buff, 0, 8);

    buff[0] = (boardcom->yaw_mode << 4) + (boardcom->chassis_mode << 1) + boardcom->power_limit_mode;
    buff[1] = (0 << 4) + (boardcom->ui_cmd << 2) + (boardcom->cap_mode_user << 1);// + boardcom->cap_speedup_flag;
    float2buff(boardcom->yaw_ref, buff + 2);

    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_imu_yaw(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_PERIOD_IMU_YAW)
        return;
    last_send_time = HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &BoardCom_GimImuYaw;
    counta[4]++;
    ratea[4] = 1000 * counta[4] / HAL_GetTick();
    memset(buff, 0, 8);

    float2buff(boardcom->yaw_spd_fdb, buff);
    float2buff(boardcom->yaw_pos_fdb, buff + 4);

    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_chassis_ref(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_PERIOD_CHAREF)
        return;
    last_send_time = HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &BoardCom_GimChassisRef;
    counta[5]++;
    ratea[5] = 1000 * counta[5] / HAL_GetTick();
    memset(buff, 0, 8);

    i162buff((int16_t)(boardcom->chassis_fb_ref * 10), buff);
    i162buff((int16_t)(boardcom->chassis_lr_ref * 10), buff + 2);

    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

static void _send_ui_state(uint8_t buff[])
{
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= SEND_PERIOD_UI_STATE)
        return;
    last_send_time = HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    FDCAN_TxHeaderTypeDef *pheader = &BoardCom_GimUIState;
    counta[6]++;
    ratea[6] = 1000 * counta[6] / HAL_GetTick();
    memset(buff, 0, 8);

    i162buff((int16_t)(boardcom->pitch_position * 100), buff);
    buff[2] = ((boardcom->magazine_state & 0x01) << 7) + ((boardcom->shooter_state & 0x01) << 6) + 
				((boardcom->auto_shoot_state & 0x03) << 4) + (minipc->team_color & 0x0F);
// 留出通讯接口，需要时使用
//    buff[3] = boardcom->minipc_offset_vertical;
//    buff[4] = boardcom->minipc_offset_horizental;
    buff[5] = boardcom->ui_x;
    buff[6] = boardcom->ui_y;

    FDCAN_Send(BOARD_CAN_HANDLER, pheader, buff);
}

/*************** RECEIVE *****************/
int countb[8];
float rateb[8];
float bullet[5] = {28.0f, 28.0f, 28.0f, 28.0f, 28.0f};
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
    boardcom->heat_now = buff2ui16(buff + 6);
    boardcom->last_update_time[BOARDCOM_PKG_REFEREE1] = HAL_GetTick();
	if (boardcom->shoot_spd_referee > 20 && boardcom->shoot_spd_referee < 30)
	{
		bullet[0] = boardcom->shoot_spd_referee;
		bullet[1] = bullet[0];
		bullet[2] = bullet[1];
		bullet[3] = bullet[2];
		bullet[4] = bullet[3];
	}
}

static void _set_referee_data_2(uint8_t buff[])
{
    countb[BOARDCOM_PKG_REFEREE2]++;
    rateb[BOARDCOM_PKG_REFEREE2] = 1000 * countb[BOARDCOM_PKG_REFEREE2] / HAL_GetTick();

    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    boardcom->robot_id = buff[0] & 0x7F;
    boardcom->game_outpost_alive = (buff[0] & 0x80) >> 7;
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
    //boardcom->infantry_code = buff[1] >> 4;
    boardcom->ui_cmd = (buff[1] & 0x04) >> 2;
    boardcom->cap_mode_user = (buff[1] & 0x02) >> 1;
    //boardcom->cap_speedup_flag = buff[1] & 0x01;
    boardcom->yaw_ref = buff2float(buff + 2);
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

    boardcom->pitch_position = ((float)buff2i16(buff)) / 100;
    boardcom->magazine_state = (buff[2] & 0x80) >> 7;
    boardcom->shooter_state = (buff[2] & 0x40) >> 6;
    boardcom->auto_shoot_state = (buff[2] & 0x30) >> 4;
// 留出通讯接口，需要时使用
//    boardcom->minipc_target_id = (buff[2] & 0x0F);
//    boardcom->ui_x = (buff[5] + 1) * 5;
//    boardcom->ui_y = (buff[6]) * 5;
    boardcom->last_update_time[BOARDCOM_PKG_UI_STATE] = HAL_GetTick();
}
