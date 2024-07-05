/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-04 00:24:46
 */

#include "periph_minipc.h"

MiniPC_DataTypeDef MiniPC_Data;

uint8_t MiniPC_TxData[32];
uint16_t bullet_speed;
extern float bullet[5];
GPIO_HandleTypeDef TRIGGER = {GPIO_PIN_RESET, GPIOB, GPIO_PIN_15};
void MiniPC_Send()
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    INS_DataTypeDef *ins = INS_GetControlPtr();

    GPIO_Set(&TRIGGER);
	if (autoaim->hit_mode == AUTOAIM_HIT_ARMOR)
	{
		osDelay(1);
	}
	else if (autoaim->hit_mode == AUTOAIM_HIT_BUFF)
	{
		osDelay(4);
	}
    float yaw = ins->yaw;
    int16_t pitch = -ins->pitch * 100;
    int16_t roll = -ins->roll * 100;
    int16_t pitch_speed = -ins->gyro[PITCH] * 100;
	if (boardcom->shoot_spd_referee > 20 && boardcom->shoot_spd_referee < 30)
	{
		bullet_speed = (bullet[0] * 100 + bullet[1] * 100 + bullet[2] * 100 + bullet[3] * 100 + bullet[4] * 100) / 5;
	}
	else
	{
		bullet_speed = 28 * 100;
	}

    uint32_t MiniPC_FrameTime = HAL_GetTick() % 60000;
    minipc->state = MINIPC_PENDING;

    uint8_t *buff = MiniPC_TxData;
    buff[0] = MINIPC_PACKET_HEADR_0;
    buff[1] = MINIPC_PACKET_HEADR_1;
    buff[2] = MINIPC_SLAVE_COMPUTER;
    buff[3] = minipc->address;
    buff[4] = MINIPC_DATA_PACKET;
    buff[5] = MINIPC_DATA_PACKET_LEN;
    buff[6] = 0;
    buff[7] = 0;
    buff[8] = boardcom->game_outpost_alive;
    float2buff(yaw, buff + 9);
    i162buff(pitch, buff + 13);
    i162buff(roll, buff + 15);
    i162buff(0 * 100, buff + 17);
    ui162buff(bullet_speed, buff + 19);
    i162buff(0 * 100, buff + 21);
    buff[23] = minipc->team_color;
    buff[24] = minipc->mode;
    ui322buff(MiniPC_FrameTime, buff + 25);
    buff[29] = autoaim->isChangeTarget;
    buff[30] = 0;
    buff[31] = 0;

    uint16_t checksum = 0;
    for (int i = 0; i < MINIPC_TXDATA_LEN; i += 2)
    {
        checksum += buff2ui16(buff + i);
    }

    ui162buff(checksum, buff + 6);
    CDC_Transmit_FS(buff, MINIPC_TXDATA_LEN);
    GPIO_Reset(&TRIGGER);
	if (autoaim->hit_mode == AUTOAIM_HIT_ARMOR)
	{
		osDelay(6);
	}
	else if (autoaim->hit_mode == AUTOAIM_HIT_BUFF)
	{
		osDelay(20);
	}
}

void MiniPC_Decode(uint8_t *buff, uint16_t rxdatalen)
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    minipc->last_update_time = HAL_GetTick();

    if (!MiniPC_Verify(buff, rxdatalen))
    {
        minipc->state = MINIPC_ERROR;
        return;
    }

    switch (buff[4])
    {
    case MINIPC_ARMOR_PACKET:
        MiniPC_ArmorPacketDecode(buff, rxdatalen);
        break;
    case MINIPC_BUFF_PACKET:
        MiniPC_BuffPacketDecode(buff, rxdatalen);
        break;
    default:
        break;
    }
}

AutoAim_ArmorPacketDataTypeDef MiniPC_ArmorPacket_Watch;
static void MiniPC_ArmorPacketDecode(void *buff, uint16_t rxdatalen)
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    AutoAim_ArmorPacketDataTypeDef *data_ptr = buff + 8;
    MiniPC_ArmorPacket_Watch = *data_ptr;
    minipc->new_data_flag = 1;
    minipc->is_get_target = data_ptr->is_get;
	minipc->is_shoot = data_ptr->is_shoot;
    minipc->timestamp = data_ptr->timestamp;
	minipc->yaw_ref = data_ptr->yaw_ref;
	minipc->pitch_ref = data_ptr->pitch_ref;
    minipc->state = MINIPC_CONNECTED;
}

static void MiniPC_BuffPacketDecode(uint8_t *buff, uint16_t rxdatalen)
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    minipc->new_data_flag = 1;
    minipc->is_get_target = buff[8];
    minipc->buff_yaw = buff2float(buff + 9);
    minipc->buff_pitch = (float)buff2i16(buff + 13);
    minipc->buff_distance = (float)buff2i16(buff + 15);
    minipc->is_shoot = buff[17];
    minipc->state = MINIPC_CONNECTED;
}

/**
 * @brief      Get pointer to minipc data object
 * @param      NULL
 * @retval     MiniPC Pointer to data object
 */
MiniPC_DataTypeDef *MiniPC_GetDataPtr()
{
    return &MiniPC_Data;
}

/**
 * @brief      Initialize minipc
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_Init()
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    minipc->team_color = 0;
    minipc->is_get_target = 0;
	minipc->is_shoot = 0;
    minipc->buff_yaw = 0;
    minipc->buff_pitch = 0;
    minipc->buff_distance = 0;
    minipc->timestamp = 0;
	minipc->yaw_ref = 0;
	minipc->pitch_ref = 0;
    minipc->address = 0;
    minipc->state = MINIPC_NULL;
    minipc->new_data_flag = 0;
    minipc->last_update_time = HAL_GetTick();
}

void MiniPC_Update(void)
{
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    switch (boardcom->robot_id)
    {
	case 3:
		minipc->team_color = MINIPC_COLOR_RED;
        minipc->address = MINIPC_INFANTRY_3;
    case 4:
		minipc->team_color = MINIPC_COLOR_RED;
        minipc->address = MINIPC_INFANTRY_4;
	case 5:
        minipc->team_color = MINIPC_COLOR_RED;
        minipc->address = MINIPC_INFANTRY_5;
        break;
    case 103:
		minipc->team_color = MINIPC_COLOR_BLUE;
        minipc->address = MINIPC_INFANTRY_3;
	case 104:
		minipc->team_color = MINIPC_COLOR_BLUE;
        minipc->address = MINIPC_INFANTRY_4;
	case 105:
        minipc->team_color = MINIPC_COLOR_BLUE;
        minipc->address = MINIPC_INFANTRY_5;
        break;
    default:
        break;
    }
}

/**
 * @brief      Minipc data decoding function
 * @param      buff: Data buffer
 * @param      rxdatalen: recevie data length
 * @retval     Match is 1  not match is 0
 */
static uint16_t checksum__ = 0, sum__ = 0;
static uint8_t MiniPC_Verify(uint8_t *buff, uint16_t rxdatalen)
{
    if (rxdatalen <= 8)
    {
        return 0;
    }
    if (buff[0] != MINIPC_PACKET_HEADR_0 || buff[1] != MINIPC_PACKET_HEADR_1)
    {
        return 0;
    }

    uint16_t checksum = 0, sum = 0;
    int size = rxdatalen;
    sum = buff2ui16(buff + 6);

    if (size % 2)
    {
        buff[size] = 0x00;
        size++;
    }
    for (int i = 0; i < size; i += 2)
    {
        if (i == 6)
        {
            continue;
        }
        checksum += buff2ui16(buff + i);
    }
    checksum__ = checksum;
    sum__ = sum;

    return checksum == sum;
}

/**
 * @brief      Determine whether minipc is lost
 * @param      NULL
 * @retval     Offline or not (1 is yes, 0 is no)
 */
uint8_t MiniPC_IsLost()
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    uint32_t now = HAL_GetTick();
    return (now - minipc->last_update_time) > 100;
}

