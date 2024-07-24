/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-24 18:13:32
 */

#include "protocol_minipc.h"

MiniPC_DataTypeDef MiniPC_Data;
uint8_t MiniPC_TxData[32];
GPIO_HandleTypeDef TRIGGER = {GPIO_PIN_RESET, GPIOB, GPIO_PIN_15};

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
    minipc->is_get_target = 0;
    minipc->is_shoot = 0;
    minipc->buff_yaw_ref = 0;
    minipc->buff_pitch_ref = 0;
    minipc->timestamp = 0;
    minipc->armor_yaw_ref = 0;
    minipc->armor_pitch_ref = 0;
    minipc->state = MINIPC_NULL;
    minipc->recieve_packet_type = 0;
    minipc->armor_trigger_ms = 1000 / ARMOR_CAMERA_FPS - ARMOR_SEND_DELAY - MINIPC_TASK_DELAY;
    minipc->buff_trigger_ms = 1000 / BUFF_CAMERA_FPS - BUFF_SEND_DELAY - MINIPC_TASK_DELAY;
    MiniPC_CheckID();
    minipc->last_update_time = HAL_GetTick();
}

void MiniPC_CheckID()
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
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

void MiniPC_Send()
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    AutoAim_ControlTypeDef *autoaim = AutoAim_GetControlPtr();
    BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
    INS_DataTypeDef *ins = INS_GetControlPtr();
    Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
    MiniPC_CheckID();
    GPIO_Set(&TRIGGER);
    if (autoaim->hit_mode == AUTOAIM_HIT_ARMOR)
    {
        osDelay(ARMOR_SEND_DELAY);
    }
    else if (autoaim->hit_mode == AUTOAIM_HIT_BUFF)
    {
        osDelay(BUFF_SEND_DELAY);
    }
    minipc->yaw_send_to_minipc = ins->yaw;
    minipc->pitch_send_to_minipc = -ins->pitch * 100;
    minipc->roll_send_to_minipc = -ins->roll * 100;
    if (shooter->shoot_speed.average_bullet_speed > 20 && shooter->shoot_speed.average_bullet_speed < 30)
    {
        minipc->bullet_speed_send_to_minipc = shooter->shoot_speed.average_bullet_speed * 100;
    }
    else
    {
        minipc->bullet_speed_send_to_minipc = 28 * 100;
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
    buff[8] = 0;
    float2buff(minipc->yaw_send_to_minipc, buff + 9);
    i162buff(minipc->pitch_send_to_minipc, buff + 13);
    i162buff(minipc->roll_send_to_minipc, buff + 15);
    buff[17] = 0;
    buff[18] = 0;
    ui162buff(minipc->bullet_speed_send_to_minipc, buff + 19);
    buff[21] = 0;
    buff[22] = 0;
    buff[23] = minipc->team_color;
    buff[24] = minipc->mode;
    ui322buff(MiniPC_FrameTime, buff + 25);
    buff[29] = autoaim->is_change_target;
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
        osDelay(minipc->armor_trigger_ms);
    }
    else if (autoaim->hit_mode == AUTOAIM_HIT_BUFF)
    {
        osDelay(minipc->buff_trigger_ms);
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

static void MiniPC_ArmorPacketDecode(void *buff, uint16_t rxdatalen)
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    AutoAim_ArmorPacketDataTypeDef *data_ptr = buff + 8;
    minipc->recieve_packet_type = MINIPC_ARMOR_PACKET;
    minipc->is_get_target = data_ptr->is_get;
    minipc->is_shoot = data_ptr->is_shoot;
    minipc->timestamp = data_ptr->timestamp;
    minipc->armor_yaw_ref = data_ptr->yaw_ref;
    minipc->armor_pitch_ref = data_ptr->pitch_ref;
    minipc->armor_yaw_spd_ref = data_ptr->yaw_spd;
    minipc->state = MINIPC_CONNECTED;
}

static void MiniPC_BuffPacketDecode(uint8_t *buff, uint16_t rxdatalen)
{
    MiniPC_DataTypeDef *minipc = MiniPC_GetDataPtr();
    minipc->recieve_packet_type = MINIPC_BUFF_PACKET;
    minipc->is_get_target = buff[8];
    minipc->buff_yaw_ref = buff2float(buff + 9);
    minipc->buff_pitch_ref = (float)buff2i16(buff + 13);
    minipc->is_shoot = buff[17];
    minipc->state = MINIPC_CONNECTED;
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
