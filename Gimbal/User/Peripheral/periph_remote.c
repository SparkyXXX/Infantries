/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-02-26 07:42:30
 */

#include "periph_remote.h"

Remote_DataTypeDef Remote_Data;
UART_HandleTypeDef* REMOTE_UART_HANDLER = &huart3;

/**
 * @brief      Remote control decoding function
 * @param      rc: The pointer points to the remote control data object
 * @param      buff: data buff
 * @param      rxdatalen: Data length
 * @retval     NULL
 */
void Remote_Decode(Remote_DataTypeDef *rc, uint8_t *buff, int rxdatalen)
{
    if (rxdatalen != REMOTE_RX_FRAME_LEN)
    {
        return;
    }
    rc->state = REMOTE_STATE_PENDING;
    rc->last_update_time = HAL_GetTick();

    rc->remote.ch[RIGHT_X] = Cancel_Offset(((uint16_t)buff[0] | (uint16_t)buff[1] << 8) & 0x07FF);
    rc->remote.ch[RIGHT_Y] = Cancel_Offset(((uint16_t)buff[1] >> 3 | (uint16_t)buff[2] << 5) & 0x07FF);
    rc->remote.ch[LEFT_X] = Cancel_Offset(((uint16_t)buff[2] >> 6 | (uint16_t)buff[3] << 2 | (uint16_t)buff[4] << 10) & 0x07FF);
    rc->remote.ch[LEFT_Y] = Cancel_Offset(((uint16_t)buff[4] >> 1 | (uint16_t)buff[5] << 7) & 0x07FF);
    rc->remote.ch[PADDLE_WHEEL] = Cancel_Offset(((uint16_t)buff[16] | (uint16_t)buff[17] << 8) & 0x07FF);
    rc->remote.s[SWITCH_LEFT] = Switch_ToState((buff[5] >> 6) & 0x03);
    rc->remote.s[SWITCH_RIGHT] = Switch_ToState((buff[5] >> 4) & 0x03);
    rc->mouse.x = ((int16_t)buff[6] | (int16_t)buff[7] << 8);
    rc->mouse.y = ((int16_t)buff[8] | (int16_t)buff[9] << 8);
    rc->mouse.z = ((int16_t)buff[10] | (int16_t)buff[11] << 8);
    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];
    KeyBoard_Decode(&(rc->key), ((int16_t)buff[14]) | ((int16_t)buff[15] << 8));
    Joy_AntiDrift(rc);

    if (rc->remote.ch[PADDLE_WHEEL] == -1024)
    {
        rc->remote.ch[PADDLE_WHEEL] = 0;
    }

    if (Remote_IsError(rc))
    {
        rc->state = REMOTE_STATE_ERROR;
        Remote_Reset(rc);
        return;
    }
    rc->state = REMOTE_STATE_CONNECTED;
//	Remote_Decode_Flag = 0;
}

/**
 * @brief      Remote control keyboard data decoding
 * @param      key: Remote control keyboard data object
 * @param      v: Original remote control keyboard data value
 * @retval     NULL
 */
void KeyBoard_Decode(Keyboard_DataTypeDef *key, uint16_t v)
{
    const uint16_t KEY_MASK_W = 1 << 0;
    const uint16_t KEY_MASK_S = 1 << 1;
    const uint16_t KEY_MASK_A = 1 << 2;
    const uint16_t KEY_MASK_D = 1 << 3;
    const uint16_t KEY_MASK_SHIFT = 1 << 4;
    const uint16_t KEY_MASK_CTRL = 1 << 5;
    const uint16_t KEY_MASK_Q = 1 << 6;
    const uint16_t KEY_MASK_E = 1 << 7;
    const uint16_t KEY_MASK_R = 1 << 8;
    const uint16_t KEY_MASK_F = 1 << 9;
    const uint16_t KEY_MASK_G = 1 << 10;
    const uint16_t KEY_MASK_Z = 1 << 11;
    const uint16_t KEY_MASK_X = 1 << 12;
    const uint16_t KEY_MASK_C = 1 << 13;
    const uint16_t KEY_MASK_V = 1 << 14;
    const uint16_t KEY_MASK_B = 1 << 15;

    key->w = (v & KEY_MASK_W) > 0;
    key->s = (v & KEY_MASK_S) > 0;
    key->a = (v & KEY_MASK_A) > 0;
    key->d = (v & KEY_MASK_D) > 0;
    key->shift = (v & KEY_MASK_SHIFT) > 0;
    key->ctrl = (v & KEY_MASK_CTRL) > 0;
    key->q = (v & KEY_MASK_Q) > 0;
    key->e = (v & KEY_MASK_E) > 0;
    key->r = (v & KEY_MASK_R) > 0;
    key->f = (v & KEY_MASK_F) > 0;
    key->g = (v & KEY_MASK_G) > 0;
    key->z = (v & KEY_MASK_Z) > 0;
    key->x = (v & KEY_MASK_X) > 0;
    key->c = (v & KEY_MASK_C) > 0;
    key->v = (v & KEY_MASK_V) > 0;
    key->b = (v & KEY_MASK_B) > 0;
}

/**
 * @brief      Gets the pointer of the remote control object
 * @param      NULL
 * @retval     Pointer to remote control object
 */
Remote_DataTypeDef *Remote_GetDataPtr()
{
    return &Remote_Data;
}

/**
 * @brief      Initialize remote control
 * @param      NULL
 * @retval     NULL
 */
void Remote_Init(UART_HandleTypeDef *huart)
{
    Remote_DataTypeDef *remote = Remote_GetDataPtr();
    remote->huart = huart;
    Remote_Reset(remote);
    Remote_Data.last_update_time = HAL_GetTick();
    UART_InitDMA(remote->huart);
    UART_ReceiveDMA(remote->huart, remote->rx_data, REMOTE_RX_BUFF_LEN);
}

/**
 * @brief      Initialize remote control data
 * @param      rc: Pointer to remote control object
 * @retval     NULL
 */
void Remote_Reset(Remote_DataTypeDef *rc)
{
    for (int i = 0; i < 5; ++i)
    {
        rc->remote.ch[i] = 0;
    }
    for (int i = 0; i < 2; ++i)
    {
        rc->remote.s[i] = Switch_ToState(0);
    }
    rc->mouse.x = 0;
    rc->mouse.y = 0;
    rc->mouse.z = 0;
    rc->mouse.l = 0;
    rc->mouse.r = 0;
    KeyBoard_Decode(&(rc->key), 0);
}

/**
 * @brief      Switch position state
 * @param      sw: Original switch value
 * @retval     Switch position status
 */
static Remote_SwitchStateEnum Switch_ToState(uint8_t sw)
{
    return (Remote_SwitchStateEnum)sw;
}

/**
 * @brief      Resist rocker zero drift
 * @param      rc: pointer to remote data
 * @retval     NULL
 */
static void Joy_AntiDrift(Remote_DataTypeDef *rc)
{
    if (abs(rc->remote.ch[LEFT_Y]) <= REMOTE_ANTI_DRIFT_VALUE)
    {
        rc->remote.ch[LEFT_Y] = 0;
    }
    if (abs(rc->remote.ch[LEFT_X]) <= REMOTE_ANTI_DRIFT_VALUE)
    {
        rc->remote.ch[LEFT_X] = 0;
    }
}

/**
 * @brief      Remove remote control offset
 * @param      ch: Original channel value
 * @retval     True value
 */
static int16_t Cancel_Offset(uint16_t ch)
{
    return (int16_t)ch - REMOTE_CHANNEL_VALUE_OFFSET;
}

/**
 * @brief      Judge whether the remote control is offline
 * @param      rc: pointer to remote control object
 * @retval     Offline or not (1 is yes, 0 is no)
 */
uint8_t Remote_IsLost(Remote_DataTypeDef *rc)
{
    uint32_t now = HAL_GetTick();
    if ((now - rc->last_update_time) > REMOTE_OFFLINE_TIME)
    {
        rc->state = REMOTE_STATE_LOST;
    }
    return rc->state == REMOTE_STATE_LOST;
}

/**
 * @brief      Judge whether the remote control data is wrong
 * @param      rc: pointer to remote control object
 * @retval     Error or not (1 is yes, 0 is no)
 */
uint8_t Remote_IsError(Remote_DataTypeDef *rc)
{
    for (int i = 0; i < 5; ++i)
    {
        if (abs(rc->remote.ch[i]) > REMOTE_CHANNEL_ERROR_LIMIT)
        {
            rc->state = REMOTE_STATE_ERROR;
        }
    }
    for (int i = 0; i < 2; ++i)
    {
        if (rc->remote.s[i] == REMOTE_SWITCH_NULL)
        {
            rc->state = REMOTE_STATE_ERROR;
        }
    }
    return rc->state == REMOTE_STATE_ERROR;
}
