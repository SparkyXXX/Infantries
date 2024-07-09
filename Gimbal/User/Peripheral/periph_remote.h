/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-15 00:45:13
 */

#ifndef PERIPH_REMOTE_H
#define PERIPH_REMOTE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stdlib.h"
#include "util_uart.h"

#define RIGHT_X 0
#define RIGHT_Y 1
#define LEFT_X 2
#define LEFT_Y 3
#define PADDLE_WHEEL 4
#define SWITCH_LEFT 0
#define SWITCH_RIGHT 1

#define REMOTE_ANTI_DRIFT_VALUE 1
#define REMOTE_CHANNEL_VALUE_OFFSET 1024
#define REMOTE_CHANNEL_ERROR_LIMIT 700
#define REMOTE_OFFLINE_TIME 1000
#define REMOTE_RX_FRAME_LEN 18
#define REMOTE_RX_BUFF_LEN 54

    typedef enum
    {
        REMOTE_STATE_NULL = 0,
        REMOTE_STATE_CONNECTED = 1,
        REMOTE_STATE_LOST = 2,
        REMOTE_STATE_ERROR = 3,
        REMOTE_STATE_PENDING = 4
    } Remote_StateEnum;

    typedef enum
    {
        REMOTE_SWITCH_NULL = 0,
        REMOTE_SWITCH_UP = 1,
        REMOTE_SWITCH_DOWN = 2,
        REMOTE_SWITCH_MIDDLE = 3
    } Remote_SwitchStateEnum;

    typedef struct
    {
        uint8_t w, a, s, d, shift, ctrl, space, q, e, r, f, g, z, x, c, v, b;
    } Keyboard_DataTypeDef;

    typedef struct
    {
        Remote_StateEnum state;
        UART_HandleTypeDef *huart;

        struct
        {
            int16_t ch[5];
            Remote_SwitchStateEnum s[2];
        } remote;

        struct
        {
            int16_t x; // mouse x
            int16_t y; // mouse y
            int16_t z; // mouse z
            uint8_t l; // mouse Left key
            uint8_t r; // mouse Right key
        } mouse;

        Keyboard_DataTypeDef key;
		
        uint8_t rx_data[REMOTE_RX_BUFF_LEN];
        uint32_t last_update_time;
    } Remote_DataTypeDef;
	
	extern uint8_t Remote_Decode_Flag;
	extern UART_HandleTypeDef* REMOTE_UART_HANDLER;

    void Remote_Decode(Remote_DataTypeDef *rc, uint8_t *buff, int rxdatalen);
    void KeyBoard_Decode(Keyboard_DataTypeDef *key, uint16_t v);

    Remote_DataTypeDef *Remote_GetDataPtr(void);
    void Remote_Init(UART_HandleTypeDef *huart);
    void Remote_Reset(Remote_DataTypeDef *rc);
    static Remote_SwitchStateEnum Switch_ToState(uint8_t sw);
    static void Joy_AntiDrift(Remote_DataTypeDef *rc);
    static int16_t Cancel_Offset(uint16_t ch);
    uint8_t Remote_IsLost(Remote_DataTypeDef *rc);
    uint8_t Remote_IsError(Remote_DataTypeDef *rc);

#ifdef __cplusplus
}
#endif

#endif
