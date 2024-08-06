/*
 * @Author: Chen Zhihong
 * @LastEditors: Chen Zhihong
 */

#ifndef EXT_REMOTE_DEV_H
#define EXT_REMOTE_DEV_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "callback_ctrl.h"
#include "stdint.h"
#include "string.h"
#include "util_uart.h"

#define REFEREE_TX_BUFF_LEN 300
#define REFEREE_RX_BUFF_LEN 500
#define REFEREE_OFFLINE_TIME 1000

    typedef enum
    {
        REFEREE_NULL = 0,
        REFEREE_CONNECTED = 1,
        REFEREE_LOST = 2,
        REFEREE_ERROR = 3,
        REFEREE_PENDING = 4
    } Referee_RefereeStateEnum;

    typedef struct
    {
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        int8_t left_button_down;
        int8_t right_button_down;
        uint16_t keyboard_value;
        uint16_t reserved;
    } ext_remote_control_t;
    typedef struct __attribute__((packed))
    {
        uint16_t data_cmd_id;
        uint16_t sender_ID;
        uint16_t receiver_ID;
    } ext_student_interactive_header_data_t;

    typedef struct
    {
        uint16_t robot_id;
        uint16_t client_id;
    } Referee_RobotAndClientIDTypeDef;
    typedef struct
    {
        UART_HandleTypeDef *huart;
        Referee_RefereeStateEnum state; // 裁判系统当前状态
        uint32_t last_update_time;      // 裁判系统上次更新时间
        uint16_t client_id;             // 客户端ID
        ext_remote_control_t remote_vtm_data;
    } Referee_DataTypeDef;
    typedef uint8_t (*Referee_RefereeCmdParseFuncDef)(Referee_DataTypeDef *referee, void *data_ptr);
    typedef struct
    {
        uint16_t cmd_id;
        uint8_t data_length;
        Referee_RefereeCmdParseFuncDef parse_func;
    } Referee_RefereeCmdTypeDef;

    Referee_DataTypeDef *Referee_GetDataPtr(void);
    void Referee_Init(UART_HandleTypeDef *huart);
    void Referee_Reset();
    Referee_DataTypeDef *Referee_GetDataPtr(void);
    // void Referee_Reset(void);
    void Referee_Decode(uint8_t *buff, uint16_t rxdatalen);
    uint8_t Referee_IsLost(void);
    uint16_t Referee_GetClientIDByRobotID(uint8_t robot_id);
    uint8_t Referee_ParseRefereeCmd(uint16_t cmd_id, uint8_t *data, uint16_t data_length);

#endif
#ifdef __cplusplus
}
#endif
