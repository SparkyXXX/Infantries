/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 00:40:52
 */

#ifndef PERIPH_MINIPC_H
#define PERIPH_MINIPC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_shoot.h"
#include "protocol_board.h"
#include "usbd_cdc_if.h"

#define MINIPC_TASK_DELAY 1
#define ARMOR_SEND_DELAY 1
#define BUFF_SEND_DELAY 4
#define ARMOR_CAMERA_FPS 100
#define BUFF_CAMERA_FPS 50

#define MINIPC_TXDATA_LEN 32
#define MINIPC_DATA_PACKET 0x08
#define MINIPC_DATA_PACKET_LEN 24

#define MINIPC_SLAVE_COMPUTER 0x00
#define MINIPC_PACKET_HEADR_0 0xa5
#define MINIPC_PACKET_HEADR_1 0X5a

#define MINIPC_ARMOR_PACKET 0x02
#define MINIPC_BUFF_PACKET 0x09

#define MINIPC_COLOR_RED 0x00
#define MINIPC_COLOR_BLUE 0x01

#define MINIPC_INFANTRY_3 0x03
#define MINIPC_INFANTRY_4 0x04
#define MINIPC_INFANTRY_5 0x05

#define PITCH 0
#define ROLL 1
#define YAW 2

    typedef enum
    {
        MINIPC_NULL = 0,
        MINIPC_CONNECTED = 1,
        MINIPC_LOST = 2,
        MINIPC_ERROR = 3,
        MINIPC_PENDING = 4
    } MiniPC_StateEnum;

    typedef struct
    {
        MiniPC_StateEnum state;
        uint32_t last_update_time;
        uint16_t armor_trigger_ms;
        uint16_t buff_trigger_ms;

        // Gimbal Send to MiniPC
        uint8_t address;
        uint8_t team_color;
        uint8_t mode;
        float yaw_send_to_minipc;
        int16_t pitch_send_to_minipc;
        int16_t roll_send_to_minipc;
        float bullet_speed_send_to_minipc;

        //  Gimbal Receive from MiniPC
        uint8_t recieve_packet_type;
        uint8_t is_get_target;
        uint8_t is_shoot;
        uint16_t timestamp;

        int16_t armor_yaw_ref;
        int16_t armor_pitch_ref;
        float armor_yaw_spd_ref;
        float buff_yaw_ref;
        float buff_pitch_ref;
    } MiniPC_DataTypeDef;

    typedef struct __attribute__((packed))
    {
        uint16_t timestamp;
        uint16_t is_get;
        uint16_t is_shoot;
        int16_t yaw_ref;
        int16_t pitch_ref;
        int16_t null0, null1, yaw_spd, null3, null4, null5, null6;
    } AutoAim_ArmorPacketDataTypeDef;

    MiniPC_DataTypeDef *MiniPC_GetDataPtr(void);
    void MiniPC_Init(void);
    void MiniPC_Send(void);
    void MiniPC_Decode(uint8_t *buff, uint16_t rxdatalen);
    static void MiniPC_ArmorPacketDecode(void *buff, uint16_t rxdatalen);
    static void MiniPC_BuffPacketDecode(uint8_t *buff, uint16_t rxdatalen);
    static uint8_t MiniPC_Verify(uint8_t *buff, uint16_t rxdatalen);
    uint8_t MiniPC_IsLost(void);

#ifdef __cplusplus
}
#endif

#endif
