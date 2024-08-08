/**
 * DreamChaser Frame Source File
 * 
 * File:        ext_referee_dev.h
 * Brief:       
 * Author:      Qu Jiuhe
 * Modified:    2023/5/7      11:37
 *
 */
#ifndef EXT_REMOTE_DEV_H
#define EXT_REMOTE_DEV_H
#ifdef __cplusplus
extern "C" {
#endif

#include "util_uart.h"
#include "lib_crc.h"
#include "protocol_referee.h"
#include "app_remote.h"

typedef enum {
    NULL_extRemote      = 0,
    CONNECTED_extRemote = 1,
    LOST_extRemote      = 2,
    ERROR_extRemote     = 3,
    PENDING_extRemote   = 4
} extRemote_extRemoteStateEnum;
typedef struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} ext_robot_command_t;
typedef struct{
	extRemote_extRemoteStateEnum state;
	ext_robot_command_t Remote_Command;
	uint8_t *rxdata;
	uint32_t last_update_time;
	//uint32_t last_cnt_freq_log_time;	uint16_t rx_comm_freq,rx_cnt;//接收图传遥控信息频率
}ext_Remote_Command_t;//图传遥控信息


extern UART_HandleTypeDef* Const_ext_Remote_UART_HANDLER ;
extern ext_Remote_Command_t ext_Remote_Command;//相关数据和状态总结构体
extern const uint16_t Const_ext_Remote_RX_BUFF_LEN;

void ext_Remote_Init(UART_HandleTypeDef* huart);//初始化函数
void extRC_DecodeExtRemoteData(uint8_t* buff, uint16_t rxdaalen);
uint8_t extRemote_IsextRemoteOffline();//判断图传串口是否断连
void Use_extRemote(Remote_DataTypeDef* remote);

#endif
#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT BIT DreamChaser *****END OF FILE****/
