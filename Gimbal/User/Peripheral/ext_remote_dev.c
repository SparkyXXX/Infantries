/**
 * DreamChaser Frame Source File
 * 
 * File:        ext_remote_dev.c
 * Brief:       This document contains the data receiving and sending of the referee system in referee_dev.c
 * Author:      Qu Jiuhe
 * Modified:    2023/5/7      11:37
 *
 */
#include "ext_remote_dev.h"
UART_HandleTypeDef* Const_ext_Remote_UART_HANDLER ;
const uint16_t Const_ext_Remote_RX_BUFF_LEN         = 60;
uint8_t ext_Remote_RxData[60];
ext_Remote_Command_t ext_Remote_Command;
const uint8_t ext_Remote_PARSE_FAILED = 0, ext_Remote_PARSE_SUCCEEDED = 1;
const Referee_RefereeCmdTypeDef Const_ext_Remote_cmd       = {0x0304,    12, NULL};    

/**
  * @brief      解码函数
  * @param      
  * @retval     
  */
uint8_t P_ext_Remote_cmd(ext_Remote_Command_t* ext_rc, void *data_ptr) {
	
		ext_robot_command_t *struct_ptr = data_ptr;
		//memcpy(&(ext_Remote_Command.Remote_Command),struct_ptr,sizeof(ext_Remote_Command.Remote_Command));
		ext_Remote_Command.Remote_Command.mouse_x = struct_ptr->mouse_x;
		ext_Remote_Command.Remote_Command.mouse_y = struct_ptr->mouse_y;
		ext_Remote_Command.Remote_Command.mouse_z = struct_ptr->mouse_z;
		ext_Remote_Command.Remote_Command.left_button_down = struct_ptr->left_button_down;
		ext_Remote_Command.Remote_Command.right_button_down = struct_ptr->right_button_down;
		ext_Remote_Command.Remote_Command.keyboard_value = struct_ptr->keyboard_value;
		
//	if(ext_rc->last_update_time - ext_rc->last_cnt_freq_log_time > 1000){
//		ext_rc->last_cnt_freq_log_time = ext_rc->last_update_time;
//		ext_rc->rx_comm_freq = ext_rc->rx_cnt;
//		ext_rc->rx_cnt=0;
//	}
//	ext_rc->rx_cnt++;
	  return ext_Remote_PARSE_SUCCEEDED;
}
/**
  * @brief      謽謨陆訆盏膴媒戮輮  * @param      
  * @retval     
  */
void ExtRemote_ResetExtRemoteData() {
    ext_Remote_Command_t* ext_rc = &ext_Remote_Command;
    memset(&(ext_rc->Remote_Command), 0, sizeof(ext_rc->Remote_Command));
}
/**
  * @brief      初始化
  * @param      
  * @retval     
  */
void ext_Remote_Init(UART_HandleTypeDef* huart) {
	
    ext_Remote_Command_t* ext_rc = &ext_Remote_Command;
    ExtRemote_ResetExtRemoteData();
    ext_rc->last_update_time = HAL_GetTick();
		ext_rc->rxdata = ext_Remote_RxData;
    UART_InitDMA(huart);
    UART_ReceiveDMA(huart, ext_Remote_RxData, Const_ext_Remote_RX_BUFF_LEN);
}

/**
  * @brief      判断图传串口是否断连
  * @param      
  * @retval     (1断连,0正常)
  */
uint8_t extRemote_IsextRemoteOffline() {
    ext_Remote_Command_t* ext_rc = &ext_Remote_Command;
    uint32_t now = HAL_GetTick();
    if ((now - ext_rc->last_update_time) > 500)//超过500ms没收到消息就判定为断连
        ext_rc->state = LOST_extRemote;
    return ext_rc->state == LOST_extRemote;
}
/**
  * @brief      判断id确定是否为键鼠遥控信息并调用解码函数
  * @param      
  * @retval     (0id正确,1未知id)
  */
uint8_t extRC_ParseExtRemoteCmd(uint16_t cmd_id, uint8_t* data, uint16_t data_length) {
    ext_Remote_Command_t* ext_rc = &ext_Remote_Command;
		
    if (cmd_id == Const_ext_Remote_cmd.cmd_id) return P_ext_Remote_cmd(ext_rc,data);
    else return ext_Remote_PARSE_FAILED;    // unknown cmd
}

/**
  * @brief      rx验证并解码
  * @param      buff
  * @param      rxdatalen
  * @retval     
  */
void extRC_DecodeExtRemoteData(uint8_t* buff, uint16_t rxdaalen) {
    ext_Remote_Command_t* ext_rc = &ext_Remote_Command;
    ext_rc->last_update_time   = HAL_GetTick();
    
    ext_rc->state              = PENDING_extRemote;
    if (buff[0] != 0xA5) {
        ext_rc->state          = ERROR_extRemote;
        return;
    }
    if (!CRC_VerifyCRC8CheckSum(buff, 5)) {
        ext_rc->state          = ERROR_extRemote;
        return;
    }
    uint16_t data_length = (uint16_t) buff[2] << 8 | buff[1];
    uint8_t seq = buff[3];
    if (seq == 0) {
        ext_rc->state          = ERROR_extRemote;
        //return;
    }
    if (!CRC_VerifyCRC16CheckSum(buff, data_length + 9)) {
        ext_rc->state          = ERROR_extRemote;
        return;
    }
    uint16_t cmd_id = (uint16_t) buff[6] << 8 | buff[5];
    if (!extRC_ParseExtRemoteCmd(cmd_id, buff + 7, data_length)) {
        ext_rc->state          = ERROR_extRemote;
        return;
    }
    ext_rc->state              = CONNECTED_extRemote;  // ??
}

void Use_extRemote(Remote_DataTypeDef* remote){
	ext_Remote_Command_t* ext_rc = &ext_Remote_Command;
	remote->remote.s[SWITCH_RIGHT] = REMOTE_SWITCH_MIDDLE;
	remote->mouse.x = ext_rc->Remote_Command.mouse_x;
	remote->mouse.y = ext_rc->Remote_Command.mouse_y;
	remote->mouse.z = ext_rc->Remote_Command.mouse_z;
	remote->mouse.l = ext_rc->Remote_Command.left_button_down;
	remote->mouse.r = ext_rc->Remote_Command.right_button_down;
	KeyBoard_Decode(&(remote->key), ext_rc->Remote_Command.keyboard_value);
}

/************************ (C) COPYRIGHT BIT DreamChaser *****END OF FILE****/
