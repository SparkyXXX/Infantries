/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-14 19:42:53
 */

#include "periph_motor_can.h"

void Motor_Init(Motor_DataTypeDef *pmotor, uint32_t stdid)
{
    if (pmotor == NULL) 
    {
        return;
    }
    pmotor->state = MOTOR_CONNECTED;
    pmotor->stdid = stdid;
    pmotor->output = 0;    
    pmotor->encoder.last_update_time = HAL_GetTick();
}

void Motor_InitGroup(Motor_GroupDataTypeDef *pgroup, uint8_t motor_num, FDCAN_HandleTypeDef *phfdcan, uint16_t stdid)
{
    if (pgroup == NULL || phfdcan == NULL) 
    {
        return;
    }
    pgroup->motor_num = motor_num;
    pgroup->can_handle = phfdcan;
    FDCAN_InitTxHander(&(pgroup->can_header), stdid, FDCAN_DLC_BYTES_8, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    
    for (int i = 0; i < motor_num; i++) 
    {
        pgroup->motor_handle[i] = NULL;
    }
}

void Motor_SetOutput(Motor_DataTypeDef *pmotor, float output)
{
    if (pmotor == NULL || pmotor->state == MOTOR_LOST) 
    {
				pmotor->output = 0;
        return;
    }
    pmotor->output = output;
}

float Motor_GetOutput(Motor_DataTypeDef *pmotor)
{
    return pmotor->output;
}

uint8_t Motor_IsLost(Motor_DataTypeDef *pmotor)
{
//    if (pmotor == NULL || pmotor->state == MOTOR_CONNECTED) 
//    {
//        return 0;
//    }
    uint32_t now = HAL_GetTick();
    return (now - pmotor->encoder.last_update_time) > MOTOR_OFFLINE_TIME;
}
uint8_t Motor_IsLostEncoderUpdate(Motor_DataTypeDef *pmotor)
{
    uint32_t now = HAL_GetTick();
		if(pmotor->encoder.last_current != pmotor->encoder.current){
			pmotor->encoder.last_current_update_time = now;
		}
		pmotor->encoder.last_current = pmotor->encoder.current;
		
    return (now - pmotor->encoder.last_current_update_time) > MOTOR_OFFLINE_TIME;
}
