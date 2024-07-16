/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Chen Zhihong
 * @LastEditTime: 2024-07-16 02:22:50
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
    FDCAN_InitTxHeader(&(pgroup->can_header), stdid);

    for (int i = 0; i < motor_num; i++)
    {
        pgroup->motor_handle[i] = NULL;
    }
}

void Motor_SetOutput(Motor_DataTypeDef *pmotor, float output)
{
    if (pmotor == NULL)
    {
        pmotor->output = 0;
        return;
    }
    else if (pmotor->state == MOTOR_LOST)
    {
        pmotor->output = 0;
    }
    else if (pmotor->state == MOTOR_CONNECTED)
    {
        pmotor->output = output;
    }
}

float Motor_GetOutput(Motor_DataTypeDef *pmotor)
{
    return pmotor->output;
}

void Motor_IsLost(Motor_DataTypeDef *pmotor)
{
    if (pmotor == NULL)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    pmotor->state = ((now - pmotor->encoder.last_update_time) > MOTOR_OFFLINE_TIME);
}
