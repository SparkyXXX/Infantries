/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 21:37:43
 */

#include "periph_motor_can.h"
#include "config_ctrl.h"

void GM6020_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata)
{
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.angle = (float)((int16_t)((uint16_t)rxdata[0] << 8 | (uint16_t)rxdata[1]));
    pmotor->encoder.speed = (float)((int16_t)((uint16_t)rxdata[2] << 8 | (uint16_t)rxdata[3]));
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxdata[4] << 8 | (uint16_t)rxdata[5]));
    pmotor->encoder.temp = rxdata[6];

    // Calculate angle difference and number of cycles
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle; // The increase of mechanical angle is positive
    if (diff < -5500)
    {
        pmotor->encoder.round_count++;
    }
    else if (diff > 5500)
    {
        pmotor->encoder.round_count--;
    }

    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f +
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->encoder.limited_angle = (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    // For yaw axis processing, the small gyroscope is rotated to the same position as the PTZ according to the nearest distance after stopping
    pmotor->encoder.last_update_time = HAL_GetTick();
}

/**
 * @brief      Feeder motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
float angle_diff, last_consequent_angle;
void M2006_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata)
{
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.angle = (float)((int16_t)((uint16_t)rxdata[0] << 8 | (uint16_t)rxdata[1]));
    pmotor->encoder.speed = (float)(((int16_t)((uint16_t)rxdata[2] << 8 | (uint16_t)rxdata[3]))) / 36.0f;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxdata[4] << 8 | (uint16_t)rxdata[5]));
    pmotor->encoder.temp = rxdata[6];
    // Calculate the angle difference and the number of turns
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;
    if (diff < -5500)
    {
        pmotor->encoder.round_count++;
    }
    else if (diff > 5500)
    {
        pmotor->encoder.round_count--;
    }
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    angle_diff = Motor_feederMotor.encoder.consequent_angle - last_consequent_angle;
    last_consequent_angle = Motor_feederMotor.encoder.consequent_angle;
    pmotor->encoder.last_update_time = HAL_GetTick();
}

void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[])
{
    if (phfdcan == &hfdcan1)
    {
        if (stdid == PITCH_CAN_ID)
        {
            GM6020_Decode(&Motor_gimbalMotorPitch, rxdata);
        }
        if (stdid == FEEDER_CAN_ID)
        {
            M2006_Decode(&Motor_feederMotor, rxdata);
        }
    }
}

void Motor_CAN_SendGroupOutput(Motor_GroupDataTypeDef *pgroup)
{
    if (pgroup == NULL)
    {
        return;
    }
    uint8_t txdata[8];
    txdata[0] = (uint8_t)((int16_t)Motor_GetOutput(pgroup->motor_handle[0]) >> 8);
    txdata[1] = (uint8_t)Motor_GetOutput(pgroup->motor_handle[0]);
    txdata[2] = (uint8_t)((int16_t)Motor_GetOutput(pgroup->motor_handle[1]) >> 8);
    txdata[3] = (uint8_t)Motor_GetOutput(pgroup->motor_handle[1]);
    txdata[4] = (uint8_t)((int16_t)Motor_GetOutput(pgroup->motor_handle[2]) >> 8);
    txdata[5] = (uint8_t)Motor_GetOutput(pgroup->motor_handle[2]);
    txdata[6] = (uint8_t)((int16_t)Motor_GetOutput(pgroup->motor_handle[3]) >> 8);
    txdata[7] = (uint8_t)Motor_GetOutput(pgroup->motor_handle[3]);
    FDCAN_Send(pgroup->can_handle, &(pgroup->can_header), txdata);
}

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

void Motor_IsLost(Motor_DataTypeDef *pmotor)
{
    if (pmotor == NULL)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    pmotor->state = ((now - pmotor->encoder.last_update_time) > MOTOR_OFFLINE_TIME);
}
