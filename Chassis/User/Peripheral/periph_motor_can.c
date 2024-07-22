/*
 * @Project: Infantry Code
 * @Author: GDDG08
 * @Date: 2024-07-17 17:42:09
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 20:01:06
 */
/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Chen Zhihong
 * @LastEditTime: 2024-07-16 02:22:50
 */

#include "periph_motor_can.h"
#include "config_ctrl.h"

FDCAN_HandleTypeDef *MOTOR_CAN_HANDLER = &hfdcan1;

/**
 * @brief      Chassis motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
void M3508_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata)
{
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.angle = (float)((int16_t)((uint16_t)rxdata[0] << 8 | (uint16_t)rxdata[1]));
    pmotor->encoder.speed = (float)((int16_t)((uint16_t)rxdata[2] << 8 | (uint16_t)rxdata[3])) / pmotor->dec_ratio;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxdata[4] << 8 | (uint16_t)rxdata[5]));
    pmotor->encoder.temp = rxdata[6];
    pmotor->encoder.last_update_time = HAL_GetTick();
}

/**
 * @brief      Gimbal motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
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
    if (pmotor == &Motor_GimbalYaw)
    {
        if (pmotor->encoder.limited_angle < -180)
        {
            pmotor->encoder.limited_angle += 360;
        }
        else if (pmotor->encoder.limited_angle > 180)
        {
            pmotor->encoder.limited_angle -= 360;
        }
    }
    pmotor->encoder.last_update_time = HAL_GetTick();
}

/**
 * @brief      Motor encoder decoding callback function
 * @param      canid: CAN Handle number
 * @param      stdid: CAN identifier
 * @param      rxdata: CAN rx data buff
 * @retval     NULL
 */
#if ROBOT_ID == WHITE_MISTRESS
void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[])
{
    if (phfdcan == &hfdcan1)
    {
        if (stdid == FORWARD_LEFT_CAN_ID)
        {
            M3508_Decode(&Motor_ForwardLeft, rxdata);
        }
        if (stdid == FORWARD_RIGHT_CAN_ID)
        {
            M3508_Decode(&Motor_ForwardRight, rxdata);
        }
        if (stdid == BACKWARD_RIGHT_CAN_ID)
        {
            M3508_Decode(&Motor_BackwardRight, rxdata);
        }
        if (stdid == BACKWARD_LEFT_CAN_ID)
        {
            M3508_Decode(&Motor_BackwardLeft, rxdata);
        }
    }
    else if (phfdcan == &hfdcan3)
    {
        if (stdid == YAW_CAN_ID)
        {
            GM6020_Decode(&Motor_GimbalYaw, rxdata);
        }
    }
}
#endif
#if ROBOT_ID == STAR_DUST
void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[])
{
    if (phfdcan == &hfdcan1)
    {
        if (stdid == FORWARD_LEFT_CAN_ID)
        {
            M3508_Decode(&Motor_ForwardLeft, rxdata);
        }
        if (stdid == FORWARD_RIGHT_CAN_ID)
        {
            M3508_Decode(&Motor_ForwardRight, rxdata);
        }
        if (stdid == BACKWARD_RIGHT_CAN_ID)
        {
            M3508_Decode(&Motor_BackwardRight, rxdata);
        }
        if (stdid == BACKWARD_LEFT_CAN_ID)
        {
            M3508_Decode(&Motor_BackwardLeft, rxdata);
        }
        if (stdid == YAW_CAN_ID)
        {
            GM6020_Decode(&Motor_GimbalYaw, rxdata);
        }
    }
}
#endif

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

void Motor_Init(Motor_DataTypeDef *pmotor, uint32_t stdid, float dec_ratio)
{
    if (pmotor == NULL)
    {
        return;
    }
    pmotor->state = MOTOR_CONNECTED;
    pmotor->stdid = stdid;
    pmotor->dec_ratio = dec_ratio;
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
