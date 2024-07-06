/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-19 23:31:47
 */

#include "protocol_motor.h"
#include "config_ctrl.h"

/**
 * @brief      Chassis motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
static void M3508_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata)
{
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.angle = (float)((int16_t)((uint16_t)rxdata[0] << 8 | (uint16_t)rxdata[1]));
    pmotor->encoder.speed = (float)((int16_t)((uint16_t)rxdata[2] << 8 | (uint16_t)rxdata[3])) / Wheel_Dec_Ratio;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxdata[4] << 8 | (uint16_t)rxdata[5]));
    pmotor->encoder.temp = rxdata[6];
    pmotor->encoder.last_update_time = HAL_GetTick();
}

/**
 * @brief      Gimbal motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
static void GM6020_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata)
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
void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[], uint32_t len)
{
    if (phfdcan == &hfdcan1)
    {
		if (stdid == FORWARD_LEFT_CAN_ID) {M3508_Decode(&Motor_ForwardLeft, rxdata);}
		if (stdid == FORWARD_RIGHT_CAN_ID) {M3508_Decode(&Motor_ForwardRight, rxdata);}
		if (stdid == BACKWARD_RIGHT_CAN_ID) {M3508_Decode(&Motor_BackwardRight, rxdata);}
		if (stdid == BACKWARD_LEFT_CAN_ID) {M3508_Decode(&Motor_BackwardLeft, rxdata);}
		if (stdid == YAW_CAN_ID) {GM6020_Decode(&Motor_GimbalYaw, rxdata);}
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
