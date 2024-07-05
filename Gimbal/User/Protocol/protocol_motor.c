/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-23 00:12:26
 */

#include "protocol_motor.h"

Motor_DecodeTableTypeDef Gimbal_MotorDecodeTable[GIMBAL_MOTOR_AMOUNT] = {
    {PITCH_CAN_ID, &Motor_gimbalMotorPitch, &GM6020_Decode},
    {FEEDER_CAN_ID, &Motor_feederMotor, &M2006_Decode}};

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

    pmotor->encoder.last_update_time = HAL_GetTick();
}

/**
 * @brief      Feeder motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
static void M2006_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata)
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
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f +
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;

    pmotor->encoder.last_update_time = HAL_GetTick();
}

/**
 * @brief      Sending motor PWM output
 * @param      pmotor: The pointer points to the motor to be sent
 * @retval     NULL
 */
void Motor_PWM_SendOutput(MotorPWM_DataTypeDef *pmotor)
{
    if (pmotor == NULL)
    {
        return;
    }
    float duty = MotorPWM_GetOutput(pmotor) * 0.008f + 0.5f;
    if (duty < 0.5f)
    {
        duty = 0.5f;
    }
    if (duty > 0.9f)
    {
        duty = 0.9f;
    }
    pmotor->output = duty;
    PWM_SetDuty(&(pmotor->pwm), pmotor->output);
}

/**
 * @brief      Read motor PWM encoder
 * @param      pmotor: The pointer points to the motor group to be sent
 * @retval     NULL
 **/
void Motor_PWM_ReadEncoder(MotorPWM_DataTypeDef *pmotor, uint8_t multiple)
{
    uint16_t fdb = 0;
    if (pmotor == NULL)
    {
        return;
    }
    pmotor->encoder.direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(pmotor->encoder.htim);
    pmotor->encoder.counter = __HAL_TIM_GET_COUNTER(pmotor->encoder.htim);

    if (pmotor->encoder.direction == 0)
    {
        fdb = pmotor->encoder.counter;
    }
    else
    {
        fdb = 65535 - pmotor->encoder.counter;
    }
    if (fdb == 65535)
    {
        fdb = 0;
    }
    __HAL_TIM_SET_COUNTER(pmotor->encoder.htim, 0);
    pmotor->encoder.speed = (float)fdb * 2 * 3.1415926f * 1000 * 0.0235 / 4096;
    // register_counter * (numbers of turns to rads:2 * PI) * (ms_to_s:1000) * (radius:0.0235m) / (4 * 1024 lines)
}

void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[], uint32_t len)
{
    if (phfdcan == &hfdcan1)
    {
        for (int i = 0; i < GIMBAL_MOTOR_AMOUNT; i++)
        {
            if ((stdid == Gimbal_MotorDecodeTable[i].can_id) && Gimbal_MotorDecodeTable[i].decode_func != NULL)
            {
                Gimbal_MotorDecodeTable[i].decode_func(Gimbal_MotorDecodeTable[i].pmotor, rxdata);
                break;
            }
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
