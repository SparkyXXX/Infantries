/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-03-18 11:46:10
 */

#ifndef PROTOCOL_MOTOR_H
#define PROTOCOL_MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "config_ctrl.h"
#include "periph_motor_can.h"
#include "periph_motor_pwm.h"

#define PITCH_CAN_ID 0x206
#define FEEDER_CAN_ID 0x201

    static void GM6020_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
    static void M2006_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
    void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
    void Motor_CAN_SendGroupOutput(Motor_GroupDataTypeDef *pgroup);
    void Motor_PWM_ReadEncoder(MotorPWM_DataTypeDef *pmotor, uint8_t multiple);
    void Motor_PWM_SendOutput(MotorPWM_DataTypeDef *pmotor);

#ifdef __cplusplus
}
#endif

#endif