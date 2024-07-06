/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-19 23:31:39
 */

#ifndef PROTOCOL_MOTOR_H
#define PROTOCOL_MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "periph_motor_can.h"

#define FORWARD_LEFT_CAN_ID 0x201
#define FORWARD_RIGHT_CAN_ID 0x202
#define BACKWARD_RIGHT_CAN_ID 0x203
#define BACKWARD_LEFT_CAN_ID 0x204
#define YAW_CAN_ID 0x205

extern float Wheel_Dec_Ratio;

	static void M3508_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
	static void GM6020_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
    void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
    void Motor_CAN_SendGroupOutput(Motor_GroupDataTypeDef *pgroup);

#ifdef __cplusplus
}
#endif

#endif