/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-25 11:34:16
 */

#ifndef PERIPH_MOTOR_CAN_H
#define PERIPH_MOTOR_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "util_fdcan.h"

#define MOTOR_OFFLINE_TIME 10
#define FDCAN_RX_LEN 200

#define FORWARD_LEFT_CAN_ID 0x201
#define FORWARD_RIGHT_CAN_ID 0x202
#define BACKWARD_RIGHT_CAN_ID 0x203
#define BACKWARD_LEFT_CAN_ID 0x204
#define YAW_CAN_ID 0x205

    typedef enum
    {
        MOTOR_CONNECTED = 0,
        MOTOR_LOST = 1
    } Motor_StateEnum;

    typedef struct
    {
        float angle;
        float speed;
        float current;
        int8_t temp;

        int16_t last_angle;
        int32_t round_count;
        float limited_angle;
        float consequent_angle;

        uint32_t last_update_time;
    } Encoder_DataTypeDef;

    typedef struct
    {
        Motor_StateEnum state;
        Encoder_DataTypeDef encoder;
        uint32_t stdid;
        float dec_ratio;
        float output;
    } Motor_DataTypeDef;

    typedef struct
    {
        uint8_t motor_num;
        Motor_DataTypeDef *motor_handle[4];
        FDCAN_HandleTypeDef *can_handle;
        FDCAN_TxHeaderTypeDef can_header;
    } Motor_GroupDataTypeDef;

    void M3508_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
    void GM6020_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
    void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[]);
    void Motor_CAN_SendGroupOutput(Motor_GroupDataTypeDef *pgroup);
    void Motor_Init(Motor_DataTypeDef *pmotor, uint32_t stdid, float dec_ratio);
    void Motor_InitGroup(Motor_GroupDataTypeDef *pgroup, uint8_t motor_num, FDCAN_HandleTypeDef *phcan, uint16_t stdid);
    void Motor_SetOutput(Motor_DataTypeDef *pmotor, float output);
    float Motor_GetOutput(Motor_DataTypeDef *pmotor);
    void Motor_IsLost(Motor_DataTypeDef *pmotor);

#ifdef __cplusplus
}
#endif

#endif
