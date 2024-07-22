/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-15 00:44:14
 */

#ifndef PERIPH_MOTOR_CAN_H
#define PERIPH_MOTOR_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "util_fdcan.h"

#define MOTOR_OFFLINE_TIME 10

#define PITCH_CAN_ID 0x206
#define FEEDER_CAN_ID 0x201

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
        float output;
    } Motor_DataTypeDef;

    typedef struct
    {
        uint8_t motor_num;
        Motor_DataTypeDef *motor_handle[4];
        FDCAN_HandleTypeDef *can_handle;
        FDCAN_TxHeaderTypeDef can_header;
    } Motor_GroupDataTypeDef;

    void GM6020_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
    void M2006_Decode(Motor_DataTypeDef *pmotor, uint8_t *rxdata);
    void Motor_CAN_Decode(FDCAN_HandleTypeDef *phfdcan, uint32_t stdid, uint8_t rxdata[]);
    void Motor_CAN_SendGroupOutput(Motor_GroupDataTypeDef *pgroup);
    void Motor_Init(Motor_DataTypeDef *pmotor, uint32_t stdid);
    void Motor_InitGroup(Motor_GroupDataTypeDef *pgroup, uint8_t motor_num, FDCAN_HandleTypeDef *phcan, uint16_t stdid);
    void Motor_SetOutput(Motor_DataTypeDef *pmotor, float output);
    float Motor_GetOutput(Motor_DataTypeDef *pmotor);
    //    uint8_t Motor_IsLost(Motor_DataTypeDef *pmotor);
    void Motor_IsLost(Motor_DataTypeDef *pmotor);

#endif

#ifdef __cplusplus
}
#endif
