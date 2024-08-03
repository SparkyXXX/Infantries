/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-08-03 10:46:14
 */

#ifndef PERIPH_MOTOR_PWM_H
#define PERIPH_MOTOR_PWM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "util_pwm.h"

    typedef enum
    {
        MOTORPWM_CONNECTED = 0,
        MOTORPWM_LOST = 1
    } MotorPWM_StateEnum;

    typedef struct
    {
        float speed;
        uint16_t encoder_lines;
        TIM_HandleTypeDef *htim;
        uint32_t ch;
        uint8_t direction;
        int64_t counter;
        int64_t last_counter;
        uint64_t last_tick;
        uint32_t last_update_time;
        int64_t temp;
        uint64_t temp_tick;
    } EncoderPWM_DataTypeDef;

    typedef struct
    {
        MotorPWM_StateEnum state;
        EncoderPWM_DataTypeDef encoder;
        PWM_HandleTypeDef pwm;
        float output;
    } MotorPWM_DataTypeDef;

    typedef struct
    {
        uint8_t motor_num;
        MotorPWM_DataTypeDef *motor_handle[4];
    } MotorPWM_GroupDataTypeDef;

    void Motor_PWM_ReadEncoder(MotorPWM_DataTypeDef *pmotor);
    void Motor_PWM_SendOutput(MotorPWM_DataTypeDef *pmotor);
    void MotorPWM_Init(MotorPWM_DataTypeDef *pmotor, TIM_HandleTypeDef *htim, uint32_t ch, uint32_t clk, uint32_t freq, TIM_HandleTypeDef *htim_enc, uint16_t encoder_lines);
    void MotorPWM_InitGroup(MotorPWM_GroupDataTypeDef *pgroup, uint8_t motor_num);
    void MotorPWM_SetOutput(MotorPWM_DataTypeDef *pmotor, float output);
    float MotorPWM_GetOutput(MotorPWM_DataTypeDef *pmotor);

#ifdef __cplusplus
}
#endif

#endif
