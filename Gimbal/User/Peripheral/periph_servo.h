/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-10 16:31:12
 */

#ifndef PERIPH_SERVO_H
#define PERIPH_SERVO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "util_pwm.h"

    typedef enum
    {
        SERVO_OFF = 0,
        SERVO_ON = 1
    } Servo_StateEnum;

    typedef struct
    {
		Servo_StateEnum state;
        PWM_HandleTypeDef pwm;
        float angle;
    } Servo_DataTypeDef;

    void Servo_SetAngle(Servo_DataTypeDef *servo, float angle);
    
    void Servo_Init(Servo_DataTypeDef *servo, TIM_HandleTypeDef *htim, uint32_t ch, uint32_t clk, float angle);
    void Servo_Start(Servo_DataTypeDef *servo);
    void Servo_Stop(Servo_DataTypeDef *servo);
    float Servo_GetAngle(Servo_DataTypeDef *servo);


#ifdef __cplusplus
}
#endif

#endif
