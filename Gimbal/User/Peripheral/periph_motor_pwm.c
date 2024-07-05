/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-17 20:28:15
 */

#include "periph_motor_pwm.h"

/**
 * @brief      Initialize the motor
 * @param      pmotor: Pointer to motor object
 * @param      htim: Pwm motor timer handle
 * @param      ch: Pwm motor timer channel
 * @param      clk: MCU clock
 * @param      freq: Pwm motor timer frequency
 * @param      htim_enc: Pwm motor encoder handle
 * @retval     NULL
 */
void MotorPWM_Init(MotorPWM_DataTypeDef *pmotor, TIM_HandleTypeDef *htim, uint32_t ch, uint32_t clk, uint32_t freq, TIM_HandleTypeDef *htim_enc)
{
    if (pmotor == NULL)
    {
        return;
    }
    pmotor->state = MOTORPWM_CONNECTED;
    pmotor->encoder.last_update_time = HAL_GetTick();
    
    PWM_Init(&(pmotor->pwm), htim, ch, clk);
    PWM_SetFreq(&(pmotor->pwm), freq);
    PWM_SetDuty(&(pmotor->pwm), 0);
    PWM_Start(&(pmotor->pwm));
    
    pmotor->encoder.htim = htim_enc;
    if (htim_enc != NULL)
    {
        HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL);
    }
}

/**
 * @brief      Initialization of motor group
 * @param      pgroup: Pointer to motor group
 * @param      motor_num: Number of motor group
 * @retval     NULL
 */
void MotorPWM_InitGroup(MotorPWM_GroupDataTypeDef *pgroup, uint8_t motor_num)
{
    if (pgroup == NULL) 
    {
        return;
    }
    pgroup->motor_num = motor_num;

    for (int i = 0; i < motor_num; i++) 
    {
        pgroup->motor_handle[i] = NULL;
    }
}

/**
 * @brief      Set motor PID output value
 * @param      pmotor: Pointer to motor object
 * @param      output: target value
 * @retval     NULL
 */
void MotorPWM_SetOutput(MotorPWM_DataTypeDef *pmotor, float output)
{
    if (pmotor == NULL) 
    {
        return;
    }
    pmotor->output = output;
}

/**
 * @brief      Get motor PID output value
 * @param      pmotor: Pointer to motor object
 * @retval     Output value
 */
float MotorPWM_GetOutput(MotorPWM_DataTypeDef *pmotor)
{
    return pmotor->output;
}
