/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-08-03 10:46:15
 */

#include "periph_motor_pwm.h"
#include "sys_dwt.h"
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
    float duty = MotorPWM_GetOutput(pmotor) * 0.4 / 50 + 0.5f;
    if (duty < 0.5f)
    {
        duty = 0.5f;
    }
    if (duty > 0.9f)
    {
        duty = 0.9f;
    }
    PWM_SetDuty(&(pmotor->pwm), duty);
}

/**
 * @brief      Read motor PWM encoder
 * @param      pmotor: The pointer points to the motor group to be sent
 * @retval     NULL
 **/
void Motor_PWM_ReadEncoder(MotorPWM_DataTypeDef *pmotor)
{
    if (pmotor == NULL)
    {
        return;
    }
    pmotor->encoder.counter = __HAL_TIM_GET_COUNTER(pmotor->encoder.htim);
    pmotor->encoder.temp_tick = DWT_GetTimeline_us();
    pmotor->encoder.temp = pmotor->encoder.counter - pmotor->encoder.last_counter;
    if (pmotor->encoder.temp > 32768)
    {
        pmotor->encoder.temp -= 65535;
    }
    else if (pmotor->encoder.temp < -32768)
    {
        pmotor->encoder.temp += 65535;
    }
    pmotor->encoder.speed = (float)pmotor->encoder.temp * 2 * 3.1415926f / (((float)(pmotor->encoder.temp_tick - pmotor->encoder.last_tick)) / 1000000) * 0.0235 / (4 * pmotor->encoder.encoder_lines);
    // register_counter * (numbers of turns to rads:2 * PI) / (time:us_to_s) * (radius:0.0235m) / (4 * lines)
    pmotor->encoder.last_counter = pmotor->encoder.counter;
    pmotor->encoder.last_tick = pmotor->encoder.temp_tick;
}

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
void MotorPWM_Init(MotorPWM_DataTypeDef *pmotor, TIM_HandleTypeDef *htim, uint32_t ch, uint32_t clk, uint32_t freq, TIM_HandleTypeDef *htim_enc, uint16_t encoder_lines)
{
    if (pmotor == NULL)
    {
        return;
    }
    pmotor->state = MOTORPWM_CONNECTED;
    pmotor->encoder.last_update_time = HAL_GetTick();
    pmotor->encoder.encoder_lines = encoder_lines;
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
