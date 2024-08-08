/*
 * @Project: Infantry Code
 *
 * @Author: Q
 * @Date: 2024-07-03 17:12:00
 * @LastEditors: Q
 * @LastEditTime: 2024-07-03 17:12:00
 */

#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"
#include "cmsis_os.h"
#include "app_gimbal.h"
#include "protocol_referee.h"
#include "periph_motor_can.h"

#define SEVEN_PIN_LOST_ERR 1
#define CAN_LOST_ERR 2
#define OVER_TEMP_ERR 3
#define SNAIL_DIED_ERR 1
void Monitor_Check(void);
void Monitor_Init(void);
void beep_once(uint16_t ,uint16_t);
#ifdef __cplusplus
}
#endif

#endif
