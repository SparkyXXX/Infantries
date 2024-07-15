/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-14 22:46:42
 */

#ifndef CALLBACK_CTRL_H
#define CALLBACK_CTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "util_fdcan.h"
#include "util_uart.h"

#define REFEREE_RX_BUFF_LEN 500
#define FDCAN_RX_DATA 200

    void UART_ReceiveHandler(UART_HandleTypeDef *huart);
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *phfdcan, uint32_t RxFifo0ITs);

#ifdef __cplusplus
}
#endif

#endif