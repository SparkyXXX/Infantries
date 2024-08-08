/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-14 11:21:49
 */

#ifndef CALLBACK_CTRL_H
#define CALLBACK_CTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "protocol_board.h"
#include "periph_remote.h"
#include "ext_remote_dev.h"

#define FDCAN_RX_LEN 200
#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))

    void UART_ReceiveHandler(UART_HandleTypeDef *huart);
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *phfdcan, uint32_t RxFifo0ITs);

#ifdef __cplusplus
}
#endif

#endif