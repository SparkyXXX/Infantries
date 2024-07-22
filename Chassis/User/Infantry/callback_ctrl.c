/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-15 00:41:13
 */

#include "callback_ctrl.h"
#include "periph_motor_can.h"
#include "protocol_board.h"
#include "protocol_referee.h"

static FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
static uint8_t FDCAN_RxData[FDCAN_RX_LEN];

extern const uint16_t REFEREE_FRAME_HEADER_SOF;
extern uint8_t Referee_RxData[REFEREE_RX_BUFF_LEN];
float uart_pe_count = 0;
float uart_fe_count = 0;
float uart_ore_count = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->ErrorCode & HAL_UART_ERROR_PE)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
        uart_pe_count++;
    }
    if (huart->ErrorCode & HAL_UART_ERROR_FE)
    {
        __HAL_UART_CLEAR_FEFLAG(huart);
        uart_fe_count++;
    }
    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        uart_ore_count++;
    }
    if (huart == &huart2)
    {
        Referee_Init(huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t rxdatalen)
{
    if (huart == &huart2)
    {
        for (uint16_t i = 0; i < rxdatalen; i++)
        {
            if (Referee_RxData[i] == REFEREE_FRAME_HEADER_SOF)
            {
                Referee_Decode(Referee_RxData + i, rxdatalen);
            }
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *phfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(phfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, FDCAN_RxData);

        BoardCom_Decode(phfdcan, (&FDCAN_RxHeader)->Identifier, FDCAN_RxData, ((&FDCAN_RxHeader)->DataLength) >> 16);
        Motor_CAN_Decode(phfdcan, (&FDCAN_RxHeader)->Identifier, FDCAN_RxData);

        if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            uint32_t ret;
            FDCAN_ErrorHandler(ret);
        }
    }
}
