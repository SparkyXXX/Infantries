/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-16 01:36:07
 */

#include "callback_ctrl.h"
#include "protocol_board.h"
#include "protocol_motor.h"
#include "protocol_referee.h"

static FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
static uint8_t FDCAN_RxData[FDCAN_RX_DATA];

extern const uint16_t REFEREE_FRAME_HEADER_SOF;
extern uint8_t Referee_RxData[REFEREE_RX_BUFF_LEN];

uint8_t uart_pe_count = 0;
uint8_t uart_fe_count = 0;
uint8_t uart_ore_count = 0;
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

/**
 * @brief      UART RX callback receiver function
 * @param      huart: Point to uart handle
 * @retval     NULL
 */
void UART_ReceiveHandler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        if (huart == &huart2)
        {
            __HAL_DMA_DISABLE(huart->hdmarx);
            uint16_t rxdatalen = REFEREE_RX_BUFF_LEN - UART_DMACurrentDataCounter(huart->hdmarx);
            for (uint16_t i = 0; i < rxdatalen; i++)
            {
                if (Referee_RxData[i] == REFEREE_FRAME_HEADER_SOF)
                {
                    Referee_Decode(Referee_RxData + i, rxdatalen);
                }
            }
            __HAL_DMA_SET_COUNTER(huart->hdmarx, REFEREE_RX_BUFF_LEN);
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *phfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(phfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, FDCAN_RxData);

        BoardCom_Decode(phfdcan, (&FDCAN_RxHeader)->Identifier, FDCAN_RxData, ((&FDCAN_RxHeader)->DataLength) >> 16);
        Motor_CAN_Decode(phfdcan, (&FDCAN_RxHeader)->Identifier, FDCAN_RxData, ((&FDCAN_RxHeader)->DataLength) >> 16);

        if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            uint32_t ret;
            FDCAN_ErrorHandler(ret);
        }
    }
}
