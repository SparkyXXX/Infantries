/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Chen Zhihong
 * @LastEditTime: 2024-08-07 03:11:06
 */

#include "callback_ctrl.h"
#include "periph_ext_remote.h"

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
    if (huart == &huart3)
    {
        HAL_UART_DMAStop(&huart3);
        HAL_UART_DeInit(&huart3);
        HAL_UART_Init(&huart3);
        Remote_Init(huart);
    }
    else if (huart == &huart2)
    {
        HAL_UART_DMAStop(huart);
        HAL_UART_DeInit(huart);
        HAL_UART_Init(huart);
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
        if (huart == &huart3)
        {
            __HAL_DMA_DISABLE(huart->hdmarx);
            Remote_DataTypeDef *remote = Remote_GetDataPtr();
            int rxdatalen = REMOTE_RX_BUFF_LEN - UART_DMACurrentDataCounter(huart->hdmarx);
            Remote_Decode(remote, remote->rx_data, rxdatalen);
            __HAL_DMA_SET_COUNTER(huart->hdmarx, REMOTE_RX_BUFF_LEN);
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
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
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Referee_RxData, REFEREE_RX_BUFF_LEN);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *phfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(phfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, FDCAN_RxData);
        if (phfdcan == BOARD_CAN_HANDLER)
        {
            BoardCom_Decode(FDCAN_RxData, (&FDCAN_RxHeader)->Identifier, ((&FDCAN_RxHeader)->DataLength) >> 16);
        }
        Motor_CAN_Decode(phfdcan, (&FDCAN_RxHeader)->Identifier, FDCAN_RxData);

        if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            uint32_t ret;
            FDCAN_ErrorHandler(ret);
        }
    }
}
