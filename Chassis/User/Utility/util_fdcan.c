/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 19:19:05
 */

#include "util_fdcan.h"

void FDCAN_InitTxHeader(FDCAN_TxHeaderTypeDef *pheader, uint32_t id)
{
    pheader->Identifier = id;
    if (id >= 0x800)
    {
        pheader->IdType = FDCAN_EXTENDED_ID;
    }
    else
    {
        pheader->IdType = FDCAN_STANDARD_ID;
    }
    pheader->TxFrameType = FDCAN_DATA_FRAME;
    pheader->DataLength = FDCAN_DLC_BYTES_8;
    pheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pheader->BitRateSwitch = FDCAN_BRS_OFF;
    pheader->FDFormat = FDCAN_CLASSIC_CAN;
    pheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pheader->MessageMarker = 0;
}

void FDCAN_IntFilterAndStart(FDCAN_HandleTypeDef *phfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x0400;
    sFilterConfig.FilterID2 = 0x0000;

    uint32_t ret;
    if (HAL_FDCAN_ConfigFilter(phfdcan, &sFilterConfig) != HAL_OK)
    {
        FDCAN_ErrorHandler(ret);
    }
    if (HAL_FDCAN_ConfigGlobalFilter(phfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        FDCAN_ErrorHandler(ret);
    }
    if (HAL_FDCAN_Start(phfdcan) != HAL_OK)
    {
        FDCAN_ErrorHandler(ret);
    }
    if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        FDCAN_ErrorHandler(ret);
    }
    if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        FDCAN_ErrorHandler(ret);
    }
}

void FDCAN_Send(FDCAN_HandleTypeDef *phfdcan, FDCAN_TxHeaderTypeDef *ptxhead, uint8_t *pdata)
{
    uint32_t ret;
    if (HAL_FDCAN_AddMessageToTxFifoQ(phfdcan, ptxhead, pdata) != HAL_OK)
    {
        if (phfdcan->ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL)
        {
            HAL_FDCAN_Stop(phfdcan);
            phfdcan->Instance->TXFQS = 0x03;
            HAL_FDCAN_Start(phfdcan);
        }
        else
        {
            FDCAN_ErrorHandler(ret);
        }
    }
}

uint8_t CAN_Error_Flag = 0;
void FDCAN_ErrorHandler(uint32_t ret)
{
    CAN_Error_Flag = 1;
}
