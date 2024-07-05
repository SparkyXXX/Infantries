/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-05 17:31:27
 */

#include "util_uart.h"

void UART_Init(UART_HandleTypeDef *huart)
{
    uint32_t ret;
    if (HAL_UART_Init(huart) != HAL_OK)
    {
        UART_ErrorHandler(ret);
    }
}

void UART_Send(UART_HandleTypeDef *huart, uint8_t txdata[], uint16_t size, uint32_t timeout)
{
    if ((huart == NULL) || (txdata == NULL))
    {
        UART_ErrorHandler(HAL_ERROR);
    }
        
    uint32_t ret = HAL_UART_Transmit(huart, txdata, size, 10);
    if (ret != HAL_OK) 
    {
        UART_ErrorHandler(ret);
    }
}

void UART_SendIT(UART_HandleTypeDef *huart, uint8_t txdata[], uint16_t size)
{
    if ((huart == NULL) || (txdata == NULL))
    {
        UART_ErrorHandler(HAL_ERROR);
    }

    uint32_t ret = HAL_UART_Transmit_IT(huart, txdata, size);
    if (ret != HAL_OK) 
    {
        UART_ErrorHandler(ret);
    }
}

void UART_SendITForce(UART_HandleTypeDef *huart, uint8_t txdata[], uint16_t size)
{
    if ((huart == NULL) || (txdata == NULL))
    {
        UART_ErrorHandler(HAL_ERROR);
    }

    __HAL_UNLOCK(huart);
    uint32_t ret = HAL_UART_Transmit_IT(huart, txdata, size);
    if (ret != HAL_OK) 
    {
        UART_ErrorHandler(ret);
    }
}

void UART_InitDMA(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void UART_SendDMA(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size) 
{
    if ((huart == NULL) || (txdata == NULL))
    {
        UART_ErrorHandler(HAL_ERROR);
    }

    uint32_t ret = HAL_UART_Transmit_DMA(huart, txdata, size);
    if (ret != HAL_OK) 
    {
        UART_ErrorHandler(ret);
    }
}

HAL_StatusTypeDef UART_ReceiveDMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U)) 
        {
            return HAL_ERROR;
        }
            
        __HAL_LOCK(huart);
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
        if (!(IS_LPUART_INSTANCE(huart->Instance)))
        {
            if (READ_BIT(huart->Instance->CR2, USART_CR2_RTOEN) != 0U)
            {
                ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RTOIE);
            }
        }
        return (UART_Start_Receive_DMA(huart, pData, Size));
    }
    else 
    {
        return HAL_BUSY;
    }
}

uint16_t UART_DMACurrentDataCounter(DMA_HandleTypeDef *dma_handle)
{
    return ((uint16_t)__HAL_DMA_GET_COUNTER(dma_handle));
}

uint8_t UART_Error_Flag = 0;
void UART_ErrorHandler(uint32_t ret)
{
    UART_Error_Flag = 1;
}
