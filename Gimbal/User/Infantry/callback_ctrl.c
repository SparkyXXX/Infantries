/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-15 01:28:43
 */

#include "callback_ctrl.h"

FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
uint8_t FDCAN_RxData[FDCAN_RX_LEN];
//uint8_t Remote_Decode_Flag = 0;
//uint8_t Boardcom_Decode_Flag = 0;
//uint8_t Gimbal_Motor_Decode_Flag = 0;
//uint8_t Shoot_Motor_Decode_Flag = 0;

/**
 * @brief      UART RX callback receiver function
 * @param      huart: Point to uart handle
 * @retval     NULL
 */
void UART_ReceiveHandler(UART_HandleTypeDef* huart)
{
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        if(huart == &huart3)
        {
			Remote_DataTypeDef* remote = Remote_GetDataPtr();
            __HAL_DMA_DISABLE(huart->hdmarx);
			int rxdatalen = REMOTE_RX_BUFF_LEN - UART_DMACurrentDataCounter(REMOTE_UART_HANDLER->hdmarx);
			Remote_Decode(remote, remote->rx_data, rxdatalen);
            __HAL_DMA_SET_COUNTER(huart->hdmarx, REMOTE_RX_BUFF_LEN);
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* phfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(phfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, FDCAN_RxData);
		if (phfdcan == BOARD_CAN_HANDLER)
        {
			BoardCom_Decode(FDCAN_RxData, (&FDCAN_RxHeader)->Identifier, ((&FDCAN_RxHeader)->DataLength) >> 16);
        }
		if (phfdcan == MOTOR_CAN_HANDLER && (&FDCAN_RxHeader)->Identifier == PITCH_CAN_ID)
		{
			GM6020_Decode(&Motor_gimbalMotorPitch, FDCAN_RxData);
		}
		if (phfdcan == MOTOR_CAN_HANDLER && (&FDCAN_RxHeader)->Identifier == FEEDER_CAN_ID)
		{
			M2006_Decode(&Motor_feederMotor, FDCAN_RxData);
		}
        if(HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            uint32_t ret;
            FDCAN_ErrorHandler(ret);
        }
    }
}
