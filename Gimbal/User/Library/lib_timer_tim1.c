#include "FreeRTOS.h"
#include "semphr.h"

#include "tim.h"
#include "lib_timer_tim1.h"
#include "main.h"
#include "sys_dwt.h"

SemaphoreHandle_t Timer1SemaphoreHandler;  
StaticSemaphore_t Timer1SemaphoreBuffer;

SemaphoreHandle_t Timer2SemaphoreHandler;  
StaticSemaphore_t Timer2SemaphoreBuffer;

SemaphoreHandle_t Timer3SemaphoreHandler;  
StaticSemaphore_t Timer3SemaphoreBuffer;

SemaphoreHandle_t Timer4SemaphoreHandler;  
StaticSemaphore_t Timer4SemaphoreBuffer;

//void TimerInit() {
//	HAL_TIM_Base_Start(&htim4);
//	Timer1SemaphoreHandler = xSemaphoreCreateBinaryStatic(&Timer1SemaphoreBuffer); 
//	Timer2SemaphoreHandler = xSemaphoreCreateBinaryStatic(&Timer2SemaphoreBuffer); 
//	Timer3SemaphoreHandler = xSemaphoreCreateBinaryStatic(&Timer3SemaphoreBuffer); 
//	Timer4SemaphoreHandler = xSemaphoreCreateBinaryStatic(&Timer4SemaphoreBuffer); 
//}

//void TIM4_IRQHandler(void){
//	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
//	uint16_t itstatus = 0x0, itenable = 0x0;
//	TIM_TypeDef *TIMx = TIM4;
//	itstatus = TIMx->SR & TIM_IT_CC1;
//	itenable = TIMx->DIER & TIM_IT_CC1;
//	
//	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET)) {
//		TIMx->SR = (uint16_t)~TIM_IT_CC1;
//		TIMx->DIER &= (uint16_t)~TIM_IT_CC1; /* 禁能CC1中断 */
//		xSemaphoreGiveFromISR(Timer1SemaphoreHandler, &xHigherPriorityTaskWoken );
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//	}
//		itstatus = TIMx->SR & TIM_IT_CC2;
//		itenable = TIMx->DIER & TIM_IT_CC2;
//	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET)) {
//		TIMx->SR = (uint16_t)~TIM_IT_CC2;
//		TIMx->DIER &= (uint16_t)~TIM_IT_CC2; /* 禁能CC2中断 */
//		xSemaphoreGiveFromISR(Timer2SemaphoreHandler, &xHigherPriorityTaskWoken );
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//	}
//		itstatus = TIMx->SR & TIM_IT_CC3;
//		itenable = TIMx->DIER & TIM_IT_CC3;
//	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET)) {
//		TIMx->SR = (uint16_t)~TIM_IT_CC3;
//		TIMx->DIER &= (uint16_t)~TIM_IT_CC3; /* 禁能CC2中断 */
//		xSemaphoreGiveFromISR(Timer3SemaphoreHandler, &xHigherPriorityTaskWoken );
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//	}
//		itstatus = TIMx->SR & TIM_IT_CC4;
//		itenable = TIMx->DIER & TIM_IT_CC4;
//	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET)) {
//		TIMx->SR = (uint16_t)~TIM_IT_CC4;
//		TIMx->DIER &= (uint16_t)~TIM_IT_CC4; /* 禁能CC4中断 */
//		xSemaphoreGiveFromISR(Timer4SemaphoreHandler, &xHigherPriorityTaskWoken );
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//	}
//}

void Timer_Delay_usE_1(uint8_t _CC, int64_t _uiTimeOut) {
	if(_uiTimeOut <= 0) return;
	
	if(_uiTimeOut <= 1000) {
		uint32_t now = DWT_GetTimeline_us();
		while(DWT_GetTimeline_us() < (uint32_t)(now + _uiTimeOut)) {
		}
		return;
	}
	
	_uiTimeOut -= 7;
	
	uint32_t cnt_now;
	uint32_t cnt_tar;
	TIM_TypeDef *TIMx = TIM4;
	
	cnt_now = TIMx->CNT;
	cnt_tar = cnt_now + _uiTimeOut; /* 计算捕获的计数器值 */
	
	if (_CC == 1) {
		TIMx->CCR1 = cnt_tar;             /* 设置捕获比较计数器CC1 */
		TIMx->SR = (uint16_t)~TIM_IT_CC1; /* 清除CC1中断标志 */
		TIMx->DIER |= TIM_IT_CC1;         /* 使能CC1中断 */
		xSemaphoreTake(Timer1SemaphoreHandler,portMAX_DELAY);
	} else if (_CC == 2) {
		TIMx->CCR2 = cnt_tar;             /* 设置捕获比较计数器CC2 */
		TIMx->SR = (uint16_t)~TIM_IT_CC2; /* 清除CC2中断标志 */
		TIMx->DIER |= TIM_IT_CC2;         /* 使能CC2中断 */
		xSemaphoreTake(Timer2SemaphoreHandler,portMAX_DELAY);
	} else if (_CC == 3) {
		TIMx->CCR3 = cnt_tar;             /* 设置捕获比较计数器CC3 */
		TIMx->SR = (uint16_t)~TIM_IT_CC3; /* 清除CC3中断标志 */
		TIMx->DIER |= TIM_IT_CC3;         /* 使能CC3中断 */
		xSemaphoreTake(Timer3SemaphoreHandler,portMAX_DELAY);
	} else if (_CC == 4) {
		TIMx->CCR4 = cnt_tar;             /* 设置捕获比较计数器CC4 */
		TIMx->SR = (uint16_t)~TIM_IT_CC4; /* 清除CC4中断标志 */
		TIMx->DIER |= TIM_IT_CC4;         /* 使能CC4中断 */
		xSemaphoreTake(Timer4SemaphoreHandler,portMAX_DELAY);
	} 
	return;
}


