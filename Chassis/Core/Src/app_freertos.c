/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_chassis.h"
#include "protocol_board.h"
#include "config_ctrl.h"
#include "app_gimbal.h"
#include "periph_cap.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId InitHandle;
osThreadId GimbalHandle;
osThreadId BoardComHandle;
osThreadId ChassisHandle;
osThreadId UIHandle;
osThreadId CapHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Init_Task(void const* argument);
void Gimbal_Task(void const* argument);
void BoardCom_Task(void const* argument);
void Chassis_Task(void const* argument);
void UI_Task(void const* argument);
void Cap_Task(void const* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
//    /* definition and creation of Init */
//    osThreadDef(Init, Init_Task, osPriorityRealtime, 0, 128);
//    InitHandle = osThreadCreate(osThread(Init), NULL);

    /* definition and creation of Gimbal */
    osThreadDef(Gimbal, Gimbal_Task, osPriorityRealtime, 0, 128);
    GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

    /* definition and creation of BoardCom */
    osThreadDef(BoardCom, BoardCom_Task, osPriorityRealtime, 0, 128);
    BoardComHandle = osThreadCreate(osThread(BoardCom), NULL);

    /* definition and creation of Chassis */
    osThreadDef(Chassis, Chassis_Task, osPriorityRealtime, 0, 512);
    ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

    /* definition and creation of UI */
    osThreadDef(UI, UI_Task, osPriorityRealtime, 0, 128);
    UIHandle = osThreadCreate(osThread(UI), NULL);

    /* definition and creation of Cap */
    osThreadDef(Cap, Cap_Task, osPriorityRealtime, 0, 128);
    CapHandle = osThreadCreate(osThread(Cap), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Init_Task */
/**
 * @brief Function implementing the Init thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Init_Task */
__weak void Init_Task(void const* argument)
{
    /* USER CODE BEGIN Init_Task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
 * @brief Function implementing the Gimbal thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const* argument)
{
    /* USER CODE BEGIN Gimbal_Task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_BoardCom_Task */
/**
* @brief Function implementing the BoardCom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BoardCom_Task */
__weak void BoardCom_Task(void const* argument)
{
    /* USER CODE BEGIN BoardCom_Task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END BoardCom_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
 * @brief Function implementing the Chassis thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const* argument)
{
    /* USER CODE BEGIN Chassis_Task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_UI_Task */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Task */
__weak void UI_Task(void const* argument)
{
    /* USER CODE BEGIN UI_Task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END UI_Task */
}

/* USER CODE BEGIN Header_Cap_Task */
/**
* @brief Function implementing the Cap thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cap_Task */
__weak void Cap_Task(void const* argument)
{
    /* USER CODE BEGIN Cap_Task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END Cap_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

