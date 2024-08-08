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
#include "protocol_board.h"
#include "config_ctrl.h"
#include "app_ins.h"
#include "app_gimbal.h"
#include "app_remote.h"
#include "app_autoaim.h"
#include "app_shoot.h"

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
osThreadId ShootHandle;
osThreadId AutoAimHandle;
osThreadId InsHandle;
osThreadId RemoteHandle;
osThreadId MonitorHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Init_Task(void const * argument);
void Gimbal_Task(void const * argument);
void BoardCom_Task(void const * argument);
void Shoot_Task(void const * argument);
void AutoAim_Task(void const * argument);
void Ins_Task(void const * argument);
void Remote_Task(void const * argument);
void Monitor_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
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
void MX_FREERTOS_Init(void) {
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
  /* definition and creation of Init */
  osThreadDef(Init, Init_Task, osPriorityRealtime, 0, 128);
  InitHandle = osThreadCreate(osThread(Init), NULL);

  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, Gimbal_Task, osPriorityRealtime, 0, 128);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of BoardCom */
  osThreadDef(BoardCom, BoardCom_Task, osPriorityRealtime, 0, 128);
  BoardComHandle = osThreadCreate(osThread(BoardCom), NULL);

  /* definition and creation of Shoot */
  osThreadDef(Shoot, Shoot_Task, osPriorityRealtime, 0, 128);
  ShootHandle = osThreadCreate(osThread(Shoot), NULL);

  /* definition and creation of AutoAim */
  osThreadDef(AutoAim, AutoAim_Task, osPriorityRealtime, 0, 256);
  AutoAimHandle = osThreadCreate(osThread(AutoAim), NULL);

  /* definition and creation of Ins */
  osThreadDef(Ins, Ins_Task, osPriorityRealtime, 0, 128);
  InsHandle = osThreadCreate(osThread(Ins), NULL);

  /* definition and creation of Remote */
  osThreadDef(Remote, Remote_Task, osPriorityRealtime, 0, 128);
  RemoteHandle = osThreadCreate(osThread(Remote), NULL);

  /* definition and creation of Monitor */
  osThreadDef(Monitor, Monitor_Task, osPriorityRealtime, 0, 128);
  MonitorHandle = osThreadCreate(osThread(Monitor), NULL);

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
__weak void Init_Task(void const * argument)
{
  /* USER CODE BEGIN Init_Task */
    /* Infinite loop */
    for (;;) {
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
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
    /* Infinite loop */
    for (;;) {
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
__weak void BoardCom_Task(void const * argument)
{
  /* USER CODE BEGIN BoardCom_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BoardCom_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
 * @brief Function implementing the Shoot thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_AutoAim_Task */
/**
* @brief Function implementing the AutoAim thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AutoAim_Task */
__weak void AutoAim_Task(void const * argument)
{
  /* USER CODE BEGIN AutoAim_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AutoAim_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the Ins thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
__weak void Ins_Task(void const * argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void const * argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Monitor_Task */
/**
* @brief Function implementing the Monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Monitor_Task */
__weak void Monitor_Task(void const * argument)
{
  /* USER CODE BEGIN Monitor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Monitor_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

