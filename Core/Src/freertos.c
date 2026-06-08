/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "tim.h"

#include "oled.h"
#include "mpu6050.h"

#include "Display.h"
#include "bluetooth.h"
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

//临时测试
void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT=0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/* USER CODE END Variables */
/* Definitions for TaskDisplay */
osThreadId_t TaskDisplayHandle;
const osThreadAttr_t TaskDisplay_attributes = {
  .name = "TaskDisplay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskBalance */
osThreadId_t TaskBalanceHandle;
const osThreadAttr_t TaskBalance_attributes = {
  .name = "TaskBalance",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskBtCmd */
osThreadId_t TaskBtCmdHandle;
const osThreadAttr_t TaskBtCmd_attributes = {
  .name = "TaskBtCmd",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BtCmdQueue */
osMessageQueueId_t BtCmdQueueHandle;
const osMessageQueueAttr_t BtCmdQueue_attributes = {
  .name = "BtCmdQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DisplayTask(void *argument);
extern void StartTaskBalance(void *argument);
extern void StartBtCmd(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  //OLED???
  OLED_Init();
  OLED_Clear();

  //MPU6050???
  MPU_Init();

  //??PWM???
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  //??????????
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

  //?????????
  extern uint8_t databyte;
  UART_Start_Receive_IT(&huart3,&databyte,1);

  //临时测试
  #include "core_cm3.h"
  DWT_Init();
  

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

  /* Create the queue(s) */
  /* creation of BtCmdQueue */
  BtCmdQueueHandle = osMessageQueueNew (16, sizeof(Bt_Cmd_t*), &BtCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskDisplay */
  TaskDisplayHandle = osThreadNew(DisplayTask, NULL, &TaskDisplay_attributes);

  /* creation of TaskBalance */
  TaskBalanceHandle = osThreadNew(StartTaskBalance, NULL, &TaskBalance_attributes);

  /* creation of TaskBtCmd */
  TaskBtCmdHandle = osThreadNew(StartBtCmd, NULL, &TaskBtCmd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DisplayTask */
/**
  * @brief  Function implementing the Display thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DisplayTask */
__weak void DisplayTask(void *argument)
{
  /* USER CODE BEGIN DisplayTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

