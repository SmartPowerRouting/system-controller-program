/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "printf.h"
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
/* Definitions for os_led_blink */
osThreadId_t os_led_blinkHandle;
const osThreadAttr_t os_led_blink_attributes = {
    .name = "os_led_blink",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for uart1_rx */
osThreadId_t uart1_rxHandle;
const osThreadAttr_t uart1_rx_attributes = {
    .name = "uart1_rx",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for uart1_rx_msg */
osMessageQueueId_t uart1_rx_msgHandle;
const osMessageQueueAttr_t uart1_rx_msg_attributes = {
    .name = "uart1_rx_msg"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void os_led_blink_tsk(void *argument);
void uart1_rx_tsk(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the queue(s) */
  /* creation of uart1_rx_msg */
  uart1_rx_msgHandle = osMessageQueueNew(16, sizeof(uint8_t), &uart1_rx_msg_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of os_led_blink */
  os_led_blinkHandle = osThreadNew(os_led_blink_tsk, NULL, &os_led_blink_attributes);

  /* creation of uart1_rx */
  uart1_rxHandle = osThreadNew(uart1_rx_tsk, NULL, &uart1_rx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_os_led_blink_tsk */
/**
 * @brief  Function implementing the os_led_blink thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_os_led_blink_tsk */
void os_led_blink_tsk(void *argument)
{
  /* USER CODE BEGIN os_led_blink_tsk */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_led_blink_tsk */
}

/* USER CODE BEGIN Header_uart1_rx_tsk */
/**
 * @brief Function implementing the uart1_rx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_uart1_rx_tsk */
void uart1_rx_tsk(void *argument)
{
  /* USER CODE BEGIN uart1_rx_tsk */
  /* Infinite loop */
  for (;;)
  {
    // read msg queue and if success, print data
    uint8_t data[255] = {'\0'};
    if (osMessageQueueGet(uart1_rx_msgHandle, &data, NULL, 100) == osOK)
    {
      printf(">>> MsgQueue Received: %s\n", data);
    }
  }
  /* USER CODE END uart1_rx_tsk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
