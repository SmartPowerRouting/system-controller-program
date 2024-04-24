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
#include "adc.h"
#include "printf.h"
#include "lcd.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * The data structure to store power data for a single source.
 */
typedef struct
{
  uint16_t voltage; // in mV
  uint16_t current; // in mA
  uint32_t power;   // in mW
} pwrData_t;

/*
 * The data structure to store power data for all channels, which is to be used in the queue.
 */
typedef struct
{
  pwrData_t mmc;
  pwrData_t bkup;
  pwrData_t out;
} sysPwrDataQueue_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint32_t adc_data[30];
/* USER CODE END Variables */
/* Definitions for os_led_blink */
osThreadId_t os_led_blinkHandle;
const osThreadAttr_t os_led_blink_attributes = {
    .name = "os_led_blink",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for esp_msg */
osThreadId_t esp_msgHandle;
const osThreadAttr_t esp_msg_attributes = {
    .name = "esp_msg",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for esp_conn */
osThreadId_t esp_connHandle;
const osThreadAttr_t esp_conn_attributes = {
    .name = "esp_conn",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for pwr_monitor */
osThreadId_t pwr_monitorHandle;
const osThreadAttr_t pwr_monitor_attributes = {
    .name = "pwr_monitor",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for dcdc_ctrl */
osThreadId_t dcdc_ctrlHandle;
const osThreadAttr_t dcdc_ctrl_attributes = {
    .name = "dcdc_ctrl",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for adc_handler */
osThreadId_t adc_handlerHandle;
const osThreadAttr_t adc_handler_attributes = {
    .name = "adc_handler",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for esp_redir */
osThreadId_t esp_redirHandle;
const osThreadAttr_t esp_redir_attributes = {
    .name = "esp_redir",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for esp_rx_queue */
osMessageQueueId_t esp_rx_queueHandle;
const osMessageQueueAttr_t esp_rx_queue_attributes = {
    .name = "esp_rx_queue"};
/* Definitions for esp_response_queue */
osMessageQueueId_t esp_response_queueHandle;
const osMessageQueueAttr_t esp_response_queue_attributes = {
    .name = "esp_response_queue"};
/* Definitions for usr_voltage_queue */
osMessageQueueId_t usr_voltage_queueHandle;
const osMessageQueueAttr_t usr_voltage_queue_attributes = {
    .name = "usr_voltage_queue"};
/* Definitions for pwr_cmd_queue */
osMessageQueueId_t pwr_cmd_queueHandle;
const osMessageQueueAttr_t pwr_cmd_queue_attributes = {
    .name = "pwr_cmd_queue"};
/* Definitions for sys_pwr_queue */
osMessageQueueId_t sys_pwr_queueHandle;
const osMessageQueueAttr_t sys_pwr_queue_attributes = {
    .name = "sys_pwr_queue"};
/* Definitions for dcdc_pwr_queue */
osMessageQueueId_t dcdc_pwr_queueHandle;
const osMessageQueueAttr_t dcdc_pwr_queue_attributes = {
    .name = "dcdc_pwr_queue"};
/* Definitions for esp_tx_queue */
osMessageQueueId_t esp_tx_queueHandle;
const osMessageQueueAttr_t esp_tx_queue_attributes = {
    .name = "esp_tx_queue"};
/* Definitions for pwrSelCmd_queue */
osMessageQueueId_t pwrSelCmd_queueHandle;
const osMessageQueueAttr_t pwrSelCmd_queue_attributes = {
    .name = "pwrSelCmd_queue"};
/* Definitions for uart1_rx_queue */
osMessageQueueId_t uart1_rx_queueHandle;
const osMessageQueueAttr_t uart1_rx_queue_attributes = {
    .name = "uart1_rx_queue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void os_led_blink_tsk(void *argument);
void esp_msg_tsk(void *argument);
void esp_conn_tsk(void *argument);
void pwr_monitor_tsk(void *argument);
void dcdc_ctrl_tsk(void *argument);
void adc_handler_tsk(void *argument);
void esp_redir_tsk(void *argument);

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
  /* creation of esp_rx_queue */
  esp_rx_queueHandle = osMessageQueueNew(6, 255, &esp_rx_queue_attributes);

  /* creation of esp_response_queue */
  esp_response_queueHandle = osMessageQueueNew(5, 255, &esp_response_queue_attributes);

  /* creation of usr_voltage_queue */
  usr_voltage_queueHandle = osMessageQueueNew(1, sizeof(uint16_t), &usr_voltage_queue_attributes);

  /* creation of pwr_cmd_queue */
  pwr_cmd_queueHandle = osMessageQueueNew(5, 2, &pwr_cmd_queue_attributes);

  /* creation of sys_pwr_queue */
  sys_pwr_queueHandle = osMessageQueueNew(2, 18, &sys_pwr_queue_attributes);

  /* creation of dcdc_pwr_queue */
  dcdc_pwr_queueHandle = osMessageQueueNew(2, 2, &dcdc_pwr_queue_attributes);

  /* creation of esp_tx_queue */
  esp_tx_queueHandle = osMessageQueueNew(5, 255, &esp_tx_queue_attributes);

  /* creation of pwrSelCmd_queue */
  pwrSelCmd_queueHandle = osMessageQueueNew(2, 1, &pwrSelCmd_queue_attributes);

  /* creation of uart1_rx_queue */
  uart1_rx_queueHandle = osMessageQueueNew(2, 255, &uart1_rx_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of os_led_blink */
  os_led_blinkHandle = osThreadNew(os_led_blink_tsk, NULL, &os_led_blink_attributes);

  /* creation of esp_msg */
  esp_msgHandle = osThreadNew(esp_msg_tsk, NULL, &esp_msg_attributes);

  /* creation of esp_conn */
  esp_connHandle = osThreadNew(esp_conn_tsk, NULL, &esp_conn_attributes);

  /* creation of pwr_monitor */
  pwr_monitorHandle = osThreadNew(pwr_monitor_tsk, NULL, &pwr_monitor_attributes);

  /* creation of dcdc_ctrl */
  dcdc_ctrlHandle = osThreadNew(dcdc_ctrl_tsk, NULL, &dcdc_ctrl_attributes);

  /* creation of adc_handler */
  adc_handlerHandle = osThreadNew(adc_handler_tsk, NULL, &adc_handler_attributes);

  /* creation of esp_redir */
  esp_redirHandle = osThreadNew(esp_redir_tsk, NULL, &esp_redir_attributes);

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
    osDelay(500);
    HAL_GPIO_TogglePin(OS_STAT_GPIO_Port, OS_STAT_Pin);
  }
  /* USER CODE END os_led_blink_tsk */
}

/* USER CODE BEGIN Header_esp_msg_tsk */
/**
 * @brief Function implementing the esp_msg thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_esp_msg_tsk */
void esp_msg_tsk(void *argument)
{
  /* USER CODE BEGIN esp_msg_tsk */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END esp_msg_tsk */
}

/* USER CODE BEGIN Header_esp_conn_tsk */
/**
 * @brief Function implementing the esp_conn thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_esp_conn_tsk */
void esp_conn_tsk(void *argument)
{
  /* USER CODE BEGIN esp_conn_tsk */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END esp_conn_tsk */
}

/* USER CODE BEGIN Header_pwr_monitor_tsk */
/**
 * @brief Function implementing the pwr_monitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_pwr_monitor_tsk */
void pwr_monitor_tsk(void *argument)
{
  /* USER CODE BEGIN pwr_monitor_tsk */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END pwr_monitor_tsk */
}

/* USER CODE BEGIN Header_dcdc_ctrl_tsk */
/**
 * @brief Function implementing the dcdc_ctrl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_dcdc_ctrl_tsk */
void dcdc_ctrl_tsk(void *argument)
{
  /* USER CODE BEGIN dcdc_ctrl_tsk */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END dcdc_ctrl_tsk */
}

/* USER CODE BEGIN Header_adc_handler_tsk */
/**
 * @brief Function implementing the adc_handler thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_adc_handler_tsk */
void adc_handler_tsk(void *argument)
{
  /* USER CODE BEGIN adc_handler_tsk */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END adc_handler_tsk */
}

/* USER CODE BEGIN Header_esp_redir_tsk */
/**
 * @brief Redirect ESP8266 response to UART1, and send UART1 commands to ESP8266
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_esp_redir_tsk */
void esp_redir_tsk(void *argument)
{
  /* USER CODE BEGIN esp_redir_tsk */
  /* Infinite loop */
  for (;;)
  {
    // check uart1 queue
    if (osMessageQueueGetCount(uart1_rx_queueHandle) > 0)
    {
      uint8_t data[255];
      osMessageQueueGet(uart1_rx_queueHandle, &data, NULL, 0);
      HAL_UART_Transmit(&huart2, data, strlen(data), 100);
    }
    if (osMessageQueueGetCount(esp_rx_queueHandle) > 0)
    {
      uint8_t data[255];
      osMessageQueueGet(esp_rx_queueHandle, &data, NULL, 0);
      printf(">> ESP8266 Sent: \r\n%s\r\n", data);
    }
  }
  /* USER CODE END esp_redir_tsk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
