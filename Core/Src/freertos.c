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
// peripherals
#include <string.h>
#include "lcd.h"
#include "usart.h"
#include "tim.h"
#include "esp.h"
#include "adc.h"
#include "dcdc_pid.h"

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
extern uint8_t os_running;
extern uint32_t adc1_data[6];
/* USER CODE END Variables */
/* Definitions for pwr_monitor */
osThreadId_t pwr_monitorHandle;
const osThreadAttr_t pwr_monitor_attributes = {
  .name = "pwr_monitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for esp_msg */
osThreadId_t esp_msgHandle;
const osThreadAttr_t esp_msg_attributes = {
  .name = "esp_msg",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for esp_send */
osThreadId_t esp_sendHandle;
const osThreadAttr_t esp_send_attributes = {
  .name = "esp_send",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dcdc_ctrl */
osThreadId_t dcdc_ctrlHandle;
const osThreadAttr_t dcdc_ctrl_attributes = {
  .name = "dcdc_ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led_blink */
osThreadId_t led_blinkHandle;
const osThreadAttr_t led_blink_attributes = {
  .name = "led_blink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for esp_rx_queue */
osMessageQueueId_t esp_rx_queueHandle;
const osMessageQueueAttr_t esp_rx_queue_attributes = {
  .name = "esp_rx_queue"
};
/* Definitions for esp_tx_queue */
osMessageQueueId_t esp_tx_queueHandle;
const osMessageQueueAttr_t esp_tx_queue_attributes = {
  .name = "esp_tx_queue"
};
/* Definitions for usr_cmd_queue */
osMessageQueueId_t usr_cmd_queueHandle;
const osMessageQueueAttr_t usr_cmd_queue_attributes = {
  .name = "usr_cmd_queue"
};
/* Definitions for dcdc_param_queue */
osMessageQueueId_t dcdc_param_queueHandle;
const osMessageQueueAttr_t dcdc_param_queue_attributes = {
  .name = "dcdc_param_queue"
};
/* Definitions for tmr_report_pwr */
osTimerId_t tmr_report_pwrHandle;
const osTimerAttr_t tmr_report_pwr_attributes = {
  .name = "tmr_report_pwr"
};
/* Definitions for adc_mutex */
osMutexId_t adc_mutexHandle;
const osMutexAttr_t adc_mutex_attributes = {
  .name = "adc_mutex"
};
/* Definitions for lcd_mutex */
osMutexId_t lcd_mutexHandle;
const osMutexAttr_t lcd_mutex_attributes = {
  .name = "lcd_mutex"
};
/* Definitions for mqttsrv_connected */
osEventFlagsId_t mqttsrv_connectedHandle;
const osEventFlagsAttr_t mqttsrv_connected_attributes = {
  .name = "mqttsrv_connected"
};
/* Definitions for mmc_en */
osEventFlagsId_t mmc_enHandle;
const osEventFlagsAttr_t mmc_en_attributes = {
  .name = "mmc_en"
};
/* Definitions for bkup_en */
osEventFlagsId_t bkup_enHandle;
const osEventFlagsAttr_t bkup_en_attributes = {
  .name = "bkup_en"
};
/* Definitions for pwr_ovld */
osEventFlagsId_t pwr_ovldHandle;
const osEventFlagsAttr_t pwr_ovld_attributes = {
  .name = "pwr_ovld"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t eb_scan();
/* USER CODE END FunctionPrototypes */

void pwr_monitor_tsk(void *argument);
extern void esp_msg_tsk(void *argument);
extern void esp_send_tsk(void *argument);
extern void dcdc_ctrl_tsk(void *argument);
void led_blink_tsk(void *argument);
extern void tmr_report_pwr_clbk(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of adc_mutex */
  adc_mutexHandle = osMutexNew(&adc_mutex_attributes);

  /* creation of lcd_mutex */
  lcd_mutexHandle = osMutexNew(&lcd_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of tmr_report_pwr */
  tmr_report_pwrHandle = osTimerNew(tmr_report_pwr_clbk, osTimerPeriodic, NULL, &tmr_report_pwr_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of esp_rx_queue */
  esp_rx_queueHandle = osMessageQueueNew (8, 255, &esp_rx_queue_attributes);

  /* creation of esp_tx_queue */
  esp_tx_queueHandle = osMessageQueueNew (8, 255, &esp_tx_queue_attributes);

  /* creation of usr_cmd_queue */
  usr_cmd_queueHandle = osMessageQueueNew (5, 2, &usr_cmd_queue_attributes);

  /* creation of dcdc_param_queue */
  dcdc_param_queueHandle = osMessageQueueNew (16, 10, &dcdc_param_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of pwr_monitor */
  pwr_monitorHandle = osThreadNew(pwr_monitor_tsk, NULL, &pwr_monitor_attributes);

  /* creation of esp_msg */
  esp_msgHandle = osThreadNew(esp_msg_tsk, NULL, &esp_msg_attributes);

  /* creation of esp_send */
  esp_sendHandle = osThreadNew(esp_send_tsk, NULL, &esp_send_attributes);

  /* creation of dcdc_ctrl */
  dcdc_ctrlHandle = osThreadNew(dcdc_ctrl_tsk, NULL, &dcdc_ctrl_attributes);

  /* creation of led_blink */
  led_blinkHandle = osThreadNew(led_blink_tsk, NULL, &led_blink_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of mqttsrv_connected */
  mqttsrv_connectedHandle = osEventFlagsNew(&mqttsrv_connected_attributes);

  /* creation of mmc_en */
  mmc_enHandle = osEventFlagsNew(&mmc_en_attributes);

  /* creation of bkup_en */
  bkup_enHandle = osEventFlagsNew(&bkup_en_attributes);

  /* creation of pwr_ovld */
  pwr_ovldHandle = osEventFlagsNew(&pwr_ovld_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_pwr_monitor_tsk */
/**
  * @brief  Function implementing the pwr_monitor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_pwr_monitor_tsk */
void pwr_monitor_tsk(void *argument)
{
  /* USER CODE BEGIN pwr_monitor_tsk */
  uint32_t adc_value_buff[6];
  float mmc_voltage, mmc_current, mmc_power, bkup_voltage, bkup_current, bkup_power, out_voltage, out_current, out_power;
	
	//LCD_Clear();

  /* Infinite loop */
  for(;;)
  {
    osMutexAcquire(adc_mutexHandle, osWaitForever);
    memcpy(adc_value_buff, adc1_data, sizeof(adc_value_buff));
    osMutexRelease(adc_mutexHandle);

    // calculate real-time power info
    mmc_voltage = adc_value_buff[0] * 3.3 / 4095 * 11.0;
    mmc_current = adc_value_buff[1] * 3.3 / 4095 * 11.0;
    mmc_power = mmc_voltage * mmc_current;

    bkup_voltage = adc_value_buff[2] * 3.3 / 4095 * 11.0;
    bkup_current = adc_value_buff[3] * 3.3 / 4095 * 11.0;
    bkup_power = bkup_voltage * bkup_current;

    out_voltage = adc_value_buff[4] * 3.3 / 4095 * 11.0;
    out_current = adc_value_buff[5] * 3.3 / 4095 * 11.0;
    out_power = out_voltage * out_current;

    // display on LCD
    osMutexAcquire(lcd_mutexHandle, osWaitForever);

    LCD_DisplayNumber(0, 20, adc_value_buff[0], 4);
    LCD_DisplayNumber(0, 40, adc_value_buff[3], 4);

    LCD_DisplayNumber(0, 80, adc_value_buff[1], 4);
    LCD_DisplayNumber(0, 100, adc_value_buff[4], 4);

    LCD_DisplayNumber(0, 140, adc_value_buff[2], 4);
    LCD_DisplayNumber(0, 160, adc_value_buff[5], 4);
    osMutexRelease(lcd_mutexHandle);

    osDelay(1000);
  }
  /* USER CODE END pwr_monitor_tsk */
}

/* USER CODE BEGIN Header_led_blink_tsk */
/**
* @brief LED blink, and also the main task for the system
*        It as well handles the panic button.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_blink_tsk */
void led_blink_tsk(void *argument)
{
  /* USER CODE BEGIN led_blink_tsk */
  /* Infinite loop */
  uint8_t eb_handled = 0;
  os_running = 1;
  osTimerStart(tmr_report_pwrHandle, 500);
  HAL_GPIO_WritePin(OS_STAT_GPIO_Port, OS_STAT_Pin, GPIO_PIN_SET);
  for(;;)
  {
    HAL_GPIO_TogglePin(OS_STAT_GPIO_Port, OS_STAT_Pin);
    if (eb_scan())
    {
      LCD_DisplayString(0, 200, "EB Pressed");
      eb_handled = 1;
      HAL_GPIO_WritePin(FAULT_GPIO_Port, FAULT_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(OS_STAT_GPIO_Port, OS_STAT_Pin, GPIO_PIN_RESET);
      osThreadSuspend(dcdc_ctrlHandle);
      // TODO: Cut off power sources and set duty ratio to zero
    }
    while (eb_scan())
    {
      osDelay(1);
    }
    if (!eb_scan() && eb_handled)
    {
      LCD_ClearRect(0, 200, 280, 20);
      HAL_GPIO_WritePin(FAULT_GPIO_Port, FAULT_Pin, GPIO_PIN_RESET);
      osThreadResume(dcdc_ctrlHandle);
      eb_handled = 0;
      // TODO: Check if the power should be resumed??
    }
    osDelay(500);
  }
  /* USER CODE END led_blink_tsk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

