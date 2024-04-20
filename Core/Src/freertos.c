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
} pwrDataQueue_t;

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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart1_rx */
osThreadId_t uart1_rxHandle;
const osThreadAttr_t uart1_rx_attributes = {
  .name = "uart1_rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adc_pwr_mnt */
osThreadId_t adc_pwr_mntHandle;
const osThreadAttr_t adc_pwr_mnt_attributes = {
  .name = "adc_pwr_mnt",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for adc_display */
osThreadId_t adc_displayHandle;
const osThreadAttr_t adc_display_attributes = {
  .name = "adc_display",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uart1_rx_msg */
osMessageQueueId_t uart1_rx_msgHandle;
const osMessageQueueAttr_t uart1_rx_msg_attributes = {
  .name = "uart1_rx_msg"
};
/* Definitions for pwrmmc_queue */
osMessageQueueId_t pwrmmc_queueHandle;
const osMessageQueueAttr_t pwrmmc_queue_attributes = {
  .name = "pwrmmc_queue"
};
/* Definitions for pwrbkup_queue */
osMessageQueueId_t pwrbkup_queueHandle;
const osMessageQueueAttr_t pwrbkup_queue_attributes = {
  .name = "pwrbkup_queue"
};
/* Definitions for pwrout_queue */
osMessageQueueId_t pwrout_queueHandle;
const osMessageQueueAttr_t pwrout_queue_attributes = {
  .name = "pwrout_queue"
};
/* Definitions for uart2_rx_msg */
osMessageQueueId_t uart2_rx_msgHandle;
const osMessageQueueAttr_t uart2_rx_msg_attributes = {
  .name = "uart2_rx_msg"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void os_led_blink_tsk(void *argument);
void uart1_rx_tsk(void *argument);
void adc_pwr_mnt_tsk(void *argument);
void adc_display_tsk(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the queue(s) */
  /* creation of uart1_rx_msg */
  uart1_rx_msgHandle = osMessageQueueNew (2, 255, &uart1_rx_msg_attributes);

  /* creation of pwrmmc_queue */
  pwrmmc_queueHandle = osMessageQueueNew (20, 8, &pwrmmc_queue_attributes);

  /* creation of pwrbkup_queue */
  pwrbkup_queueHandle = osMessageQueueNew (20, 8, &pwrbkup_queue_attributes);

  /* creation of pwrout_queue */
  pwrout_queueHandle = osMessageQueueNew (20, 8, &pwrout_queue_attributes);

  /* creation of uart2_rx_msg */
  uart2_rx_msgHandle = osMessageQueueNew (5, 255, &uart2_rx_msg_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of os_led_blink */
  os_led_blinkHandle = osThreadNew(os_led_blink_tsk, NULL, &os_led_blink_attributes);

  /* creation of uart1_rx */
  uart1_rxHandle = osThreadNew(uart1_rx_tsk, NULL, &uart1_rx_attributes);

  /* creation of adc_pwr_mnt */
  adc_pwr_mntHandle = osThreadNew(adc_pwr_mnt_tsk, NULL, &adc_pwr_mnt_attributes);

  /* creation of adc_display */
  adc_displayHandle = osThreadNew(adc_display_tsk, NULL, &adc_display_attributes);

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
  for (int i = 0;; i++)
  {
    osDelay(1000);
    HAL_GPIO_TogglePin(MMC_STAT_GPIO_Port, MMC_STAT_Pin);
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
    uint8_t data2[255] = {'\0'};
    osDelay(200);
    osStatus_t status;
    status = osMessageQueueGet(uart1_rx_msgHandle, &data2, NULL, 100);
    if (status == osOK)
    {
      printf(">>> MsgQueue Received: %s\n", data2);
    }
  }
  /* USER CODE END uart1_rx_tsk */
}

/* USER CODE BEGIN Header_adc_pwr_mnt_tsk */
/**
 * @brief Calculate power data from ADC data every 1ms.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_adc_pwr_mnt_tsk */
void adc_pwr_mnt_tsk(void *argument)
{
  /* USER CODE BEGIN adc_pwr_mnt_tsk */
  /** ADC Channel Config **
   * IN0: VMMC
   * IN1: VBKUP
   * IN2: VOUT
   * IN3: IMMC
   * IN4: IBKUP
   * IN5: IOUT
   * ******************* */
  // ADC calibration
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data, 6);
  /* Infinite loop */
  for (;;)
  {
    pwrDataQueue_t pwrData;
    uint32_t tmp[6] = {0};
    for (int8_t i = 0; i < 10; i++)
    {
      for (int8_t i = 0; i < 6; i++)
        tmp[i] += adc_data[i];
      
      osDelay(1);
    }
    for (int8_t i = 0; i < 6; i++)
      adc_data[i] = tmp[i] / 10;
    // Calculate voltage in mV
    pwrData.mmc.voltage = adc_data[0] * 3300 / 4095;
    pwrData.bkup.voltage = adc_data[1] * 3300 / 4095;
    pwrData.out.voltage = adc_data[2] * 3300 / 4095;
    // Calculate curren in mA
    pwrData.mmc.current = adc_data[3] * 3300 / 4095;
    pwrData.bkup.current = adc_data[4] * 3300 / 4095;
    pwrData.out.current = adc_data[5] * 3300 / 4095;
    // Calculate power
    pwrData.mmc.power = pwrData.mmc.voltage * pwrData.mmc.current;
    pwrData.bkup.power = pwrData.bkup.voltage * pwrData.bkup.current;
    pwrData.out.power = pwrData.out.voltage * pwrData.out.current;

    osMessageQueuePut(pwrmmc_queueHandle, &pwrData, 0, 0);
  }
  /* USER CODE END adc_pwr_mnt_tsk */
}

/* USER CODE BEGIN Header_adc_display_tsk */
/**
 * @brief Display ADC data on LCD
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_adc_display_tsk */
void adc_display_tsk(void *argument)
{
  /* USER CODE BEGIN adc_display_tsk */
  /* Infinite loop */
  for (;;)
  {
    pwrDataQueue_t pwrData;
    if (osMessageQueueGet(pwrmmc_queueHandle, &pwrData, NULL, 0) == osOK)
    {
      // LCD_Clear();
      // LCD_DisplayNumber(100,100,pwrData.mmc.voltage, 5);
      // LCD_DisplayNumber(100,120,pwrData.mmc.current, 5);
      // LCD_DisplayNumber(100,140,pwrData.mmc.power, 5);
      // LCD_DisplayNumber(100,160,pwrData.bkup.voltage, 5);
      // LCD_DisplayNumber(100,180,pwrData.bkup.current, 5);
      // LCD_DisplayNumber(100,200,pwrData.bkup.power, 5);
      // LCD_DisplayNumber(100,220,pwrData.out.voltage, 5);
      // LCD_DisplayNumber(100,240,pwrData.out.current, 5);
      // LCD_DisplayNumber(100,260,pwrData.out.power, 5);

      // // report to console
      // printf("Queue MMC: V=%d, I=%d, P=%d\r\n", pwrData.mmc.voltage, pwrData.mmc.current, pwrData.mmc.power);
      // printf("Queue BKUP: V=%d, I=%d, P=%d\r\n", pwrData.bkup.voltage, pwrData.bkup.current, pwrData.bkup.power);
      // printf("Queue OUT: V=%d, I=%d, P=%d\r\n", pwrData.out.voltage, pwrData.out.current, pwrData.out.power);
    }
  }
  /* USER CODE END adc_display_tsk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

