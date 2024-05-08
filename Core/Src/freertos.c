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
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// peripherals
#include "adc.h"
#include "dcdc_pid.h"
#include "esp.h"
#include "lcd.h"
#include "os_events.h"
#include "tim.h"
#include "usart.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OVERLOAD_LIMIT 20 // in W
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t os_running;

// ADC data buffer
extern uint32_t adc1_data[6];

// Power data
sysPwrData_t sys_pwr = {0};

/* USER CODE END Variables */
/* Definitions for pwr_monitor */
osThreadId_t pwr_monitorHandle;
const osThreadAttr_t pwr_monitor_attributes = {
    .name = "pwr_monitor",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for esp_msg */
osThreadId_t esp_msgHandle;
const osThreadAttr_t esp_msg_attributes = {
    .name = "esp_msg",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for dcdc_ctrl */
osThreadId_t dcdc_ctrlHandle;
const osThreadAttr_t dcdc_ctrl_attributes = {
    .name = "dcdc_ctrl",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for led_blink */
osThreadId_t led_blinkHandle;
const osThreadAttr_t led_blink_attributes = {
    .name = "led_blink",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for esp_rx_queue */
osMessageQueueId_t esp_rx_queueHandle;
const osMessageQueueAttr_t esp_rx_queue_attributes = {.name = "esp_rx_queue"};
/* Definitions for esp_tx_queue */
osMessageQueueId_t esp_tx_queueHandle;
const osMessageQueueAttr_t esp_tx_queue_attributes = {.name = "esp_tx_queue"};
/* Definitions for usr_cmd_queue */
osMessageQueueId_t usr_cmd_queueHandle;
const osMessageQueueAttr_t usr_cmd_queue_attributes = {.name = "usr_cmd_queue"};
/* Definitions for dcdc_param_queue */
osMessageQueueId_t dcdc_param_queueHandle;
const osMessageQueueAttr_t dcdc_param_queue_attributes = {.name = "dcdc_param_queue"};
/* Definitions for tmr_report_pwr */
osTimerId_t tmr_report_pwrHandle;
const osTimerAttr_t tmr_report_pwr_attributes = {.name = "tmr_report_pwr"};
/* Definitions for adc_mutex */
osMutexId_t adc_mutexHandle;
const osMutexAttr_t adc_mutex_attributes = {.name = "adc_mutex"};
/* Definitions for lcd_mutex */
osMutexId_t lcd_mutexHandle;
const osMutexAttr_t lcd_mutex_attributes = {.name = "lcd_mutex"};
/* Definitions for sys_stat */
osEventFlagsId_t sys_statHandle;
const osEventFlagsAttr_t sys_stat_attributes = {.name = "sys_stat"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t eb_scan();
/* USER CODE END FunctionPrototypes */

void pwr_monitor_tsk(void *argument);
extern void esp_msg_tsk(void *argument);
extern void dcdc_ctrl_tsk(void *argument);
void led_blink_tsk(void *argument);
extern void tmr_report_pwr_clbk(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */
    os_running = 1;
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
    esp_rx_queueHandle = osMessageQueueNew(8, 255, &esp_rx_queue_attributes);

    /* creation of esp_tx_queue */
    esp_tx_queueHandle = osMessageQueueNew(8, 255, &esp_tx_queue_attributes);

    /* creation of usr_cmd_queue */
    usr_cmd_queueHandle = osMessageQueueNew(5, 2, &usr_cmd_queue_attributes);

    /* creation of dcdc_param_queue */
    dcdc_param_queueHandle = osMessageQueueNew(16, 10, &dcdc_param_queue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of pwr_monitor */
    pwr_monitorHandle = osThreadNew(pwr_monitor_tsk, NULL, &pwr_monitor_attributes);

    /* creation of esp_msg */
    esp_msgHandle = osThreadNew(esp_msg_tsk, NULL, &esp_msg_attributes);

    /* creation of dcdc_ctrl */
    dcdc_ctrlHandle = osThreadNew(dcdc_ctrl_tsk, NULL, &dcdc_ctrl_attributes);

    /* creation of led_blink */
    led_blinkHandle = osThreadNew(led_blink_tsk, NULL, &led_blink_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* creation of sys_stat */
    sys_statHandle = osEventFlagsNew(&sys_stat_attributes);

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
    uint32_t adc_value_accumulate[6];

    /* Infinite loop */
    for (;;)
    {
        // get ADC data 10 times
        memset(adc_value_accumulate, 0, sizeof(adc_value_accumulate));
        for (uint8_t i = 0; i < 10; i++)
        {
            // get ADC data
            if (osMutexAcquire(adc_mutexHandle, 0) == osOK)
            {
                memcpy(adc_value_buff, adc1_data, sizeof(adc_value_buff));
                osMutexRelease(adc_mutexHandle);
                for (uint8_t j = 0; j < 6; j++)
                {
                    adc_value_accumulate[j] += adc_value_buff[j];
                }
            }
        }
        // get average ADC data
        for (uint8_t i = 0; i < 6; i++)
        {
            adc_value_buff[i] = adc_value_accumulate[i] / 10;
        }
        // Calculate power data
        sys_pwr.mmc.voltage = adc_value_buff[0] / ADC_COEFFICIENT;
        sys_pwr.bkup.voltage = adc_value_buff[1] / ADC_COEFFICIENT;
        sys_pwr.out.voltage = adc_value_buff[2] / ADC_COEFFICIENT;
        sys_pwr.mmc.current = (2.5 - (adc_value_buff[3] / ADC_COEFFICIENT)) / 0.1;
        sys_pwr.bkup.current = (2.5 - (adc_value_buff[4] / ADC_COEFFICIENT)) / 0.1;
        sys_pwr.out.current = (2.5 - (adc_value_buff[5] / ADC_COEFFICIENT)) / 0.1;
        sys_pwr.mmc.power = sys_pwr.mmc.voltage * sys_pwr.mmc.current;
        sys_pwr.bkup.power = sys_pwr.bkup.voltage * sys_pwr.bkup.current;
        sys_pwr.out.power = sys_pwr.out.voltage * sys_pwr.out.current;

        // Display ADC Data to LCD
        if (osMutexAcquire(lcd_mutexHandle, 0) == osOK)
        {
            // if (0) {
            //  Display ADC Data
            LCD_SetAsciiFont(&ASCII_Font20);
            LCD_SetColor(LCD_BLACK);
            LCD_SetBackColor(LCD_WHITE);
            LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_MMC_Y, sys_pwr.mmc.voltage, 5, 2);
            LCD_DisplayDecimals(LCD_CURRENT_X, LCD_MMC_Y, sys_pwr.mmc.current, 5, 2);
            LCD_DisplayDecimals(LCD_POWER_X, LCD_MMC_Y, sys_pwr.mmc.power, 5, 2);
            LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_BKUP_Y, sys_pwr.bkup.voltage, 5, 2);
            LCD_DisplayDecimals(LCD_CURRENT_X, LCD_BKUP_Y, sys_pwr.bkup.current, 5, 2);
            LCD_DisplayDecimals(LCD_POWER_X, LCD_BKUP_Y, sys_pwr.bkup.power, 5, 2);
            LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_OUT_Y, sys_pwr.out.voltage, 5, 2);
            LCD_DisplayDecimals(LCD_CURRENT_X, LCD_OUT_Y, sys_pwr.out.current, 5, 2);
            LCD_DisplayDecimals(LCD_POWER_X, LCD_OUT_Y, sys_pwr.out.power, 5, 2);
            osMutexRelease(lcd_mutexHandle);
        }

        if (sys_pwr.mmc.power > OVERLOAD_LIMIT || sys_pwr.bkup.power > OVERLOAD_LIMIT ||
            sys_pwr.out.power > OVERLOAD_LIMIT)
        {
            osEventFlagsSet(sys_statHandle, SYS_OVERLD);

            // disable DCDC
            osThreadSuspend(dcdc_ctrlHandle);

            // set timer duty ratio to zero
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

            // display info on LCD
            if (osMutexAcquire(lcd_mutexHandle, osWaitForever) == osOK)
            {
                LCD_SetColor(LCD_RED);
                LCD_SetBackColor(LCD_WHITE);
                LCD_SetTextFont(&ASCII_Font20);
                LCD_FillRect(LCD_SYS_STAT_BOX_X, LCD_SYS_STAT_BOX_Y, LCD_SYS_STAT_BOX_WIDTH, LCD_SYS_STAT_BOX_HEIGHT);
                LCD_DisplayString(LCD_SYS_STAT_OVLD_X, LCD_SYS_STAT_OVLD_Y, "OVERLOAD");
                osMutexRelease(lcd_mutexHandle);
            }
        }
        osDelay(1000);
    }
    /* USER CODE END pwr_monitor_tsk */
}

/* USER CODE BEGIN Header_led_blink_tsk */
/**
 * @brief Function implementing the led_blink thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_led_blink_tsk */
void led_blink_tsk(void *argument)
{
    /* USER CODE BEGIN led_blink_tsk */
    /* Infinite loop */
    uint8_t eb_handled = 0;
    osTimerStart(tmr_report_pwrHandle, 1000);
    HAL_GPIO_WritePin(OS_STAT_GPIO_Port, OS_STAT_Pin, GPIO_PIN_SET);
    for (;;)
    {
        HAL_GPIO_TogglePin(OS_STAT_GPIO_Port, OS_STAT_Pin);
        if (eb_scan())
        {
            eb_handled = 1;
            HAL_GPIO_WritePin(FAULT_GPIO_Port, FAULT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OS_STAT_GPIO_Port, OS_STAT_Pin, GPIO_PIN_RESET);
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_ClearRect(LCD_SYS_STAT_BOX_X, LCD_SYS_STAT_BOX_Y, LCD_SYS_STAT_BOX_WIDTH, LCD_SYS_STAT_BOX_HEIGHT);
            LCD_SetColor(LCD_RED);
            LCD_FillRect(LCD_SYS_STAT_BOX_X, LCD_SYS_STAT_BOX_Y, LCD_SYS_STAT_BOX_WIDTH, LCD_SYS_STAT_BOX_HEIGHT);
            LCD_SetColor(LCD_WHITE);
            LCD_SetBackColor(LCD_RED);
            LCD_DisplayString(LCD_SYS_STAT_EMERGENCY_STOP_X, LCD_SYS_STAT_EMERGENCY_STOP_Y, "EMERGENCY");
            osMutexRelease(lcd_mutexHandle);
            osThreadSuspend(dcdc_ctrlHandle);
            // set timer duty ratio to zero
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            // TODO: Cut off power sources and set duty ratio to zero
        }
        while (eb_scan())
        {
            osDelay(1);
        }
        if (!eb_scan() && eb_handled)
        {
            eb_handled = 0;
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetColor(LCD_GREEN);
            LCD_FillRect(LCD_SYS_STAT_BOX_X, LCD_SYS_STAT_BOX_Y, LCD_SYS_STAT_BOX_WIDTH, LCD_SYS_STAT_BOX_HEIGHT);
            LCD_SetColor(LCD_BLACK);
            LCD_SetBackColor(LCD_GREEN);
            LCD_DisplayString(LCD_SYS_STAT_NORMAL_X, LCD_SYS_STAT_NORMAL_Y, "NORMAL");
            osMutexRelease(lcd_mutexHandle);
            HAL_GPIO_WritePin(FAULT_GPIO_Port, FAULT_Pin, GPIO_PIN_RESET);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            osThreadResume(dcdc_ctrlHandle);
            // TODO: Check if the power should be resumed??
        }
        osDelay(200);
    }
    /* USER CODE END led_blink_tsk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
