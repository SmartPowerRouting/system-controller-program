/* USER CODE BEGIN Header */
/**
 * @file freertos.c
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief FreeRTOS initialization and tasks.
 *        This file is modified based on the original freertos.c file generated by STM32CubeMX.
 * @date 2024-05-11
 *
 * @copyright Copyright (c) 2024 Tiantian Zhong @ Zhejiang University
 *          This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
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
#include "esp.h"
#include "lcd.h"
#include "os_events.h"
#include "queue.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OVERLOAD_LIMIT 3.           // in A
#define UNDERLOAD_VOLTAGE_LIMIT 2.5 // in V
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t os_running;

// ADC data buffer
extern uint32_t adc1_data[4];

// ADC current sensor voltage base
extern float adc_current_base_mmc, adc_current_base_bkup;

// Power data
sysPwrData_t sys_pwr = {0};

/* USER CODE END Variables */
/* Definitions for pwr_monitor */
osThreadId_t pwr_monitorHandle;
const osThreadAttr_t pwr_monitor_attributes = {
    .name = "pwr_monitor",
    .stack_size = 1050 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for esp_msg */
osThreadId_t esp_msgHandle;
const osThreadAttr_t esp_msg_attributes = {
    .name = "esp_msg",
    .stack_size = 1050 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for led_blink */
osThreadId_t led_blinkHandle;
const osThreadAttr_t led_blink_attributes = {
    .name = "led_blink",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for mqtt_msg */
osThreadId_t mqtt_msgHandle;
const osThreadAttr_t mqtt_msg_attributes = {
    .name = "mqtt_msg",
    .stack_size = 1050 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for display_stat */
osThreadId_t display_statHandle;
const osThreadAttr_t display_stat_attributes = {
    .name = "display_stat",
    .stack_size = 1050 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for pwr_switch */
osThreadId_t pwr_switchHandle;
const osThreadAttr_t pwr_switch_attributes = {
    .name = "pwr_switch",
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
/* Definitions for report_pwr_queue */
osMessageQueueId_t report_pwr_queueHandle;
const osMessageQueueAttr_t report_pwr_queue_attributes = {.name = "report_pwr_queue"};
/* Definitions for mqtt_rx_msg_queue */
osMessageQueueId_t mqtt_rx_msg_queueHandle;
const osMessageQueueAttr_t mqtt_rx_msg_queue_attributes = {.name = "mqtt_rx_msg_queue"};
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
/* Definitions for state_machine */
osEventFlagsId_t state_machineHandle;
const osEventFlagsAttr_t state_machine_attributes = {.name = "state_machine"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t eb_scan();
/* USER CODE END FunctionPrototypes */

void pwr_monitor_tsk(void *argument);
extern void esp_msg_tsk(void *argument);
void led_blink_tsk(void *argument);
extern void mqtt_msg_tsk(void *argument);
void sys_stat_tsk(void *argument);
void pwr_switch_tsk(void *argument);
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
    usr_cmd_queueHandle = osMessageQueueNew(4, 50, &usr_cmd_queue_attributes);

    /* creation of report_pwr_queue */
    report_pwr_queueHandle = osMessageQueueNew(1, 50, &report_pwr_queue_attributes);

    /* creation of mqtt_rx_msg_queue */
    mqtt_rx_msg_queueHandle = osMessageQueueNew(4, 255, &mqtt_rx_msg_queue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of pwr_monitor */
    pwr_monitorHandle = osThreadNew(pwr_monitor_tsk, NULL, &pwr_monitor_attributes);

    /* creation of esp_msg */
    esp_msgHandle = osThreadNew(esp_msg_tsk, NULL, &esp_msg_attributes);

    /* creation of led_blink */
    led_blinkHandle = osThreadNew(led_blink_tsk, NULL, &led_blink_attributes);

    /* creation of mqtt_msg */
    mqtt_msgHandle = osThreadNew(mqtt_msg_tsk, NULL, &mqtt_msg_attributes);

    /* creation of display_stat */
    display_statHandle = osThreadNew(sys_stat_tsk, NULL, &display_stat_attributes);

    /* creation of pwr_switch */
    pwr_switchHandle = osThreadNew(pwr_switch_tsk, NULL, &pwr_switch_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* creation of sys_stat */
    sys_statHandle = osEventFlagsNew(&sys_stat_attributes);

    /* creation of state_machine */
    state_machineHandle = osEventFlagsNew(&state_machine_attributes);

    /* USER CODE BEGIN RTOS_EVENTS */
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
    uint8_t sys_state = IDLE_STATE; // state machine
    uint8_t state_change = 0;

    uint16_t voltage_backup_cut_in = 12;
    uint16_t voltage_backup_cut_out = 20;
    uint16_t current_limit = 5;

    user_cmd_t user_cmd = {0};
    user_cmd.voltage_backup_cut_in = 10;
    user_cmd.voltage_backup_cut_out = 12;
    user_cmd.current = 10;

    osTimerStart(tmr_report_pwrHandle, 1000);
    osEventFlagsClear(state_machineHandle, 0xFFFFFFU);
    osEventFlagsSet(state_machineHandle, STATE_MACHINE_IDLE);
    HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_RESET);
    lcd_show_limits(voltage_backup_cut_in, voltage_backup_cut_out, current_limit);
    lcd_show_states(0);

    /* Infinite loop */
    for (;;)
    {
        // report power info to lcd and esp8266 (every one second)
        xQueueOverwrite(report_pwr_queueHandle, &sys_pwr);
        // update smart power routing voltage parameters and overload current limit parameter
        if (osMessageQueueGet(usr_cmd_queueHandle, &user_cmd, NULL, 0) == osOK)
        {
            if (user_cmd.voltage_backup_cut_in > 0) // 0 means no change
            {
                voltage_backup_cut_in = user_cmd.voltage_backup_cut_in / 100;
            }
            if (user_cmd.voltage_backup_cut_out > 0) // 0 means no change
            {
                voltage_backup_cut_out = user_cmd.voltage_backup_cut_out / 100;
            }
            if (user_cmd.current > 0) // 0 means no change
            {
                current_limit = user_cmd.current / 100;
            }
            if (sys_state != EB_STATE)
            {
                printf("User command received\r\n");
                state_change = 1;
                switch (user_cmd.mode)
                {
                case (CMD_PWR_OFF - '0'):
                    printf("CMD_PWR_OFF\r\n");
                    sys_state = IDLE_STATE;
                    lcd_show_states(0);
                    break;
                case (CMD_SMART_PWR_ROUTING - '0'):
                    printf("CMD_SMART_PWR_ROUTING\r\n");
                    sys_state = PWR_ROUTING_STATE;
                    lcd_show_states(1);
                    break;
                case (CMD_PWR_FORCE_PRIMARY - '0'):
                    printf("CMD_PWR_FORCE_PRIMARY\r\n");
                    sys_state = PWR_FORCE_PRIMARY_STATE;
                    lcd_show_states(2);
                    break;
                case (CMD_PWR_FORCE_BACKUP - '0'):
                    printf("CMD_PWR_FORCE_BACKUP\r\n");
                    sys_state = PWR_FORCE_BACKUP_STATE;
                    lcd_show_states(3);
                    break;
                default:
                    break;
                }
            }
            osMessageQueuePut(esp_tx_queueHandle, "AT+MQTTPUB=0,\"system/response\",1,2,0\r\n", 0, 500);
            lcd_show_limits(voltage_backup_cut_in, voltage_backup_cut_out, current_limit);
        }

        // get ADC data 20 times
        memset(adc_value_accumulate, 0, sizeof(adc_value_accumulate));
        for (uint8_t i = 0; i < 20; i++)
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
            osDelay(10);
        }
        // get average ADC data
        for (uint8_t i = 0; i < 6; i++)
        {
            adc_value_buff[i] = adc_value_accumulate[i] / 20;
        }
        // Calculate power data
        sys_pwr.mmc.voltage = adc_value_buff[0] / ADC_COEFFICIENT * ADC_COEFFICIENT_VOLTAGE_MMC;
        sys_pwr.bkup.voltage = adc_value_buff[1] / ADC_COEFFICIENT * ADC_COEFFICIENT_VOLTAGE_BKUP;
        sys_pwr.mmc.current = (adc_current_base_mmc - (adc_value_buff[3] / ADC_COEFFICIENT)) / 0.1;
        sys_pwr.bkup.current = (adc_current_base_bkup - (adc_value_buff[4] / ADC_COEFFICIENT)) / 0.1;
        sys_pwr.mmc.power = sys_pwr.mmc.voltage * sys_pwr.mmc.current;
        sys_pwr.bkup.power = sys_pwr.bkup.voltage * sys_pwr.bkup.current;

        //  overload protection
        if (sys_pwr.mmc.current > current_limit || sys_pwr.bkup.current > current_limit)
        {
            sys_state = OVERLOAD_STATE;
            osEventFlagsSet(sys_statHandle, EVENT_OVERLOAD);
            state_change = 1;
            osDelay(50);
        }

        // emergency button
        if (eb_scan() && sys_state != EB_STATE) // EB pressed
        {
            sys_state = EB_STATE;
            osEventFlagsSet(sys_statHandle, EVENT_EB);
            state_change = 1;
            osDelay(50);
        }
        if (!eb_scan() && sys_state == EB_STATE)
        {
            sys_state = IDLE_STATE;
            osEventFlagsSet(sys_statHandle, EVENT_PWR_OFF);
            state_change = 1;
            osDelay(50);
        }

        if (state_change)
        {
            printf("State change detected\r\n");
            state_change = 0;
            switch (sys_state)
            {
            case IDLE_STATE: {
                printf("IDLE_STATE SET\r\n");
                sys_pwr.pwr_src = PWR_SRC_OFF;
                osEventFlagsSet(sys_statHandle, EVENT_PWR_OFF);
                osEventFlagsSet(state_machineHandle, STATE_MACHINE_IDLE);
                break;
            }
            case PWR_ROUTING_STATE: {
                printf("PWR_ROUTING_STATE SET\r\n");
                // By default connect MMC to output if possible
                if (sys_pwr.mmc.voltage > voltage_backup_cut_in)
                {
                    sys_pwr.pwr_src = PWR_SRC_MMC;
                    osEventFlagsSet(sys_statHandle, EVENT_MMC_EN);
                }
                else if (sys_pwr.mmc.voltage < voltage_backup_cut_in)
                {
                    sys_pwr.pwr_src = PWR_SRC_BKUP;
                    osEventFlagsSet(sys_statHandle, EVENT_BKUP_EN);
                }
                break;
            }
            case PWR_FORCE_PRIMARY_STATE: {
                printf("PWR_FORCE_PRIMARY_STATE SET\r\n");
                sys_pwr.pwr_src = PWR_SRC_MMC;
                osEventFlagsSet(sys_statHandle, EVENT_MMC_EN);
                break;
            }
            case PWR_FORCE_BACKUP_STATE: {
                printf("PWR_FORCE_BACKUP_STATE SET\r\n");
                sys_pwr.pwr_src = PWR_SRC_BKUP;
                osEventFlagsSet(sys_statHandle, EVENT_BKUP_EN);
                break;
            }
            case OVERLOAD_STATE: {
                printf("OVERLOAD_STATE SET\r\n");
                sys_pwr.pwr_src = PWR_SRC_OFF;
                osEventFlagsSet(sys_statHandle, EVENT_OVERLOAD);
                osEventFlagsSet(state_machineHandle, STATE_MACHINE_OVERLD);
                break;
            }
            case EB_STATE: {
                printf("EB_STATE SET\r\n");
                sys_pwr.pwr_src = PWR_SRC_OFF;
                osEventFlagsSet(sys_statHandle, EVENT_EB);
                osEventFlagsSet(state_machineHandle, STATE_MACHINE_EB);
                break;
            }
            default:
                break;
            }
        }

        if (sys_state != PWR_ROUTING_STATE)
        {
            continue;
        }

        // Power Routing Policy Starts Here
        if (sys_pwr.mmc.voltage < voltage_backup_cut_in && sys_pwr.pwr_src != PWR_SRC_BKUP)
        {
            sys_pwr.pwr_src = PWR_SRC_BKUP;
            osEventFlagsSet(sys_statHandle, EVENT_BKUP_EN);
        }
        else if (sys_pwr.mmc.voltage > voltage_backup_cut_out && sys_pwr.pwr_src != PWR_SRC_MMC)
        {
            sys_pwr.pwr_src = PWR_SRC_MMC;
            osEventFlagsSet(sys_statHandle, EVENT_MMC_EN);
        }

        osDelay(10);
    }
    /* USER CODE END pwr_monitor_tsk */
}

/* USER CODE BEGIN Header_led_blink_tsk */
/**
 * @brief Blink the OS_STAT LED to indicate FreeRTOS is running.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_led_blink_tsk */
void led_blink_tsk(void *argument)
{
    /* USER CODE BEGIN led_blink_tsk */
    /* Infinite loop */
    HAL_GPIO_WritePin(OS_STAT_GPIO_Port, OS_STAT_Pin, GPIO_PIN_SET);
    for (;;)
    {
        HAL_GPIO_TogglePin(OS_STAT_GPIO_Port, OS_STAT_Pin);
        osDelay(500);
    }
    /* USER CODE END led_blink_tsk */
}

/* USER CODE BEGIN Header_sys_stat_tsk */
/**
 * @brief Send state warning to MQTT broker
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_sys_stat_tsk */
void sys_stat_tsk(void *argument)
{
    /* USER CODE BEGIN sys_stat_tsk */
    uint8_t buff[256] = {0};
    uint8_t ovld_sent = 0, eb_sent = 0, normal_sent = 0;
    uint8_t lcd_normal_handled = 0, lcd_bkup_handled = 0;
    uint32_t state_machine_flags = 0;
    /* Infinite loop */
    for (;;)
    {
        osEventFlagsWait(sys_statHandle, EVENT_MQTT_CONN_STAT, osFlagsNoClear,
                         osWaitForever); // wait for mqtt connection
        state_machine_flags =
            osEventFlagsWait(state_machineHandle, STATE_MACHINE_EB | STATE_MACHINE_OVERLD | STATE_MACHINE_IDLE,
                             osFlagsWaitAny, osWaitForever);
        if ((state_machine_flags & STATE_MACHINE_OVERLD))
        {
            printf("SM Overload detected\r\n");
            sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"%d\",%d,0\r\n", MQTT_TOPIC_WARN, MQTT_WARN_OVERLOAD, MQTT_QOS2);
            osMessageQueuePut(esp_tx_queueHandle, buff, 0, 500);
        }
        else if ((state_machine_flags & STATE_MACHINE_EB))
        {
            printf("SM EB detected\r\n");
            sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"%d\",%d,0\r\n", MQTT_TOPIC_WARN, MQTT_WARN_EB_PRESSED, MQTT_QOS2);
            osMessageQueuePut(esp_tx_queueHandle, buff, 0, 500);
        }
        else if ((state_machine_flags & STATE_MACHINE_IDLE))
        {
            printf("SM idle detected\r\n");
            sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"%d\",%d,0\r\n", MQTT_TOPIC_WARN, MQTT_WARN_NORMAL, MQTT_QOS2);
            osMessageQueuePut(esp_tx_queueHandle, buff, 0, 500);
        }
        osDelay(10);
    }
    /* USER CODE END sys_stat_tsk */
}

/* USER CODE BEGIN Header_pwr_switch_tsk */
/**
 * @brief Send control signals to relays.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_pwr_switch_tsk */
void pwr_switch_tsk(void *argument)
{
    /* USER CODE BEGIN pwr_switch_tsk */
    uint32_t event_flags;
    /* Infinite loop */
    for (;;)
    {
        event_flags =
            osEventFlagsWait(sys_statHandle, EVENT_MMC_EN | EVENT_BKUP_EN | EVENT_PWR_OFF | EVENT_OVERLOAD | EVENT_EB,
                             osFlagsWaitAny, osWaitForever);
        {
            printf("Event received\r\n");
            if (event_flags & EVENT_MMC_EN)
            {
                HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_SET);
                osDelay(100);
                HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_RESET);
                lcd_show_normal();
            }
            else if (event_flags & EVENT_BKUP_EN)
            {
                HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_RESET);
                osDelay(100);
                HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_SET);
                lcd_show_backup();
            }
            else if (event_flags & EVENT_PWR_OFF)
            {
                HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_RESET);
                osDelay(100);
                HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_RESET);
                lcd_show_idle();
            }
            else if (event_flags & EVENT_OVERLOAD)
            {
                HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_RESET);
                osDelay(100);
                HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_RESET);
                lcd_show_overload();
            }
            else if (event_flags & EVENT_EB)
            {
                HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_RESET);
                lcd_show_eb();
            }
        }
    }
    /* USER CODE END pwr_switch_tsk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
