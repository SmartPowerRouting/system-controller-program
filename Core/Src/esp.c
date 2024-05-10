/**
 * @file esp.c
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief
 * @version 1.0
 * @date 2024-05-10
 *
 * @copyright Copyright (c) 2024 Tiantian Zhong @ Zhejiang University
 *            This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
 */

// Peripherals
#include "esp.h"
#include "adc.h"
#include "lcd.h"

// string
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include "cmsis_os.h"
#include "os_events.h"
#include "queue.h"
#include "task.h"

// Network info
#include "srv_info_private.h"

uint8_t esp_response[256] = {0};

// mqtt client info
#define MQTT_CLIENT_ID "WindPwr"
#define MQTT_KEEPALIVE 60

// mqtt QoS
#define MQTT_QOS0 0
#define MQTT_QOS1 1
#define MQTT_QOS2 2

// mqtt topic
#define MQTT_TOPIC_STATUS "system/status"
#define MQTT_TOPIC_WARN "system/warning"
#define MQTT_TOPIC_USR_CMD "usr/cmd"

// mqtt lwt msg
#define MQTT_LWT_MSG "-1"

extern uint8_t uart2_rx_data[256]; // UART2 DMA buffer
extern uint8_t uart2_rx_flag;      // Flag to indicate that UART2 DMA has received data
uint8_t mqtt_recv_topic[256];      // MQTT received topic
uint8_t mqtt_recv_msg[256];        // MQTT received message

extern uint32_t adc1_data[6]; // ADC data

// Message queues
extern osMessageQueueId_t esp_rx_queueHandle;     // ESP message queue
extern osMessageQueueId_t esp_tx_queueHandle;     // MQTT message queue
extern osMessageQueueId_t usr_cmd_queueHandle;    // User command queue
extern osMessageQueueId_t report_pwr_queueHandle; // Power report queue
extern osMessageQueueId_t mqtt_rx_msg_queueHandle; // MQTT received message queue

// Tasks
extern osThreadId_t esp_msg_tskHandle; // ESP message task

// Events
extern osEventFlagsId_t sys_statHandle;

// mutexes
extern osMutexId_t adc_mutexHandle;
extern osMutexId_t lcd_mutexHandle;

// semaphores
extern osSemaphoreId_t report_pwr_semphrHandle;

/**
 * @brief ESP-12F initialization (in OS)
 *
 */
void esp_init_os(void)
{
    uint8_t cmd[256] = {0};
    uint8_t esp_response_buff[256] = {0}; // ESP response buffer (privately
                                          // used in this function)
    // Perform reset
    osDelay(1000);
    HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_RESET);
    osEventFlagsClear(sys_statHandle, WIFI_CONN_STAT | MQTT_CONN_STAT);

    // Set UI
    osMutexAcquire(lcd_mutexHandle, osWaitForever);
    LCD_SetAsciiFont(&ASCII_Font16);
    LCD_SetBackColor(LCD_WHITE);
    LCD_SetColor(LCD_BLACK);
    LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
    LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "Initializing...");
    osMutexRelease(lcd_mutexHandle);

    // randomly send something to avoid the first response from ESP8266
    sprintf(cmd, "AT\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    osDelay(1000);

    // restore factory settings
    sprintf(cmd, "AT+RESTORE\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    osDelay(2000);

    // turn off echo
    sprintf(cmd, "ATE0\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    osDelay(1000);

    osMessageQueueReset(esp_rx_queueHandle); // empty the message queue

    // Set station mode
    sprintf(cmd, "AT+CWMODE=1\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 5000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "OK") != NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
            LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "Connecting...");
            osMutexRelease(lcd_mutexHandle);
        }
        else
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
            LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "CWMODE ERR");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_SetBackColor(LCD_WHITE);
        LCD_SetColor(LCD_BLACK);
        LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
        LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "CWMODE Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
    osDelay(1000);

    // MQTT USR CFGuration
    sprintf(cmd, "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n", MQTT_CLIENT_ID, MQTT_USER, MQTT_PWD);
    HAL_UART_Transmit_DMA(&huart1, cmd, strlen(cmd));
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 2000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "OK") != NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "USR CFG OK");
            osMutexRelease(lcd_mutexHandle);
        }
        else
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "USR CFG ERR");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_SetBackColor(LCD_WHITE);
        LCD_SetColor(LCD_BLACK);
        LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
        LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "USR CFG Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
    osDelay(1000);

    // MQTT connection config
    // AT+MQTTCONNCFG=<LinkID>,<keepalive>,<disable_clean_session>,<"lwt_topic">,<"lwt_msg">,<lwt_qos>,<lwt_retain>
    sprintf(cmd, "AT+MQTTCONNCFG=0,15,1,\"%s\",\"%s\",%d,1\r\n", MQTT_TOPIC_WARN, MQTT_LWT_MSG, MQTT_QOS2);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 2000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "OK") != NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "CONN CFG OK");
            osMutexRelease(lcd_mutexHandle);
        }
        else
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "CONN CFG ERR");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_SetBackColor(LCD_WHITE);
        LCD_SetColor(LCD_BLACK);
        LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
        LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "CONN CFG Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }

    // Connect to WiFi
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", (uint8_t *)WIFI_SSID, (uint8_t *)WIFI_PWD);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 8000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "WIFI CONNECTED") != NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
            LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, WIFI_SSID);
            osMutexRelease(lcd_mutexHandle);
            HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_SET);
            osEventFlagsSet(sys_statHandle, WIFI_CONN_STAT);
        }
        else
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
            LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "Error");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_SetBackColor(LCD_WHITE);
        LCD_SetColor(LCD_BLACK);
        LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
        LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
    osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, osWaitForever); // WIFI GOT IP \r\n OK
    osDelay(1000);

    // connect to MQTT broker
    sprintf(cmd, "AT+MQTTCONN=0,\"%s\",%d,1\r\n", MQTT_BROKER, MQTT_PORT); // enable auto reconnect
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 3000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "OK") != NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, MQTT_BROKER);
            LCD_DisplayString(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, MQTT_CLIENT_ID);
            osMutexRelease(lcd_mutexHandle);
            HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_SET);
        }
        else
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "MQTT Error");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_SetBackColor(LCD_WHITE);
        LCD_SetColor(LCD_BLACK);
        LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
        LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "MQTT Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
    osDelay(1000);

    // MQTT subscribe to a topic
    sprintf(cmd, "AT+MQTTSUB=0,\"%s\",2\r\n", MQTT_TOPIC_USR_CMD);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 1000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "OK") == NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_SetBackColor(LCD_WHITE);
            LCD_SetColor(LCD_BLACK);
            LCD_ClearRect(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, LCD_Width - LCD_MQTT_CLNT_X, 16);
            LCD_DisplayString(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, "Subscr. Error");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_SetBackColor(LCD_WHITE);
        LCD_SetColor(LCD_BLACK);
        LCD_ClearRect(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, LCD_Width - LCD_MQTT_CLNT_X, 16);
        LCD_DisplayString(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, "Subscr. Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
    osEventFlagsSet(sys_statHandle, MQTT_CONN_STAT);
}

// FreeRTOS tasks
/**
 * @brief ESP response analysis.
 *
 * @param argument
 */
void esp_msg_tsk(void *argument)
{
    esp_init_os();
    for (;;)
    {
        osDelay(50);
        uint8_t buff[256] = {0};
        uint8_t dummy; // dummy variable to store the message length
        // Send Message
        if (osMessageQueueGet(esp_tx_queueHandle, buff, NULL, 0) == osOK)
        {
            HAL_UART_Transmit_DMA(&huart2, buff, strlen(buff));
            osDelay(200);
        }

        // Receive message
        if (osMessageQueueGet(esp_rx_queueHandle, buff, NULL, 0) == osOK)
        {
            // Case a: WIFI Disconnected
            // In this case, set system event flag
            if (strstr((char *)buff, "WIFI DISCONNECT") != NULL)
            {
                osEventFlagsClear(sys_statHandle, WIFI_CONN_STAT);
                HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_RESET);

                // display on LCD
                osMutexAcquire(lcd_mutexHandle, osWaitForever);
                LCD_SetAsciiFont(&ASCII_Font16);
                LCD_SetBackColor(LCD_WHITE);
                LCD_SetColor(LCD_BLACK);
                LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
                LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "Disconnected");
                osMutexRelease(lcd_mutexHandle);
            }

            // Case b: MQTT Disconnected
            if (strstr((char *)buff, "MQTTDISCONNECTED") != NULL)
            {
                osEventFlagsClear(sys_statHandle, MQTT_CONN_STAT);
                HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_RESET);

                // display on LCD
                osMutexAcquire(lcd_mutexHandle, osWaitForever);
                LCD_SetAsciiFont(&ASCII_Font16);
                LCD_SetBackColor(LCD_WHITE);
                LCD_SetColor(LCD_BLACK);
                LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
                LCD_ClearRect(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, LCD_Width - LCD_MQTT_CLNT_X, 16);
                osMutexRelease(lcd_mutexHandle);
            }

            // Case c: WIFI reconnected
            if (strstr((char *)buff, "WIFI GOT IP") != NULL)
            {
                osEventFlagsSet(sys_statHandle, WIFI_CONN_STAT);
                HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_SET);

                // display on LCD
                osMutexAcquire(lcd_mutexHandle, osWaitForever);
                LCD_SetAsciiFont(&ASCII_Font16);
                LCD_SetBackColor(LCD_WHITE);
                LCD_SetColor(LCD_BLACK);
                LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
                LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, WIFI_SSID);
                osMutexRelease(lcd_mutexHandle);
            }

            // Case d: MQTT reconnected
            if (strstr((char *)buff, "MQTTCONNECTED") != NULL)
            {
                osEventFlagsSet(sys_statHandle, MQTT_CONN_STAT);
                HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_SET);

                // display on LCD
                osMutexAcquire(lcd_mutexHandle, osWaitForever);
                LCD_SetAsciiFont(&ASCII_Font16);
                LCD_SetBackColor(LCD_WHITE);
                LCD_SetColor(LCD_BLACK);
                LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
                LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, MQTT_BROKER);
                LCD_DisplayString(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, MQTT_CLIENT_ID);
                osMutexRelease(lcd_mutexHandle);
            }
        }
    }
}

/**
 * @brief Real-time power transmission to ESP using a Timer, this is the
 * callback fcn
 *
 * @param argument
 */
void tmr_report_pwr_clbk(void *argument)
{
    // For adc data calculating
    uint16_t vmmc, vbackup, vout, immc, ibackup, iout, pmmc, pbackup, pout;
    uint32_t adc_to_send[6];
    uint8_t buff[256] = {0};
    uint32_t sys_event = 0;
    uint8_t pwr_src = 0; // 0: off, 1: MMC, 2: backup
    sysPwrData_t pwrData_buff;

    // get power data
    if (osMessageQueueGet(report_pwr_queueHandle, &pwrData_buff, NULL, 100) != osOK)
    {
        return;
    }

    // report to LCD screen
    if (osMutexAcquire(lcd_mutexHandle, 500) == osOK)
    {
        LCD_SetAsciiFont(&ASCII_Font20);
        LCD_SetColor(LCD_BLACK);
        LCD_SetBackColor(LCD_WHITE);
        LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_MMC_Y, pwrData_buff.mmc.voltage, 5, 2);
        LCD_DisplayDecimals(LCD_CURRENT_X, LCD_MMC_Y, pwrData_buff.mmc.current, 5, 2);
        LCD_DisplayDecimals(LCD_POWER_X, LCD_MMC_Y, pwrData_buff.mmc.power, 5, 2);
        LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_BKUP_Y, pwrData_buff.bkup.voltage, 5, 2);
        LCD_DisplayDecimals(LCD_CURRENT_X, LCD_BKUP_Y, pwrData_buff.bkup.current, 5, 2);
        LCD_DisplayDecimals(LCD_POWER_X, LCD_BKUP_Y, pwrData_buff.bkup.power, 5, 2);
        LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_OUT_Y, pwrData_buff.out.voltage, 5, 2);
        LCD_DisplayDecimals(LCD_CURRENT_X, LCD_OUT_Y, pwrData_buff.out.current, 5, 2);
        LCD_DisplayDecimals(LCD_POWER_X, LCD_OUT_Y, pwrData_buff.out.power, 6, 2);
        osMutexRelease(lcd_mutexHandle);
    }

    // get currently used power source
    sys_event = osEventFlagsGet(sys_statHandle);

    // get MQTT connection status; if not connected then return
    if ((sys_event & MQTT_CONN_STAT) == 0 || (sys_event & WIFI_CONN_STAT) == 0)
    {
        return;
    }

    if (sys_event & MMC_EN)
    {
        pwr_src = 1;
    }
    else if (sys_event & BKUP_EN)
    {
        pwr_src = 2;
    }
    else
    {
        pwr_src = 0;
    }

    // Send data to ESP8266
    vmmc = voltage_current_format(pwrData_buff.mmc.voltage);
    vbackup = voltage_current_format(pwrData_buff.bkup.voltage);
    vout = voltage_current_format(pwrData_buff.out.voltage);
    immc = voltage_current_format(pwrData_buff.mmc.current);
    ibackup = voltage_current_format(pwrData_buff.bkup.current);
    iout = voltage_current_format(pwrData_buff.out.current);
    pmmc = power_format(pwrData_buff.mmc.power);
    pbackup = power_format(pwrData_buff.bkup.power);
    pout = power_format(pwrData_buff.out.power);

    sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"0 %d %d %d %d %d %d %d %d %d %d\",2,0\r\n", (char *)MQTT_TOPIC_STATUS, vmmc,
            vbackup, vout, immc, ibackup, iout, pmmc, pbackup, pout, pwr_src);

    osMessageQueuePut(esp_tx_queueHandle, buff, 0, 0);
}


/**
 * @brief Extract user response from MQTT message.
 * 
 * @param argument 
 */
void mqtt_msg_tsk(void *argument)
{
    for(;;)
    {
        osDelay(50);
    }
}

/**
 * @brief Format voltage and current data to 2 decimal places (12.34V-->1234)
 *
 * @param f
 * @return uint16_t
 */
uint16_t voltage_current_format(float f)
{
    return (uint16_t)(f * 100) % 1000;
}

/**
 * @brief Format power data to 1 decimal places (92.1W-->921)
 *
 * @param f
 * @return uint16_t
 */
uint16_t power_format(float f)
{
    return (uint16_t)(f * 10) % 10000;
}