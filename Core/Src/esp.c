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
#include <stdio.h>
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

extern uint8_t uart2_rx_data[256]; // UART2 DMA buffer
extern uint8_t uart2_rx_flag;      // Flag to indicate that UART2 DMA has received data

extern uint32_t adc1_data[6]; // ADC data

// Message queues
extern osMessageQueueId_t esp_rx_queueHandle;      // ESP message queue
extern osMessageQueueId_t esp_tx_queueHandle;      // MQTT message queue
extern osMessageQueueId_t usr_cmd_queueHandle;     // User command queue
extern osMessageQueueId_t report_pwr_queueHandle;  // Power report queue
extern osMessageQueueId_t mqtt_rx_msg_queueHandle; // MQTT received message queue

// Tasks
extern osThreadId_t esp_msg_tskHandle; // ESP message task

// Events
extern osEventFlagsId_t sys_statHandle;
extern osEventFlagsId_t state_machineHandle;

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
    osEventFlagsClear(sys_statHandle, EVENT_WIFI_CONN_STAT | EVENT_MQTT_CONN_STAT);

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
    sprintf(cmd, "AT+MQTTCONNCFG=0,%d,1,\"%s\",\"%d\",%d,1\r\n", MQTT_KEEPALIVE, MQTT_TOPIC_WARN, MQTT_LWT_MSG,
            MQTT_QOS2);
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
            osEventFlagsSet(sys_statHandle, EVENT_WIFI_CONN_STAT);
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
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "CONN ERR");
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
        LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "CONN Timeout");
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
            LCD_DisplayString(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, "SUBSCR ERR");
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
        LCD_DisplayString(LCD_MQTT_CLNT_X, LCD_MQTT_CLNT_Y, "SUBSCR Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }

    osMessageQueueReset(esp_tx_queueHandle); // empty the message queue

    // send system online msg
    sprintf(cmd, "AT+MQTTPUB=0,\"%s\",\"%d\",2,1\r\n", MQTT_TOPIC_WARN, MQTT_WARN_NORMAL);
    osMessageQueuePut(esp_tx_queueHandle, cmd, 0, 0);
    osEventFlagsSet(sys_statHandle, EVENT_MQTT_CONN_STAT);
}

/*
 * -------------------- FreeRTOS Tasks --------------------
 */

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
        uint8_t dummy;              // dummy variable to store the message length
        uint8_t mqtt_rx_topic[256]; // MQTT received topic
        uint8_t mqtt_rx_msg[256];   // MQTT received message
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
                osEventFlagsClear(sys_statHandle, EVENT_WIFI_CONN_STAT);
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
            if (strstr((char *)buff, "ECTED:0") != NULL)
            {
                osEventFlagsClear(sys_statHandle, EVENT_MQTT_CONN_STAT);
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
                osEventFlagsSet(sys_statHandle, EVENT_WIFI_CONN_STAT);
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
                osEventFlagsSet(sys_statHandle, EVENT_MQTT_CONN_STAT);
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

            // Case e: Received MQTT msg
            if (strstr((char *)buff, "+MQTTSUBRECV:0") != NULL)
            {
                osMessageQueuePut(mqtt_rx_msg_queueHandle, buff, 0, osWaitForever);
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
        LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_MMC_Y, pwrData_buff.mmc.voltage, 3, 1);
        LCD_DisplayDecimals(LCD_CURRENT_X, LCD_MMC_Y, pwrData_buff.mmc.current, 3, 1);
        LCD_DisplayDecimals(LCD_POWER_X, LCD_MMC_Y, pwrData_buff.mmc.power, 3, 1);
        LCD_DisplayDecimals(LCD_VOTAGE_X, LCD_BKUP_Y, pwrData_buff.bkup.voltage, 3, 1);
        LCD_DisplayDecimals(LCD_CURRENT_X, LCD_BKUP_Y, pwrData_buff.bkup.current, 3, 1);
        LCD_DisplayDecimals(LCD_POWER_X, LCD_BKUP_Y, pwrData_buff.bkup.power, 3, 1);
        osMutexRelease(lcd_mutexHandle);
    }

    // get currently used power source
    sys_event = osEventFlagsGet(sys_statHandle);

    // get MQTT connection status; if not connected then return
    if ((sys_event & EVENT_MQTT_CONN_STAT) == 0 || (sys_event & EVENT_WIFI_CONN_STAT) == 0)
    {
        return;
    }
    
    // Send data to ESP8266
    vmmc = voltage_current_format(pwrData_buff.mmc.voltage);
    vbackup = voltage_current_format(pwrData_buff.bkup.voltage);
    immc = voltage_current_format(pwrData_buff.mmc.current);
    ibackup = voltage_current_format(pwrData_buff.bkup.current);
    pmmc = power_format(pwrData_buff.mmc.power);
    pbackup = power_format(pwrData_buff.bkup.power);
    pwr_src = pwrData_buff.pwr_src;

    sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"0 %d %d %d %d %d %d %d\",2,0\r\n", (char *)MQTT_TOPIC_STATUS, vmmc, vbackup,
            immc, ibackup, pmmc, pbackup, pwr_src);

    osMessageQueuePut(esp_tx_queueHandle, buff, 0, 0);

    // send state machine flags
    sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"%d\",2,0\r\n", "dev/state", osEventFlagsGet(state_machineHandle));
    osMessageQueuePut(esp_tx_queueHandle, buff, 0, 0);
}

/**
 * @brief Extract user response from MQTT message.
 *
 * @param argument
 */
void mqtt_msg_tsk(void *argument)
{
    uint8_t buff[255] = {0};
    uint8_t buff_msg[60] = {0};
    uint8_t buff_topic[20] = {0};
    user_cmd_t usr_cmd;
    for (;;)
    {
        // reset usr_cmd
        usr_cmd.mode = 0;
        usr_cmd.voltage_backup_cut_in = 0;
        usr_cmd.voltage_backup_cut_out = 0;
        usr_cmd.current = 0;

        // get message from ESP8266
        osMessageQueueGet(mqtt_rx_msg_queueHandle, buff, NULL, osWaitForever);

        // check if the message is a valid mqtt message
        if (sscanf((char *)buff, "+MQTTSUBRECV:0,\"%[^\"]\",%*d,%[^\r]", &buff_topic, &buff_msg) != NULL)
        {
            if (strcmp((uint8_t *)buff_topic, (uint8_t *)MQTT_TOPIC_USR_CMD) == 0)
            {
                usr_cmd.mode = buff_msg[0] - '0';
                if (buff_msg[0] == CMD_PWR_OFF) // force power off
                {
                    // transmit parameters to power monitor task
                    osMessageQueuePut(usr_cmd_queueHandle, &usr_cmd, 0, 500);
                }
                if (buff_msg[0] == CMD_SMART_PWR_ROUTING) // smart power routing
                {
                    // transmit parameters to power monitor task
                    sscanf(buff_msg, "1 %d %d %d", &usr_cmd.voltage_backup_cut_in, &usr_cmd.voltage_backup_cut_out,
                           &usr_cmd.current);
                    osMessageQueuePut(usr_cmd_queueHandle, &usr_cmd, 0, 500);
                }
                if (buff_msg[0] == CMD_PWR_FORCE_PRIMARY) // force primary
                {
                    sscanf(buff_msg, "2 %d", &usr_cmd.current);
                    // transmit parameters to power monitor task
                    osMessageQueuePut(usr_cmd_queueHandle, &usr_cmd, 0, 500);
                }
                if (buff_msg[0] == CMD_PWR_FORCE_BACKUP) // force backup
                {
                    sscanf(buff_msg, "3 %d", &usr_cmd.current);
                    osMessageQueuePut(usr_cmd_queueHandle, &usr_cmd, 0, 500);
                }
            }
            osDelay(10);
        }
    }
}

/**
 ** ---------------------- Utility functions -------------------------
 */

/**
 * @brief Format voltage and current data to 2 decimal places (12.34V-->1234)
 *
 * @param f
 * @return uint16_t
 */
uint16_t voltage_current_format(float f)
{
    return (uint16_t)(f * 100) % 10000;
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
