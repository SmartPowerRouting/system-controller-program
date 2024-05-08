/**
 * @file esp.c
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-04-27
 *
 * @copyright Copyright (c) 2024 This file is part of ZJUI ECE 445 Spring 2024
 * Project 19.
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

uint8_t esp_response[255] = {0};

// mqtt client info
#define MQTT_CLIENT_ID "WindPwr"
#define MQTT_KEEPALIVE 60

// mqtt QoS
#define MQTT_QOS0 0
#define MQTT_QOS1 1
#define MQTT_QOS2 2

// mqtt topic
#define MQTT_TOPIC_STATUS "system/pwr"
#define MQTT_TOPIC_WARN "system/warining"
#define MQTT_TOPIC_USR_CMD "usr/cmd"

extern uint8_t uart2_rx_data[255]; // UART2 DMA buffer
extern uint8_t uart2_rx_flag;      // Flag to indicate that UART2 DMA has received data
uint8_t mqtt_recv_topic[255];      // MQTT received topic
uint8_t mqtt_recv_msg[255];        // MQTT received message

extern uint32_t adc1_data[6]; // ADC data

// Message queues
extern osMessageQueueId_t esp_rx_queueHandle;  // ESP message queue
extern osMessageQueueId_t esp_tx_queueHandle;  // MQTT message queue
extern osMessageQueueId_t usr_cmd_queueHandle; // User command queue

// Tasks
extern osThreadId_t esp_msg_tskHandle; // ESP message task

// Events
extern osEventFlagsId_t sys_statHandle;

// mutexes
extern osMutexId_t adc_mutexHandle;
extern osMutexId_t lcd_mutexHandle;

/**
 * @brief ESP-12F initialization (in OS)
 *
 */
void esp_init_os(void)
{
    uint8_t cmd[255] = {0};
    uint8_t esp_response_buff[255] = {0}; // ESP response buffer (privately
                                          // used in this function)
    // Perform reset
    osDelay(1000);
    HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_RESET);
    osEventFlagsClear(sys_statHandle, WIFI_CONN_STAT | MQTT_CONN_STAT);

    // Set UI
    osMutexAcquire(lcd_mutexHandle, osWaitForever);
    LCD_SetAsciiFont(&ASCII_Font16);
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
            LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
            LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "Connecting...");
            osMutexRelease(lcd_mutexHandle);
        }
        else
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
            LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "CWMODE Error");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
        LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "CWMODE Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
    osDelay(1000);

    // Connect to WiFi
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", (uint8_t *)WIFI_SSID, (uint8_t *)WIFI_PWD);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 8000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "WIFI CONNECTED") != NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
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
        LCD_ClearRect(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, LCD_Width - LCD_WIFI_STAT_X, 16);
        LCD_DisplayString(LCD_WIFI_STAT_X, LCD_WIFI_STAT_Y, "Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
    osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, osWaitForever); // WIFI GOT IP \r\n OK
    osDelay(1000);

    // MQTT configuration
    sprintf(cmd, "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n", MQTT_CLIENT_ID, MQTT_USER, MQTT_PWD);
    HAL_UART_Transmit_DMA(&huart1, cmd, strlen(cmd));
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (osMessageQueueGet(esp_rx_queueHandle, esp_response_buff, NULL, 2000) == osOK)
    {
        if (strstr((char *)esp_response_buff, "OK") != NULL)
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "Config Success");
            osMutexRelease(lcd_mutexHandle);
        }
        else
        {
            osMutexAcquire(lcd_mutexHandle, osWaitForever);
            LCD_SetAsciiFont(&ASCII_Font16);
            LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
            LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "Config Error");
            osMutexRelease(lcd_mutexHandle);
            return;
        }
    }
    else
    {
        osMutexAcquire(lcd_mutexHandle, osWaitForever);
        LCD_SetAsciiFont(&ASCII_Font16);
        LCD_ClearRect(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, LCD_Width - LCD_MQTT_BRKR_X, 16);
        LCD_DisplayString(LCD_MQTT_BRKR_X, LCD_MQTT_BRKR_Y, "Config Timeout");
        osMutexRelease(lcd_mutexHandle);
        return;
    }
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
        uint8_t buff[255] = {0};
        uint8_t dummy; // dummy variable to store the message length
        // Send Message
        if (osMessageQueueGet(esp_tx_queueHandle, buff, NULL, 0) == osOK)
        {
            HAL_UART_Transmit_DMA(&huart2, buff, strlen(buff));
            osDelay(200);
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
    uint8_t buff[255] = {0};
    if (osMutexAcquire(adc_mutexHandle, 50) == osOK)
    {
        memcpy(adc_to_send, (uint32_t *)adc1_data, sizeof(adc_to_send));
        osMutexRelease(adc_mutexHandle);
    }
    else
    {
        return;
    }

    // get MQTT connection status; if not connected then return
    if (!(osEventFlagsGet(sys_statHandle) & MQTT_CONN_STAT))
    {
        return;
    }

    // Send data to ESP8266
    vmmc = voltage_current_format(adc_to_send[0] / ADC_COEFFICIENT);
    vbackup = voltage_current_format(adc_to_send[1] / ADC_COEFFICIENT);
    vout = voltage_current_format(adc_to_send[2] / ADC_COEFFICIENT);
    immc = voltage_current_format((2.5 - (adc_to_send[3] / ADC_COEFFICIENT)) / 0.1);
    ibackup = voltage_current_format((2.5 - (adc_to_send[4] / ADC_COEFFICIENT)) / 0.1);
    iout = voltage_current_format((2.5 - (adc_to_send[5] / ADC_COEFFICIENT)) / 0.1);
    pmmc = power_format(adc_to_send[0] / ADC_COEFFICIENT * (2.5 - (adc_to_send[3] / ADC_COEFFICIENT)) / 0.1);
    pbackup = power_format(adc_to_send[1] / ADC_COEFFICIENT * (2.5 - (adc_to_send[4] / ADC_COEFFICIENT)) / 0.1);
    pout = power_format(adc_to_send[2] / ADC_COEFFICIENT * (2.5 - (adc_to_send[5] / ADC_COEFFICIENT)) / 0.1);

    sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"0 %d %d %d %d %d %d %d %d %d\",2,0\r\n", (char *)MQTT_TOPIC_STATUS,
            vmmc, vbackup, vout, immc, ibackup, iout, pmmc, pbackup, pout);

    // TODO: Fix the power report message

    osMessageQueuePut(esp_tx_queueHandle, buff, 0, 0);
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