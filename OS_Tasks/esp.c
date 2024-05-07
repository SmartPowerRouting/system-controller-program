/**
 * @file esp.c
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-04-27
 *
 * @copyright Copyright (c) 2024 This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
 */

#include "esp.h"
#include "lcd.h"
#include "adc.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
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

// mutexes
extern osMutexId_t adc_mutexHandle;

// For adc data calculating
sysPwrData_t sys_pwr_report = {0};

// For user command
usrCmd_t usr_cmd = {0};

/**
 * @brief ESP-12F initialization
 *
 */
void esp_init(void)
{
    LCD_DisplayString(0, 20, "ESP8266 initializing...");
    uint8_t cmd[255] = {0};        // Command buffer
    memset(uart2_rx_data, 0, 255); // Clear UART2 buffer
    // randomly send something to avoid the first response from ESP8266
    sprintf(cmd, "AT\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));

    HAL_Delay(1000);

    // Perform reset
    HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_RESET);

    sprintf(cmd, "AT+RESTORE\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));

    HAL_Delay(2000);
    LCD_DisplayString(0, 40, "ESP8266 restore success");

    // turn off echo
    sprintf(cmd, "ATE0\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(1000);

    // Set station mode
    sprintf(cmd, "AT+CWMODE=1\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(1000);

    // Connect to WiFi
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", (uint8_t *)WIFI_SSID, (uint8_t *)WIFI_PWD);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(8000);

    sprintf(cmd, "AT+MQTTUSERCFG=0,1,\"2222\",\"2222\",\"2222\",0,0,\"\"\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(1000);

    sprintf(cmd, "AT+MQTTCONN=0,\"118.31.68.218\",1883,0\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);

    sprintf(cmd, "AT+MQTTSUB=0,\"%s\",2\r\n", (uint8_t *)MQTT_TOPIC_USR_CMD);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(1000);

    sprintf(cmd, "AT+MQTTPUB=0,\"%s\",\"hello world\",2,0\r\n", (uint8_t *)MQTT_TOPIC_STATUS);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(1000);
}

// FreeRTOS tasks
/**
 * @brief ESP response analysis.
 *
 * @param argument
 */
void esp_msg_tsk(void *argument)
{
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
 * @brief Real-time power transmission to ESP using a Timer, this is the callback fcn
 *
 * @param argument
 */
void tmr_report_pwr_clbk(void *argument)
{
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

    // Send data to ESP8266
    sys_pwr_report.mmc.voltage = adc_to_send[0] / ADC_COEFFICIENT;
    sys_pwr_report.mmc.current = (2.5 - (adc_to_send[1] / ADC_COEFFICIENT)) / 0.1;
    sys_pwr_report.mmc.power = sys_pwr_report.mmc.voltage * sys_pwr_report.mmc.current;
    sys_pwr_report.bkup.voltage = adc_to_send[2] / ADC_COEFFICIENT;
    sys_pwr_report.bkup.current = (2.5 - (adc_to_send[4] / ADC_COEFFICIENT)) / 0.1;
    sys_pwr_report.bkup.power = sys_pwr_report.bkup.voltage * sys_pwr_report.bkup.current;
    sys_pwr_report.out.voltage = adc_to_send[4] / ADC_COEFFICIENT;
    sys_pwr_report.out.current = (2.5 - (adc_to_send[5] / ADC_COEFFICIENT)) / 0.1;
    sys_pwr_report.out.power = sys_pwr_report.out.voltage * sys_pwr_report.out.current;

    sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"0%2.2f%2.2f%2.2f%2.2f%2.2f%2.2f%2.2f%2.2f%2.2f,2,0\r\n",
            (uint8_t *)MQTT_TOPIC_STATUS,
            sys_pwr_report.mmc.voltage,
            sys_pwr_report.mmc.current,
            sys_pwr_report.mmc.power,
            sys_pwr_report.bkup.voltage,
            sys_pwr_report.bkup.current,
            sys_pwr_report.bkup.power,
            sys_pwr_report.out.voltage,
            sys_pwr_report.out.current,
            sys_pwr_report.out.power);

    // TODO: Fix the power report message

    osMessageQueuePut(esp_tx_queueHandle, buff, 0, 0);
}