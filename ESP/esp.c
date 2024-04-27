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

uint8_t esp_response[255] = "\0";

// AP info
const char* WIFI_SSID = "WindPwr_Demo";
const char* WIFI_PWD = "WindPwr666";

// user info
#define MQTT_USER "WindPwr"
#define MQTT_PWD  "WindPwr666"

// mqtt broker info
#define MQTT_BROKER "iot.zjui.top"
#define MQTT_PORT 1883

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

// Msg queues
extern osMessageQueueId_t esp_response_queueHandle;

/**
 * @brief Extract and analyze response from ESP-12F
 * 
 * @return uint8_t 
 */
uint8_t esp_get_response()
{
    // extract response from queue
    // osMessageQueueGet(&esp_response_queueHandle, esp_response, NULL, osWaitForever);
    return ESP_OK;
}
 // extract response from ESP8266 msg queue


/**
 * @brief ESP-12F initialization
 * 
 */
void esp_init(void) {
    uint8_t cmd[255] = {0};   // Command buffer

    // randomly send something to avoid the first response from ESP8266
    sprintf(cmd, "AT\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(10);

    // Perform reset
    HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_RESET);
    sprintf(cmd, "AT+RST\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (esp_get_response() == ESP_ERR) {
        printf("ESP8266 reset failed\r\n");
        return;
    }
    printf("ESP8266 reset successfully\r\n");

    // Set station mode
    sprintf(cmd, "AT+CWMODE=1\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (esp_get_response() == ESP_ERR) {
        printf("ESP8266 set station mode failed\r\n");
        return;
    }

    // Connect to WiFi
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", (uint8_t*) WIFI_SSID, (uint8_t*) WIFI_PWD);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (esp_get_response() == ESP_ERR) {
        printf("ESP8266 connect to WiFi failed\r\n");
        return;
    }

    // configure MQTT user info

}

