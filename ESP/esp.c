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

#include "lcd.h"
#include "esp.h"

uint8_t esp_response[255] = "\0";

// AP info
const char *WIFI_SSID = "WindPwr_Demo";
const char *WIFI_PWD = "WindPwr666";

// user info
#define MQTT_USER "WindPwr"
#define MQTT_PWD "WindPwr666"

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

extern uint8_t uart2_rx_data[255];   // UART2 DMA buffer
extern uint8_t mqtt_recv_topic[255]; // MQTT received topic
extern uint8_t mqtt_recv_msg[255];   // MQTT received message

/**
 * @brief Extract and analyze response from ESP-12F
 *
 * @return uint8_t
 */
uint8_t esp_get_response()
{
    return 0;
}
// extract response from ESP8266 msg queue

/**
 * @brief ESP-12F initialization
 *
 */
void esp_init(void)
{
    uint8_t cmd[255] = {0}; // Command buffer
    // randomly send something to avoid the first response from ESP8266
    sprintf(cmd, "AT\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(1000);

    // Perform reset
    HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MQTTSRV_STAT_GPIO_Port, MQTTSRV_STAT_Pin, GPIO_PIN_RESET);
    sprintf(cmd, "AT+RESTORE\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    if (esp_get_response() == ESP_ERR)
    {
        LCD_DisplayString(100, 100, "ESP8266 reset failed");
        return;
    }
    LCD_DisplayString(100, 100, "ESP8266 reset success");
    HAL_Delay(5000);

    // Set station mode
    sprintf(cmd, "AT+CWMODE=1\r\n");
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_Delay(1000);

    // Connect to WiFi
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", (uint8_t *)WIFI_SSID, (uint8_t *)WIFI_PWD);
    HAL_UART_Transmit_DMA(&huart2, cmd, strlen(cmd));
    HAL_GPIO_WritePin(WIFI_STAT_LED_GPIO_Port, WIFI_STAT_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(20000);

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

void esp_mqtt_report_pwr(float mmc_voltage, float mmc_current, float mmc_power, float bkup_voltage, float bkup_current, float bkup_power, float out_voltage, float out_current, float out_power)
{
    uint8_t buff[255] = {0};
    buff[0] = 0; //set header
    uint16_t tmp_mmc_voltage, tmp_mmc_current, tmp_mmc_power, tmp_bkup_voltage, tmp_bkup_current, tmp_bkup_power, tmp_out_voltage, tmp_out_current, tmp_out_power;
    tmp_mmc_voltage = (uint16_t)(mmc_voltage * 100);
    tmp_mmc_current = (uint16_t)(mmc_current * 100);
    tmp_mmc_power = (uint16_t)(mmc_power * 100);
    tmp_bkup_voltage = (uint16_t)(bkup_voltage * 100);
    tmp_bkup_current = (uint16_t)(bkup_current * 100);
    tmp_bkup_power = (uint16_t)(bkup_power * 100);
    tmp_out_voltage = (uint16_t)(out_voltage * 100);
    tmp_out_current = (uint16_t)(out_current * 100);
    tmp_out_power = (uint16_t)(out_power * 100);
    sprintf(buff, "AT+MQTTPUB=0,\"%s\",\"%04x%04x%04x%04x%04x%04x%04x%04x%04x\",2,0\r\n", (uint8_t *)MQTT_TOPIC_STATUS, tmp_mmc_voltage, tmp_mmc_current, tmp_mmc_power, tmp_bkup_voltage, tmp_bkup_current, tmp_bkup_power, tmp_out_voltage, tmp_out_current, tmp_out_power);
    HAL_UART_Transmit_DMA(&huart2, buff, strlen(buff));
}