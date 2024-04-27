/**
 * @file esp.h
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2024-04-27
 * 
 * @copyright Copyright (c) 2024 This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 * 
 */

#ifndef __ESP_H
#define __ESP_H

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "printf.h"
#include "string.h"

// status code of response
#define ESP_OK 0
#define ESP_ERR 1

// Function prototypes
void esp_init(void);
uint8_t esp_get_response();
void esp_mqtt_report_pwr(float mmc_voltage, float mmc_current, float mmc_power, float bkup_voltage, float bkup_current, float bkup_power, float out_voltage, float out_current, float out_power);

#endif // __ESP_H