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

#include <stdint.h>

// status code of response
#define ESP_OK 0
#define ESP_ERR 1

// type of user commands
#define CMD_SET_VOLTAGE 0
#define CMD_SET_PWR_SRC 1   // Use MMC or backup
#define CMD_SET_PWR_STAT 2  // Output ON/OFF

// For CMD_SET_PWR_SRC command:
#define CMD_USE_MMC 0
#define CMD_USE_BKUP 1

// For setting power status:
#define CMD_PWR_OFF 0
#define CMD_PWR_ON 1

// User command structure
typedef struct
{
    uint8_t cmdType;
    uint8_t cmdValue;  // CMD_SET_VOLTAGE: 10-30 (unit: V)
                    // CMD_SET_PWR_SRC: CMD_USE_MMC/CMD_USE_BKUP
                    // CMD_SET_PWR_STAT: CMD_PWR_ON/CMD_PWR_OFF
} usrCmd_t;

// Function prototypes
void esp_init(void);
uint8_t esp_get_response();

#endif // __ESP_H
