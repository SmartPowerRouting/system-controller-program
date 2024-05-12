/**
 * @file esp.h
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief
 * @date 2024-05-10
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
#define CMD_SET_VOLTAGE 1
#define CMD_SET_PWR_STAT 0 // Use MMC or backup

// For setting power status:
#define CMD_PWR_OFF 0
#define CMD_PWR_USE_MMC 1
#define CMD_PWR_USE_BKUP 2

// User command structure
typedef struct
{
    uint8_t cmdType;
    uint8_t cmdValue; // CMD_SET_VOLTAGE: 10-30 (unit: V)
                      // CMD_SET_PWR_SRC: CMD_USE_MMC/CMD_USE_BKUP
                      // CMD_SET_PWR_STAT: CMD_PWR_ON/CMD_PWR_OFF
} usrCmd_t;

// OS task function prototypes
void esp_msg_tsk(void *argument);
void tmr_report_pwr_clbk(void *argument);
void mqtt_msg_tsk(void *argument);

// Helper function prototypes
void esp_init_os(void);
uint16_t voltage_current_format(float f);
uint16_t power_format(float f);

#endif // __ESP_H
