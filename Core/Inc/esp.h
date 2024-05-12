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
#define CMD_PWR_OFF 0            // Use MMC or backup
#define CMD_SMART_PWR_ROUTING 1  // Use power routing algorithm
#define CMD_PWR_FOURCE_PRIMARY 2 // Force primary power source
#define CMD_PWR_FOURCE_BACKUP 3  // Force backup power source

// command response
#define RESP_CMD_OK 1
#define RESP_CMD_ERR 0

// warning code
#define WARN_OFFLINE -1
#define WARN_NORMAL 0
#define WARN_OVERLOAD 1
#define WARN_EB_PRESSED 2


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
