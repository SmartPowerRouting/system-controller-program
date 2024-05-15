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

// mqtt client info
#define MQTT_CLIENT_ID "WindPwr"
#define MQTT_KEEPALIVE 10

// mqtt topic
#define MQTT_TOPIC_STATUS "system/status"
#define MQTT_TOPIC_WARN "system/warning"
#define MQTT_TOPIC_RESPONSE "system/response"
#define MQTT_TOPIC_USR_CMD "usr/cmd"

// mqtt lwt msg
#define MQTT_LWT_MSG MQTT_WARN_OFFLINE

// type of user commands
#define CMD_PWR_OFF '0'           // Use MMC or backup
#define CMD_SMART_PWR_ROUTING '1' // Use power routing algorithm
#define CMD_PWR_FORCE_PRIMARY '2' // Force primary power source
#define CMD_PWR_FORCE_BACKUP '3'  // Force backup power source

// command response
#define RESP_CMD_OK 1
#define RESP_CMD_ERR 0

// mqtt QoS
#define MQTT_QOS0 0
#define MQTT_QOS1 1
#define MQTT_QOS2 2

// warning code
#define MQTT_WARN_OFFLINE -1
#define MQTT_WARN_NORMAL 0
#define MQTT_WARN_OVERLOAD 1
#define MQTT_WARN_EB_PRESSED 2

// User command structure (used for setting voltage and currents)
typedef struct
{
    uint32_t voltage_backup_cut_in;
    uint32_t voltage_backup_cut_out;
    uint32_t current;
} user_cmd_t;

// OS task function prototypes
void esp_msg_tsk(void *argument);
void tmr_report_pwr_clbk(void *argument);
void mqtt_msg_tsk(void *argument);

// Helper function prototypes
void esp_init_os(void);
uint16_t voltage_current_format(float f);
uint16_t power_format(float f);

// lcd helper functions
void lcd_show_idle();
void lcd_show_normal();
void lcd_show_overload();
void lcd_show_eb();
void lcd_show_backup();
void lcd_show_limits(uint8_t v_cutin, uint8_t v_cutout, uint8_t i_limit);
void lcd_show_states(uint8_t state);

#endif // __ESP_H
