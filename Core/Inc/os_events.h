/**
 * @file os_events.h
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-04-27
 *
 * @copyright Copyright (c) 2024 This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
 */

#ifndef __OS_EVENTS_H
#define __OS_EVENTS_H

// status of system components
#define EVENT_WIFI_CONN_STAT 0x01 << 0
#define EVENT_MQTT_CONN_STAT 0x01 << 1
#define EVENT_MMC_EN 0x01 << 2
#define EVENT_BKUP_EN 0x01 << 3
#define EVENT_PWR_OFF 0x01 << 4
#define EVENT_OVERLOAD 0x01 << 5
#define EVENT_EB 0x01 << 6

// state machine
#define STATE_MACHINE_IDLE 0x01 << 0   // Idle state
#define STATE_MACHINE_OVERLD 0x01 << 1 // Overload state
#define STATE_MACHINE_EB 0x01 << 2     // Emergency backup state
#define STATE_MACHINE_NORMAL 0x01 << 3 // Normal state

#define IDLE_STATE 0
#define PWR_ROUTING_STATE 1
#define PWR_FORCE_PRIMARY_STATE 2
#define PWR_FORCE_BACKUP_STATE 3
#define OVERLOAD_STATE 4
#define EB_STATE 5

#endif // __OS_EVENTS_H
