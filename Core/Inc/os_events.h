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
#define EVENT_OVERLD 0x01 << 4
#define EVENT_UNDRLD 0x01 << 5
#define EVENT_EB_PRESSED 0x01 << 6

// state machine
#define STATE_MACHINE_IDLE 0x01 << 0              // Idle state, power output cut off
#define STATE_MACHINE_PWR_ROUTING 0x01 << 1       // Power routing algorithm
#define STATE_MACHINE_PWR_FORCE_PRIMARY 0x01 << 2 // Force primary power source
#define STATE_MACHINE_PWR_FORCE_BACKUP 0x01 << 3  // Force backup power source
#define STATE_MACHINE_OVERLD 0x01 << 4            // Overload state
#define STATE_MACHINE_EB 0x01 << 5                // Emergency backup state

#endif // __OS_EVENTS_H
