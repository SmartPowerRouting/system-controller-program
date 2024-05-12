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
#define WIFI_CONN_STAT 1
#define MQTT_CONN_STAT 2
#define MMC_EN 4
#define BKUP_EN 8
#define SYS_OVERLD 16
#define SYS_UNDRLD 32
#define REPORT_PWR 64

// state machine
#define STATE_MACHINE_IDLE 0               // Idle state, power output cut off
#define STATE_MACHINE_PWR_ROUTING 1        // Power routing algorithm
#define STATE_MACHINE_PWR_FOURCE_PRIMARY 2 // Force primary power source
#define STATE_MACHINE_PWR_FOURCE_BACKUP 3  // Force backup power source

#endif // __OS_EVENTS_H
