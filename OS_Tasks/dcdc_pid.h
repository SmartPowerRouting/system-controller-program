/**
 * @file dcdc_pid.h
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @version 0.1
 * @date 2024-04-28
 *
 * @copyright Copyright (c) 2024 This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
 */

#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"

// Macros for power sources
#define PWR_SRC_MMC 0
#define PWR_SRC_BKUP 1
#define PWR_SRC_OFF 2

// Struct for control parameters
typedef struct
{
    float mmc_voltage;
    float bkup_voltage;
    float out_voltage;
    uint8_t target_voltage; // desired output voltage: 10-30V
    uint8_t pwr_src;        // PWR_SRC_MMC, PWR_SRC_BKUP, PWR_SRC_OFF
} dcdcParams_t;

// Function prototypes
void dcdc_ctrl_tsk(void *argument);