/**
 * @file dcdc_pid.h
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @version 0.1
 * @date 2024-04-28
 *
 * @copyright Copyright (c) 2024 This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
 */
#ifndef __DCDC_PID_H
#define __DCDC_PID_H

#include <stdint.h>

// Function prototypes
void dcdc_ctrl_tsk(void *argument);
uint16_t round_float(float f);

#endif // __DCDC_PID_H
