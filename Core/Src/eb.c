/**
 * @file eb.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-11
 * 
 * @copyright Copyright (c) 2024 Tiantian Zhong @ Zhejiang University
 *            This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
 */

#include "main.h"
#include "gpio.h"
#include "eb.h"

/**
 * @brief Scan if the emergency button is pressed.
 * 
 * @return 1 if the button is pressed, 0 otherwise.
 */
uint8_t eb_scan()
{
	return !HAL_GPIO_ReadPin(K_EB_GPIO_Port, K_EB_Pin);
}
