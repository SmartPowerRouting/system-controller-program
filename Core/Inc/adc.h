/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
#define ADC_COEFFICIENT 1320.00
#define ADC_COEFFICIENT_VOLTAGE_BKUP 5.4
#define ADC_COEFFICIENT_VOLTAGE_MMC 11.
#define ADC_COEFFICIENT_VOLTAGE 16.
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
/*
 * The data structure to store power data for a single source.
 */
typedef struct
{
  float voltage; // in V
  float current; // in A
  float power;   // in W
} pwrDataSingleSrc_t;

/*
 * The data structure to store power data for all channels, which is to be used in the queue.
 */
typedef struct
{
  pwrDataSingleSrc_t mmc;
  pwrDataSingleSrc_t bkup;
  uint8_t pwr_src; // 0: off, 1: MMC, 2: backup
} sysPwrData_t;

#define PWR_SRC_OFF 0
#define PWR_SRC_MMC 1
#define PWR_SRC_BKUP 2

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

