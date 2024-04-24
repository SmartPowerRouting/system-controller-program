/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BKUP_STAT_Pin GPIO_PIN_0
#define BKUP_STAT_GPIO_Port GPIOF
#define MMC_STAT_Pin GPIO_PIN_1
#define MMC_STAT_GPIO_Port GPIOF
#define WIFI_STAT_LED_Pin GPIO_PIN_2
#define WIFI_STAT_LED_GPIO_Port GPIOF
#define FAULT_Pin GPIO_PIN_3
#define FAULT_GPIO_Port GPIOF
#define K_EB_Pin GPIO_PIN_4
#define K_EB_GPIO_Port GPIOF
#define MQTTSRV_STAT_Pin GPIO_PIN_5
#define MQTTSRV_STAT_GPIO_Port GPIOF
#define ADC1_VMMC_Pin GPIO_PIN_0
#define ADC1_VMMC_GPIO_Port GPIOA
#define ADC1_VBKUP_Pin GPIO_PIN_1
#define ADC1_VBKUP_GPIO_Port GPIOA
#define ADC1_VOUT_Pin GPIO_PIN_2
#define ADC1_VOUT_GPIO_Port GPIOA
#define ADC2_IMMC_Pin GPIO_PIN_3
#define ADC2_IMMC_GPIO_Port GPIOA
#define ADC2_IBKUP_Pin GPIO_PIN_4
#define ADC2_IBKUP_GPIO_Port GPIOA
#define ADC2_IOUT_Pin GPIO_PIN_5
#define ADC2_IOUT_GPIO_Port GPIOA
#define PWRSEL_Pin GPIO_PIN_7
#define PWRSEL_GPIO_Port GPIOE
#define PWROFF_Pin GPIO_PIN_8
#define PWROFF_GPIO_Port GPIOE
#define TB_BRAKE_Pin GPIO_PIN_9
#define TB_BRAKE_GPIO_Port GPIOE
#define TB_RELEASE_Pin GPIO_PIN_10
#define TB_RELEASE_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOD
#define LCD_DC_Pin GPIO_PIN_12
#define LCD_DC_GPIO_Port GPIOD
#define LCD_Backlight_Pin GPIO_PIN_13
#define LCD_Backlight_GPIO_Port GPIOD
#define OS_STAT_Pin GPIO_PIN_8
#define OS_STAT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
