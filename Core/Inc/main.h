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
#define K2_Pin GPIO_PIN_2
#define K2_GPIO_Port GPIOE
#define K1_Pin GPIO_PIN_3
#define K1_GPIO_Port GPIOE
#define K0_Pin GPIO_PIN_4
#define K0_GPIO_Port GPIOE
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
#define VCMU_V1P_Pin GPIO_PIN_0
#define VCMU_V1P_GPIO_Port GPIOC
#define VCMU_V1N_Pin GPIO_PIN_1
#define VCMU_V1N_GPIO_Port GPIOC
#define VCMU_V2P_Pin GPIO_PIN_2
#define VCMU_V2P_GPIO_Port GPIOC
#define VCMU_V2N_Pin GPIO_PIN_3
#define VCMU_V2N_GPIO_Port GPIOC
#define VCMU_V3P_Pin GPIO_PIN_0
#define VCMU_V3P_GPIO_Port GPIOA
#define VCMU_V3N_Pin GPIO_PIN_1
#define VCMU_V3N_GPIO_Port GPIOA
#define VCMU_C1N_Pin GPIO_PIN_2
#define VCMU_C1N_GPIO_Port GPIOA
#define VCMU_C1P_Pin GPIO_PIN_3
#define VCMU_C1P_GPIO_Port GPIOA
#define VCMU_C2P_Pin GPIO_PIN_4
#define VCMU_C2P_GPIO_Port GPIOA
#define VCMU_C2N_Pin GPIO_PIN_5
#define VCMU_C2N_GPIO_Port GPIOA
#define VCMU_C3P_Pin GPIO_PIN_6
#define VCMU_C3P_GPIO_Port GPIOA
#define VCMU_C3N_Pin GPIO_PIN_7
#define VCMU_C3N_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOD
#define LCD_DC_Pin GPIO_PIN_12
#define LCD_DC_GPIO_Port GPIOD
#define LCD_Backlight_Pin GPIO_PIN_13
#define LCD_Backlight_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOC
#define LCD_SCK_Pin GPIO_PIN_3
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_5
#define LCD_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
