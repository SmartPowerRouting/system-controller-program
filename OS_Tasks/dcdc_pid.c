/**
 * @file dcdc_pid.c
 * @author Tiantian Zhong (giant@zju.edu.cn)
 * @brief PID control of DC/DC converter with power-routing control functionality.
 * @version 0.1
 * @date 2024-04-28
 *
 * @copyright Copyright (c) 2024 This file is part of ZJUI ECE 445 Spring 2024 Project 19.
 *
 */

#include "dcdc_pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "tim.h"

// message queues
extern osMessageQueueId_t dcdc_param_queueHandle;
dcdcParams_t dcdc_params;

// PID parameters
float Kp = 0.25;
float Ki = 100.;
float Kd = 0.;

// Function prototypes
void dcdc_ctrl_tsk(void *argument);

/**
 * @brief DC/DC converter control task which implements PID control and power-routing control.
 *
 * @param argument
 */
void dcdc_ctrl_tsk(void *argument)
{
    float last_duty_ratio = 0.; // in percentage
    float duty_ratio = 0.;      // in percentage
    float duty_ratio_increment = 0.; // in percentage
    float last_error = 0.;
    float last_last_error = 0.;
    float error = 0.;

    for (;;)
    {
        last_last_error = last_error;
        last_error = error;
        last_duty_ratio = duty_ratio;
        // Get parameters from the message queue
        osMessageQueueGet(dcdc_param_queueHandle, &dcdc_params, NULL, osWaitForever);
        switch (dcdc_params.pwr_src)
        {
        case PWR_SRC_MMC:
            // Set the output to MMC
            HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_RESET);
            osDelay(20);    // wait for the relay to switch
            HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_SET);
            break;
        case PWR_SRC_BKUP:
            // Set the output to backup
            HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_RESET);
            osDelay(20);    // wait for the relay to switch
            HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_SET);
            break;
        case PWR_SRC_OFF:
            HAL_GPIO_WritePin(MMC_EN_GPIO_Port, MMC_EN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BKUP_EN_GPIO_Port, BKUP_EN_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
        }

        // calculate error
        error = dcdc_params.target_voltage - dcdc_params.out_voltage;
        // Incremental PID
        duty_ratio_increment = Kp * (error - last_error) + Ki * error + Kd * (error - 2 * last_error + last_last_error);
        duty_ratio += duty_ratio_increment;

        // Limit the duty ratio
        if (duty_ratio > 100.)
        {
            duty_ratio = 100.;
        }
        else if (duty_ratio < 0.)
        {
            duty_ratio = 0.;
        }

        // calculate PWM duty ratio
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000); // TODO: Check how to calculate the duty ratio

        osDelay(1);
    }
}
