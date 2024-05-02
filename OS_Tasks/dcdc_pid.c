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

// mutexes
extern osMutexId_t adc_mutexHandle;

// adc data
extern uint32_t adc1_data[6];

// message queues
extern osMessageQueueId_t dcdc_param_queueHandle;
dcdcParams_t dcdc_params;

// PID parameters
float Kp = 0.25;
float Ki = 100.;
float Kd = 0.;

/**
 * @brief DC/DC converter control task which implements PID control and power-routing control.
 *
 * @param argument
 */
void dcdc_ctrl_tsk(void *argument)
{
    float last_duty_ratio = 0.;      // in percentage
    float duty_ratio = 0.;           // in percentage
    float duty_ratio_increment = 0.; // in percentage
    float last_error = 0.;
    float last_last_error = 0.;
    float error = 0.;
    float desired_voltage = 12.; // in V

    for (;;)
    {
        last_last_error = last_error;
        last_error = error;
        last_duty_ratio = duty_ratio;
        // Get parameters from the message queue
        osMutexAcquire(adc_mutexHandle, osWaitForever);
        error = desired_voltage - adc1_data[0] * 3.3 / 4096;
        osMutexRelease(adc_mutexHandle);

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
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000); // TODO: Check how to calculate the duty ratio

        osDelay(1);
    }
}
