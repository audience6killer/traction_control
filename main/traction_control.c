/**
 * @file traction_control.c
 * @author Adrian Pulido
 * @brief 
 * @version 0.1
 * @date 2024-09-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

#include "traction_control.h"

static const char *TAG = "TRACTION_CTRL";

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

static void pid_loop_callback(void *args)
{

}

esp_err_t traction_control_init(const traction_control_config_t *motor_config, const pid_config_t *motor_left, const pid_config_t *motor_right)
{
    static motor_control_context_t motor_left_ctx = {
        .pcnt_encoder = NULL,
    };
    static motor_control_context_t motor_right_ctx = {
        .pcnt_encoder = NULL,
    };

}

esp_err_t traction_task_start(void)
{

}






