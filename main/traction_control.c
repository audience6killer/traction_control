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

#include "traction_control.h"
#include "tasks_common.h"

static const char *TAG = "TRACTION_CTRL";


static void pid_loop_callback(void *args)
{
    static int motor_left_last_pulse_count = 0;
    static int motor_right_last_pulse_count = 0;

    traction_control_handle_t *traction_handle = (traction_control_handle_t *)args;

    motor_control_context_t motor_right_ctx = traction_handle->motor_right_ctx; 
    motor_control_context_t motor_left_ctx = traction_handle->motor_left_ctx;

    // Get result from rotary encoder
    int motor_left_cur_pulse_count = 0;
    int motor_right_cur_pulse_count = 0;
    pcnt_unit_get_count(motor_left_ctx.pcnt_encoder, &motor_left_cur_pulse_count);
    pcnt_unit_get_count(motor_right_ctx.pcnt_encoder, &motor_right_cur_pulse_count);

    int motor_left_real_pulses = motor_left_cur_pulse_count - motor_left_last_pulse_count;
    int motor_right_real_pulses = motor_right_cur_pulse_count - motor_right_last_pulse_count;

    motor_right_last_pulse_count = motor_right_cur_pulse_count;
    motor_left_last_pulse_count = motor_left_cur_pulse_count;

    traction_handle->motor_right_ctx.report_pulses = motor_right_real_pulses;
    traction_handle->motor_left_ctx.report_pulses = motor_left_real_pulses;

    // Calculate speed error
    float motor_left_error = motor_left_ctx.desired_speed - motor_left_real_pulses;
    float motor_right_error = motor_right_ctx.desired_speed - 
    motor_right_real_pulses;

    float motor_right_new_speed = 0;
    float motor_left_new_speed = 0;

    // Set the new speed
    pid_compute(motor_right_ctx.pid_ctrl, motor_right_error, &motor_right_new_speed);
    pid_compute(motor_left_ctx.pid_ctrl, motor_left_error, &motor_left_new_speed);

    bdc_motor_set_speed(motor_right_ctx.motor, (uint32_t)motor_right_new_speed);
    bdc_motor_set_speed(motor_left_ctx.motor, (uint32_t)motor_left_new_speed);

}

esp_err_t traction_set_motors_desired_speed(const int motor_left_speed, const int motor_right_speed, traction_control_handle_t *traction_handle)
{
    if(motor_left_speed != -1)
    {
        traction_handle->motor_left_ctx.desired_speed = motor_left_speed;
    }
    if(motor_right_speed != -1)
    {
        traction_handle->motor_right_ctx.desired_speed = motor_right_speed;
    }

    return ESP_OK;
}

esp_err_t traction_control_init(const traction_control_config_t *motor_config, const pid_config_t *motor_left_pid_gains, const pid_config_t *motor_right_pid_gains, traction_control_handle_t *traction_handle)
{

    traction_handle->motor_left_ctx.pcnt_encoder = NULL;
    traction_handle->motor_right_ctx.pcnt_encoder = NULL;

    ESP_LOGI(TAG, "Setting up motors");

    /*Set up BDC motors*/
    bdc_motor_config_t motor_left_config = {
        .pwm_freq_hz = motor_config->pwm_freq_hz,
        .pwma_gpio_num = motor_config->motor_left_pwma_gpio_num,
        .pwmb_gpio_num = motor_config->motor_left_pwmb_gpio_num,
    };
    bdc_motor_config_t motor_right_config = {
        .pwm_freq_hz = motor_config->pwm_freq_hz,
        .pwma_gpio_num = motor_config->motor_right_pwma_gpio_num,
        .pwmb_gpio_num = motor_config->motor_right_pwmb_gpio_num,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor_left = NULL;
    bdc_motor_handle_t motor_right = NULL;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_left_config, &mcpwm_config, &motor_left));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_right_config, &mcpwm_config, &motor_right));

    traction_handle->motor_left_ctx.motor = motor_left;
    traction_handle->motor_right_ctx.motor = motor_right;

    /*Setting up motor encoders*/
    ESP_LOGI(TAG, "Setting up motor encoders");
    pcnt_unit_config_t motor_left_pcnt_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_config_t motor_right_pcnt_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };

    pcnt_unit_handle_t motor_left_pcnt_unit = NULL;
    pcnt_unit_handle_t motor_right_pcnt_unit = NULL;

    ESP_ERROR_CHECK(pcnt_new_unit(&motor_left_pcnt_config, &motor_left_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_unit(&motor_right_pcnt_config, &motor_right_pcnt_unit));
    pcnt_glitch_filter_config_t pcnt_filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor_left_pcnt_unit, &pcnt_filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor_right_pcnt_unit, &pcnt_filter_config));

    /* Setting up encoder channels*/
    /* Motor left*/
    pcnt_chan_config_t motor_left_pcnt_chan_a_config = {
        .edge_gpio_num = motor_config->motor_left_encodera_gpio_num,
        .level_gpio_num = motor_config->motor_left_encoderb_gpio_num,
    };
    pcnt_chan_config_t motor_left_pcnt_chan_b_config = {
        .edge_gpio_num = motor_config->motor_left_encoderb_gpio_num,
        .level_gpio_num = motor_config->motor_left_encodera_gpio_num,
    };
    pcnt_channel_handle_t motor_left_pcnt_chan_a = NULL;
    pcnt_channel_handle_t motor_left_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(motor_left_pcnt_unit, &motor_left_pcnt_chan_a_config, &motor_left_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(motor_left_pcnt_unit, &motor_left_pcnt_chan_b_config, &motor_left_pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor_left_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor_left_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor_left_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor_left_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_left_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_left_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    traction_handle->motor_left_ctx.pcnt_encoder = motor_left_pcnt_unit;

    /*Motor right*/
    pcnt_chan_config_t motor_right_pcnt_chan_a_config = {
        .edge_gpio_num = motor_config->motor_right_encodera_gpio_num,
        .level_gpio_num = motor_config->motor_right_encoderb_gpio_num,
    };
    pcnt_chan_config_t motor_right_pcnt_chan_b_config = {
        .edge_gpio_num = motor_config->motor_right_encoderb_gpio_num,
        .level_gpio_num = motor_config->motor_right_encodera_gpio_num,
    };
    pcnt_channel_handle_t motor_right_pcnt_chan_a = NULL;
    pcnt_channel_handle_t motor_right_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(motor_right_pcnt_unit, &motor_right_pcnt_chan_a_config, &motor_right_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(motor_right_pcnt_unit, &motor_right_pcnt_chan_b_config, &motor_right_pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor_right_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor_right_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor_right_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor_right_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_right_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_right_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    traction_handle->motor_right_ctx.pcnt_encoder = motor_right_pcnt_unit;

    /* PID control init*/
    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t motor_left_pid_runtime_param = {
        .kp = motor_left_pid_gains->kp,
        .ki = motor_left_pid_gains->ki,
        .kd = motor_left_pid_gains->kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_TIMER_RESOLUTION_HZ / motor_config->pwm_freq_hz,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_parameter_t motor_right_pid_runtime_param = {
        .kp = motor_right_pid_gains->kp,
        .ki = motor_right_pid_gains->ki,
        .kd = motor_right_pid_gains->kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_TIMER_RESOLUTION_HZ / motor_config->pwm_freq_hz,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };

    pid_ctrl_block_handle_t motor_left_pid_ctrl = NULL;
    pid_ctrl_config_t motor_left_pid_config = {
        .init_param = motor_left_pid_runtime_param,
    };
    pid_ctrl_block_handle_t motor_right_pid_ctrl = NULL;
    pid_ctrl_config_t motor_right_pid_config = {
        .init_param = motor_right_pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&motor_left_pid_config, &motor_left_pid_ctrl));
    ESP_ERROR_CHECK(pid_new_control_block(&motor_right_pid_config, &motor_right_pid_ctrl));

    traction_handle->motor_right_ctx.pid_ctrl = motor_right_pid_ctrl;
    traction_handle->motor_left_ctx.pid_ctrl = motor_left_pid_ctrl;

    return ESP_OK;

}

static void traction_control_task(void *pvParameter)
{
    // traction_control_handle_t *traction_handle = (traction_control_handle_t *)pvParameter;

    traction_control_config_t traction_config = {
        .motor_left_pwma_gpio_num = TRACTION_MOTOR_LEFT_PWMA,
        .motor_left_pwmb_gpio_num = TRACTION_MOTOR_LEFT_PWMB,
        .motor_right_pwma_gpio_num = TRACTION_MOTOR_RIGHT_PWMA,
        .motor_right_pwmb_gpio_num = TRACTION_MOTOR_RIGHT_PWMB,
        .motor_left_encodera_gpio_num = TRACTION_MOTOR_LEFT_ENCODER_A,
        .motor_left_encoderb_gpio_num = TRACTION_MOTOR_LEFT_ENCODER_B,
        .motor_right_encodera_gpio_num = TRACTION_MOTOR_RIGHT_ENCODER_A,
        .motor_right_encoderb_gpio_num = TRACTION_MOTOR_RIGHT_ENCODER_B,
        .pwm_freq_hz = TRACTION_MOTORS_PWM_FREQ,
    };

    pid_config_t motor_left_pid_config = {
        .kp = MOTOR_LEFT_KP,
        .ki = MOTOR_LEFT_KI,
        .kd = MOTOR_LEFT_KD,
    };

    pid_config_t motor_right_pid_config = {
        .kp = MOTOR_RIGHT_KP,
        .ki = MOTOR_LEFT_KI,
        .kd = MOTOR_RIGHT_KD,
    };

    traction_control_handle_t *traction_handle = malloc(sizeof(traction_control_handle_t));
    
    traction_control_init(&traction_config, &motor_left_pid_config, &motor_right_pid_config, traction_handle);

    traction_set_motors_desired_speed((const int)MOTOR_LEFT_DESIRED_SPEED, (const int)MOTOR_RIGHT_DESIRED_SPEED, traction_handle);

    ESP_ERROR_CHECK(pcnt_unit_enable(traction_handle->motor_left_ctx.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_enable(traction_handle->motor_right_ctx.pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(traction_handle->motor_left_ctx.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(traction_handle->motor_right_ctx.pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_start(traction_handle->motor_left_ctx.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(traction_handle->motor_right_ctx.pcnt_encoder));

    ESP_LOGI(TAG, "Create PID timer");

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_callback,
        .arg = traction_handle,
        .name = "pid_loop",
    };

    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motors");
    ESP_ERROR_CHECK(bdc_motor_enable(traction_handle->motor_left_ctx.motor));
    ESP_ERROR_CHECK(bdc_motor_enable(traction_handle->motor_right_ctx.motor));

    /* TEST ONLY */
    ESP_LOGI(TAG, "FORWARD MOTORS");
    ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
    ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        #if SERIAL_DEBUG_ENABLE
        
        printf("/*speed_left,%d; speed_right, %d*/\r\n", traction_handle->motor_left_ctx.report_pulses, traction_handle->motor_right_ctx.report_pulses);
        
        #endif 
        
    }   
}

esp_err_t traction_task_start()
{
    ESP_LOGI(TAG, "traction_control: Creating task");
    xTaskCreatePinnedToCore(&traction_control_task, "traction_task", 4096, NULL, TRACTION_CONTROL_TASK_PRIORITY, NULL, TRACTION_CONTROL_CORE_ID);

    return ESP_OK;
}




