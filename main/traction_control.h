/**
 * @file traction_control.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MAIN_TRACTION_CONTROL_H
#define MAIN_TRACTION_CONTROL_H

#include <stdint.h>
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

typedef struct {
    uint32_t motor_left_pwma_gpio_num;      /*Motor 1 PWM A gpio number */
    uint32_t motor_left_pwmb_gpio_num;      /*Motor 1 PWM B gpio number */
    uint32_t motor_right_pwma_gpio_num;     /*Motor 2 PWM A gpio number */
    uint32_t motor_right_pwmb_gpio_num;     /*Motor 2 PWM B gpio number */
    uint32_t motor_left_encodera_gpio_num;  /*Motor 1 Encoder A gpio number*/
    uint32_t motor_left_encoderb_gpio_num;  /*Motor 1 Encoder B gpio number*/
    uint32_t motor_right_encodera_gpio_num; /*Motor 2 Encoder A gpio number*/
    uint32_t motor_right_encoderb_gpio_num; /*Motor 2 Encoder B gpio number*/
    uint32_t pwm_freq_hz;      /*PWM frequency for both motors */
} traction_control_config_t;

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_config_t;

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
    int desired_speed;
} motor_control_context_t;

typedef struct {
    motor_control_context_t motor_left_ctx;
    motor_control_context_t motor_right_ctx;
} traction_control_handle_t;

/**
 * @brief Initialize the traction control device
 * 
 * @param motor_config 
 * @param motor_left_pid_config 
 * @param motor_right_pid_config 
 * @param traction_handle 
 * @return esp_err_t 
 */
esp_err_t traction_control_init(const traction_control_config_t *motor_config, const pid_config_t *motor_left_pid_config, const pid_config_t *motor_right_pid_config, traction_control_handle_t *traction_handle);

/**
 * @brief Set the motor desired speed for both motors
 * 
 * @param motor_left_speed 
 * @param motor_right_speed 
 * @param traction_handle 
 * @return esp_err_t 
 */
esp_err_t traction_set_motors_desired_speed(const int motor_left_speed, const int motor_right_speed, traction_control_handle_t *traction_handle);

/**
 * @brief Once the task is started the traction motors will begin to move according to the speed and direciton indicated
 * 
 * @param traction_handle 
 * @return esp_err_t 
 */
esp_err_t traction_task_start(traction_control_handle_t *traction_handle);


#endif