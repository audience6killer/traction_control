#ifndef SEED_DISPENSER_CONTROL_H
#define SEED_DISPENSER_CONTROL_H

#include "stdio.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

typedef struct
{
    uint32_t pwma_gpio_num;
    uint32_t pwmb_gpio_num;
    uint32_t encoder_a_gpio_num;
    uint32_t encoder_b_gpio_num;
    uint32_t pwm_freq_hz;
} seed_dispenser_motor_config_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
} seed_dispenser_pid_config_t;

typedef struct
{
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
    int desired_speed; // In pulses
} seed_dispenser_motor_control_context_t;

typedef struct
{
    seed_dispenser_motor_control_context_t seed_dispenser_motor;
    seed_dispenser_motor_control_context_t cutter_disc_motor;
} seed_dispenser_sys_handle_t;

/**
 * @brief Start seed dispenser task
 *
 */
void seed_dispenser_task_start(void);

#endif
