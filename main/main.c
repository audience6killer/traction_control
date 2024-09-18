#include <stdio.h>

#include "tasks_common.h"
#include "traction_control.h"

void app_main(void)
{
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

    traction_task_start(traction_handle);

}