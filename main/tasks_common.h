
#ifndef TASKS_COMMON_H
#define TASKS_COMMON_H

/**
 * @brief 
 * Right motor connected to Driver's OUT2
 */

#define TRACTION_CONTROL_TASK_PRIORITY  2
#define TRACTION_CONTROL_CORE_ID        0

#define BDC_MCPWM_TIMER_RESOLUTION_HZ   10000000 // 10MHz, 1 tick = 0.1us
#define BDC_ENCODER_PCNT_HIGH_LIMIT     2000
#define BDC_ENCODER_PCNT_LOW_LIMIT      -2000
#define BDC_PID_LOOP_PERIOD_MS          10

#define TRACTION_MOTORS_PWM_FREQ        25000       // 25kHz PWM

#define TRACTION_MOTOR_LEFT_PWMA        14
#define TRACTION_MOTOR_LEFT_PWMB        27
#define TRACTION_MOTOR_LEFT_ENCODER_A   26
#define TRACTION_MOTOR_LEFT_ENCODER_B   25

#define TRACTION_MOTOR_RIGHT_PWMA        32
#define TRACTION_MOTOR_RIGHT_PWMB        33
#define TRACTION_MOTOR_RIGHT_ENCODER_A   34
#define TRACTION_MOTOR_RIGHT_ENCODER_B   35

#define MOTOR1_ENCODER_RES              908
#define MOTOR2_ENCODER_RES              908

#define MOTOR_LEFT_KP                   0.6
#define MOTOR_LEFT_KI                   0.4
#define MOTOR_LEFT_KD                   0.2

#define MOTOR_RIGHT_KP                  0.6
#define MOTOR_RIGHT_KI                  0.4
#define MOTOR_RIGHT_KD                  0.2

#define MOTOR_LEFT_DESIRED_SPEED        5
#define MOTOR_RIGHT_DESIRED_SPEED       10

#define SERIAL_DEBUG_ENABLE             true

#endif