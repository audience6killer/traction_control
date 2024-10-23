#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bdc_motor.h"
#include "pid_ctrl.h"

#include "seed_dispenser_control.h"
#include "task_common.h"

static const char TAG[] = "Seed_dispenser";

static seed_dispenser_sys_handle_t *seed_dispenser_handle = NULL;

static void seed_dispenser_pid_loop_callback(void *args)
{
    static int seed_dispenser_last_pulse_count = 0;
    static int cutter_disc_last_pulse_count = 0;

    // traction_control_handle_t *traction_handle = (traction_control_handle_t *)args;

    seed_dispenser_motor_control_context_t *seed_dispenser_ctx = &seed_dispenser_handle->seed_dispenser_motor;
    seed_dispenser_motor_control_context_t *cutter_disc_ctx = &seed_dispenser_handle->cutter_disc_motor;

    /* Calculate current speed */
    int seed_dispenser_cur_pulse_count = 0;
    int cutter_disc_cur_pulse_count = 0;
    pcnt_unit_get_count(seed_dispenser_ctx->pcnt_encoder, &seed_dispenser_cur_pulse_count);
    pcnt_unit_get_count(cutter_disc_ctx->pcnt_encoder, &cutter_disc_cur_pulse_count);

    /*The sign of the speed doesn't matter, as the forward and reverse of the motor
    will control the direction of the spin
    */
    int seed_dispenser_real_pulses = abs(seed_dispenser_cur_pulse_count - seed_dispenser_last_pulse_count);
    int cutter_disc_real_pulses = abs(cutter_disc_cur_pulse_count - cutter_disc_last_pulse_count);

    seed_dispenser_last_pulse_count = seed_dispenser_cur_pulse_count;
    cutter_disc_last_pulse_count = cutter_disc_cur_pulse_count;

    seed_dispenser_handle->cutter_disc_motor.report_pulses = cutter_disc_real_pulses;
    seed_dispenser_handle->seed_dispenser_motor.report_pulses = seed_dispenser_real_pulses;

    // Calculate speed error
    float seed_dispenser_error = seed_dispenser_ctx->desired_speed - seed_dispenser_real_pulses;
    float cutter_disc_error = cutter_disc_ctx->desired_speed - cutter_disc_real_pulses;

    float seed_dispenser_new_speed = 0;
    float cutter_disc_new_speed = 0;

    // Set the new speed
    pid_compute(seed_dispenser_ctx->pid_ctrl, seed_dispenser_error, &seed_dispenser_new_speed);
    pid_compute(cutter_disc_ctx->pid_ctrl, cutter_disc_error, &cutter_disc_new_speed);

    bdc_motor_set_speed(seed_dispenser_ctx->motor, (uint32_t)seed_dispenser_new_speed);
    bdc_motor_set_speed(cutter_disc_ctx->motor, (uint32_t)cutter_disc_new_speed);
}

/**
 * @brief Initialize bdc motor and pid config for the task
 *
 * @param dispenser_conf
 * @param cutter_disk
 * @param dispenser_pid
 * @param cutter_pid
 * @return esp_err_t
 */
esp_err_t seed_dispenser_init(seed_dispenser_motor_config_t *seed_dispenser_conf, seed_dispenser_motor_config_t *cutter_disc_conf, seed_dispenser_pid_config_t *dispenser_pid, seed_dispenser_pid_config_t *cutter_pid)
{
    if (seed_dispenser_handle == NULL)
    {
        ESP_LOGE(TAG, "seed_dispenser_handle is NULL while init task");
        return ESP_FAIL;
    }

    seed_dispenser_handle->cutter_disc_motor.pcnt_encoder = NULL;
    seed_dispenser_handle->seed_dispenser_motor.pcnt_encoder = NULL;

    ESP_LOGI(TAG, "Setting up seed dispenser motor");

    // BDC motor configuration
    bdc_motor_config_t seed_dispenser_motor_config = {
        .pwm_freq_hz = SEED_DISPENSER_FREQ_HZ,
        .pwma_gpio_num = SEED_DISPENSER_PWM_A,
        .pwmb_gpio_num = SEED_DISPENSER_PWM_B,
    };
    bdc_motor_config_t cutter_disc_motor_config = {
        .pwm_freq_hz = CUTTER_DISC_FREQ_HZ,
        .pwma_gpio_num = CUTTER_DISC_PWM_A,
        .pwmb_gpio_num = CUTTER_DISC_PWM_B,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = SEED_DISPENSER_MCPWM_GROUP_ID,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t seed_dispenser_motor = NULL;
    bdc_motor_handle_t cutter_disc_motor = NULL;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&seed_dispenser_motor_config, &mcpwm_config, &seed_dispenser_motor));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&cutter_disc_motor_config, &mcpwm_config, &cutter_disc_motor));

    seed_dispenser_handle->cutter_disc_motor.motor = cutter_disc_motor;
    seed_dispenser_handle->seed_dispenser_motor.motor = seed_dispenser_motor;

    // Setting up encoder configuration
    ESP_LOGI(TAG, "Setting up encoders");

    pcnt_unit_config_t seed_dispenser_pcnt_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_config_t cutter_disc_pcnt_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };

    pcnt_unit_handle_t seed_dispenser_pcnt_unit = NULL;
    pcnt_unit_handle_t cutter_disc_pcnt_unit = NULL;

    ESP_ERROR_CHECK(pcnt_new_unit(&seed_dispenser_pcnt_config, &seed_dispenser_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_unit(&cutter_disc_pcnt_config, &cutter_disc_pcnt_unit));

    pcnt_glitch_filter_config_t pcnt_filter_glitch_config = {
        .max_glitch_ns = 1000,
    };

    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(seed_dispenser_pcnt_unit, &pcnt_filter_glitch_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(cutter_disc_pcnt_unit, &pcnt_filter_glitch_config));

    // Setting up encoder channels
    // Seed dispenser encoder
    pcnt_chan_config_t seed_dispenser_pcnt_chan_a_config = {
        .edge_gpio_num = seed_dispenser_conf->encoder_a_gpio_num,
        .level_gpio_num = seed_dispenser_conf->encoder_b_gpio_num,
    };
    pcnt_chan_config_t seed_dispenser_pcnt_chan_b_config = {
        .edge_gpio_num = seed_dispenser_conf->encoder_b_gpio_num,
        .level_gpio_num = seed_dispenser_conf->encoder_a_gpio_num,
    };
    pcnt_channel_handle_t seed_dispenser_pcnt_chan_a = NULL;
    pcnt_channel_handle_t seed_dispenser_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(seed_dispenser_pcnt_unit, &seed_dispenser_pcnt_chan_a_config, &seed_dispenser_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(seed_dispenser_pcnt_unit, &seed_dispenser_pcnt_chan_b_config, &seed_dispenser_pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(seed_dispenser_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(seed_dispenser_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(seed_dispenser_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(seed_dispenser_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(seed_dispenser_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(seed_dispenser_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    seed_dispenser_handle->seed_dispenser_motor.pcnt_encoder = seed_dispenser_pcnt_unit;

    // Cutter disc
    pcnt_chan_config_t cutter_disc_pcnt_chan_a_config = {
        .edge_gpio_num = cutter_disc_conf->encoder_a_gpio_num,
        .level_gpio_num = cutter_disc_conf->encoder_b_gpio_num,
    };
    pcnt_chan_config_t cutter_disc_pcnt_chan_b_config = {
        .edge_gpio_num = cutter_disc_conf->encoder_b_gpio_num,
        .level_gpio_num = cutter_disc_conf->encoder_a_gpio_num,
    };
    pcnt_channel_handle_t cutter_disc_pcnt_chan_a = NULL;
    pcnt_channel_handle_t cutter_disc_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(cutter_disc_pcnt_unit, &cutter_disc_pcnt_chan_a_config, &cutter_disc_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(cutter_disc_pcnt_unit, &cutter_disc_pcnt_chan_b_config, &cutter_disc_pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cutter_disc_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(cutter_disc_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cutter_disc_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(cutter_disc_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(cutter_disc_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(cutter_disc_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    seed_dispenser_handle->cutter_disc_motor.pcnt_encoder = cutter_disc_pcnt_unit;

    // PID control init
    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t seed_dispenser_pid_runtime_param = {
        .kp = dispenser_pid->kp,
        .ki = dispenser_pid->ki,
        .kd = dispenser_pid->kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_TIMER_RESOLUTION_HZ / seed_dispenser_conf->pwm_freq_hz,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_parameter_t cutter_disc_pid_runtime_param = {
        .kp = cutter_pid->kp,
        .ki = cutter_pid->ki,
        .kd = cutter_pid->kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_TIMER_RESOLUTION_HZ / cutter_disc_conf->pwm_freq_hz,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };

    pid_ctrl_block_handle_t seed_dispenser_pid_ctrl = NULL;
    pid_ctrl_config_t seed_dispenser_pid_config = {
        .init_param = seed_dispenser_pid_runtime_param,
    };
    pid_ctrl_block_handle_t cutter_disc_pid_ctrl = NULL;
    pid_ctrl_config_t cutter_disc_pid_config = {
        .init_param = cutter_disc_pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&seed_dispenser_pid_config, &seed_dispenser_pid_ctrl));
    ESP_ERROR_CHECK(pid_new_control_block(&cutter_disc_pid_config, &cutter_disc_pid_ctrl));

    seed_dispenser_handle->cutter_disc_motor.pid_ctrl = cutter_disc_pid_ctrl;
    seed_dispenser_handle->seed_dispenser_motor.pid_ctrl = seed_dispenser_pid_ctrl;

    return ESP_OK;
}

/**
 * @brief Seed dispenser task
 *
 * @param pvParameters
 */
void seed_dispenser_task(void *pvParameters)
{

    seed_dispenser_handle = (seed_dispenser_sys_handle_t *)malloc(sizeof(seed_dispenser_sys_handle_t));

    // Motors configuration
    seed_dispenser_motor_config_t seed_dispenser_config = {
        .encoder_a_gpio_num = SEED_DISPENSER_ENCODER_A,
        .encoder_b_gpio_num = SEED_DISPENSER_ENCODERA_B,
        .pwma_gpio_num = SEED_DISPENSER_PWM_A,
        .pwmb_gpio_num = SEED_DISPENSER_PWM_B,
        .pwm_freq_hz = SEED_DISPENSER_FREQ_HZ,
    };
    seed_dispenser_motor_config_t cutter_disk_config = {
        .encoder_a_gpio_num = CUTTER_DISC_ENCODER_A,
        .encoder_b_gpio_num = CUTTER_DISC_ENCODER_B,
        .pwma_gpio_num = CUTTER_DISC_PWM_A,
        .pwmb_gpio_num = CUTTER_DISC_PWM_B,
        .pwm_freq_hz = CUTTER_DISC_FREQ_HZ,
    };

    // PID configuration
    seed_dispenser_pid_config_t seed_dispenser_pid_config = {
        .kp = SEED_DISPENSER_KP,
        .ki = SEED_DISPENSER_KI,
        .kd = SEED_DISPENSER_KD,
    };
    seed_dispenser_pid_config_t cutter_disk_pid_config = {
        .kp = CUTTER_DISC_KP,
        .ki = CUTTER_DISC_KI,
        .kd = CUTTER_DISC_KD,
    };

    // Initialize the bdc motors
    seed_dispenser_init(&seed_dispenser_config, &cutter_disk_config, &seed_dispenser_pid_config, &cutter_disk_pid_config);

    ESP_ERROR_CHECK(pcnt_unit_enable(seed_dispenser_handle->seed_dispenser_motor.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_enable(seed_dispenser_handle->cutter_disc_motor.pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(seed_dispenser_handle->seed_dispenser_motor.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(seed_dispenser_handle->cutter_disc_motor.pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_start(seed_dispenser_handle->seed_dispenser_motor.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(seed_dispenser_handle->cutter_disc_motor.pcnt_encoder));

    ESP_LOGI(TAG, "Create PID timer");

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = seed_dispenser_pid_loop_callback,
        .arg = NULL,
        .name = "seed_dispenser_pid_loop",
    };

    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motors");
    ESP_ERROR_CHECK(bdc_motor_enable(seed_dispenser_handle->cutter_disc_motor.motor));
    ESP_ERROR_CHECK(bdc_motor_enable(seed_dispenser_handle->seed_dispenser_motor.motor));

    // Initial State
    ESP_ERROR_CHECK(bdc_motor_brake(seed_dispenser_handle->cutter_disc_motor.motor));
    ESP_ERROR_CHECK(bdc_motor_brake(seed_dispenser_handle->seed_dispenser_motor.motor));

    ESP_LOGI(TAG, "Start seed dispenser speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, SEED_DISPENSER_PID_LOOP_PERIOD_MS * 1000));

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

#if SERIAL_DEBUG_ENABLE

        printf("/*left_desired_speed:%d, speed_left,%d; right_des_speed: %d, speed_right, %d*/\r\n", traction_handle->motor_left_ctx.desired_speed, traction_handle->motor_left_ctx.report_pulses, traction_handle->motor_right_ctx.desired_speed, traction_handle->motor_right_ctx.report_pulses);

#endif
    }
}

void seed_dispenser_task_start(void)
{
    ESP_LOGI(TAG, "Starting seed dispenser task");
    xTaskCreatePinnedToCore(&seed_dispenser_task,
                            "seed_dispenser",
                            SEED_DISPENSER_STACK_SIZE,
                            NULL,
                            SEED_DISPENSER_TASK_PRIORITY,
                            NULL,
                            SEED_DISPENSER_CORE_ID);
}