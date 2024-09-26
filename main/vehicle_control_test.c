
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "tasks_common.h"
#include "vehicle_control_test.h"
#include "traction_control.h"

const char *TAG = "VEHICLE_CONTROL_TEST";

static void vehicle_control_test_task(traction_control_handle_t *traction_handle)
{
    const int traction_speed = 10;
    traction_handle->traction_state = FORWARD;
    traction_set_desired_speed(traction_speed, traction_handle);
    for (;;)
    {
        ESP_LOGI(TAG, "FORWARD");
        ESP_ERROR_CHECK(traction_set_forward(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "BREAK");
        ESP_ERROR_CHECK(traction_set_break(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "REVERSE");
        ESP_ERROR_CHECK(traction_set_reverse(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "BREAK");
        ESP_ERROR_CHECK(traction_set_break(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "REVERSE");
        ESP_ERROR_CHECK(traction_set_forward(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN LEFT FORWARD");
        ESP_ERROR_CHECK(traction_set_turn_left_forward(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN RIGHT FORWARD");
        ESP_ERROR_CHECK(traction_set_turn_right_forward(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN LEFT REVERSE");
        ESP_ERROR_CHECK(traction_set_turn_left_reverse(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN RIGHT REVERSE");
        ESP_ERROR_CHECK(traction_set_turn_right_reverse(traction_handle));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t vehicle_control_test_task_start(traction_control_handle_t *traction_handle)
{
    ESP_LOGI(TAG, "Creating task");

    xTaskCreatePinnedToCore(&vehicle_control_test_task, "vehicle_test_task", 4096, traction_handle, VEHICLE_TEST_TASK_PRIORITY, NULL, TRACTION_CONTROL_CORE_ID);

    return ESP_OK;
}
