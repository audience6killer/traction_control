
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "tasks_common.h"
#include "vehicle_control_test.h"
#include "traction_control.h"

const char *TAG = "VEHICLE_CONTROL_TEST";

static void vehicle_control_test_task(void *pvParameters)
{
    const float traction_speed = 1.688;
    traction_set_desired_speed(traction_speed);
    for (;;)
    {
        ESP_LOGI(TAG, "FORWARD");
        ESP_ERROR_CHECK(traction_set_direction(FORWARD));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "BREAK");
        ESP_ERROR_CHECK(traction_set_direction(BREAK));
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "REVERSE");
        ESP_ERROR_CHECK(traction_set_direction(REVERSE));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "BREAK");
        ESP_ERROR_CHECK(traction_set_direction(BREAK));
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "REVERSE");
        ESP_ERROR_CHECK(traction_set_direction(REVERSE));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN LEFT FORWARD");
        ESP_ERROR_CHECK(traction_set_direction(TURN_LEFT_FORWARD));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN RIGHT FORWARD");
        ESP_ERROR_CHECK(traction_set_direction(TURN_RIGHT_FORWARD));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN LEFT REVERSE");
        ESP_ERROR_CHECK(traction_set_direction(TURN_LEFT_REVERSE));
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "TURN RIGHT REVERSE");
        ESP_ERROR_CHECK(traction_set_direction(TURN_RIGHT_REVERSE));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t vehicle_control_test_task_start(void)
{
    ESP_LOGI(TAG, "Creating task");

    xTaskCreatePinnedToCore(&vehicle_control_test_task, "vehicle_test_task", 4096, NULL, VEHICLE_TEST_TASK_PRIORITY, NULL, TRACTION_CONTROL_CORE_ID);

    return ESP_OK;
}
