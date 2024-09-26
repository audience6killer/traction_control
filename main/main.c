#include <stdio.h>

#include "tasks_common.h"
#include "traction_control.h"
#include "vehicle_control_test.h"

void app_main(void)
{
    traction_control_handle_t *traction_handle = malloc(sizeof(traction_control_handle_t));

    traction_task_start(traction_handle);
    // vehicle_control_test_task_start(&traction_handle);
}