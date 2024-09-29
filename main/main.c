#include <stdio.h>

#include "tasks_common.h"
#include "traction_control.h"
#include "vehicle_control_test.h"

void app_main(void)
{

    traction_task_start();
    vehicle_control_test_task_start();
}
