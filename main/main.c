#include <stdio.h>

#include "tasks_common.h"
#include "traction_control.h"

void app_main(void)
{

    traction_control_handle_t *traction_handle = NULL;
    traction_task_start(traction_handle); 

}