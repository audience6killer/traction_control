idf_component_register(SRCS "traction_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_timer driver bdc_motor pid_ctrl)

