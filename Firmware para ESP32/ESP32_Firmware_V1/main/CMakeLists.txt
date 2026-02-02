idf_component_register(SRCS "ismartv1.c" "printer_task.c" "printer_mqtt.c"
                    REQUIRES esp_driver_uart
                    INCLUDE_DIRS ".")
