idf_component_register(SRCS "ismartv1.c" "printer_task.c" "printer_mqtt.c"
                    PRIV_REQUIRES esp_wifi nvs_flash
                    REQUIRES driver esp_driver_uart mqtt esp_wifi esp_event
                    INCLUDE_DIRS ".")
