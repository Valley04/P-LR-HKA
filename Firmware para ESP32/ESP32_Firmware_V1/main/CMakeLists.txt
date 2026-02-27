idf_component_register(SRCS "ota_task.c" "ismartv1.c" "printer_task.c" "printer_mqtt.c" "ota_task.c"
                    PRIV_REQUIRES esp_wifi nvs_flash
                    REQUIRES driver esp_driver_uart mqtt esp_wifi esp_event esp_http_client esp_https_ota
                    INCLUDE_DIRS ".")
