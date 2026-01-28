idf_component_register(SRCS "mqttv5.c" "station_example_main.c"
                    PRIV_REQUIRES esp_wifi nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_wifi esp_event)
