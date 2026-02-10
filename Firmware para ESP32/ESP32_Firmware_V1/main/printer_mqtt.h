// printer_mqtt.h
#ifndef PRINTER_MQTT_H
#define PRINTER_MQTT_H

#include "printer_config.h"

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// CONFIGURACIÓN MQTT

#define MQTT_STACK              5120
#define MQTT_PRIORITY           5
#define MQTT_NAME               "mqtt_task"

// VARIABLES GLOBALES
extern volatile bool wifi_connected;
extern volatile bool mqtt_connected;
extern esp_mqtt_client_handle_t mqtt_client;

// PROTOTIPOS PÚBLICOS

// Inicialización
void start_mqtt_system(void);
void wifi_init_sta(void);
void mqtt_app_start(void);
void mqtt_data_init(void);

// Tareas
void mqtt_task(void* pvParameters);

// Manejadores de eventos
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

// Formateo de datos
void prepare_mqtt_topic(char* topic, size_t size, const char* serial, const char* data_type);
void format_mqtt_json(char* buffer, size_t size, const mqtt_data_t* data);
bool mqtt_publish(const char* topic, const char* payload);
void update_printer_serial(const char* serial);

#endif // PRINTER_MQTT_H