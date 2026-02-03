// printer_mqtt.h
#ifndef PRINTER_MQTT_H
#define PRINTER_MQTT_H

#include "printer_config.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "mqtt_client.h"
#include <mqtt5_client.h>

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#define ESP_WIFI_SAE_MODE 0
#define EXAMPLE_H2E_IDENTIFIER NULL

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// CONFIGURACIÓN MQTT

#define MQTT_SIMULATOR_STACK    8192
#define MQTT_SIMULATOR_PRIORITY 4
#define MQTT_SIMULATOR_NAME     "mqtt_task"
#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

// ESTRUCTURAS MQTT


// PROTOTIPOS PÚBLICOS

// Envio de datos MQTT
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void wifi_init_sta(void);

// Inicialización
void mqtt_app_start(void);
void mqtt_data_init(void);

// Tareas
void mqtt_task(void* pvParameters);
void start_mqtt_system(void);

// Formateo de datos
void prepare_mqtt_topic(char* topic, size_t size, const char* data_type);
void format_mqtt_json(char* buffer, size_t size, const mqtt_data_t* data);
void format_mqtt_status_json(char* buffer, size_t size);

// Gestión de datos
void update_printer_serial(const char* serial);
void prepare_mqtt_payload(uint8_t sts1, uint8_t sts2, 
                         const char* estado_str, const char* error_str);

// Publicación
bool mqtt_publish(const char* topic, const char* payload);
void log_mqtt_message(const char* topic, const char* payload);

#endif // PRINTER_MQTT_H