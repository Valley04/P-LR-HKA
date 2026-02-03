// printer_mqtt.h
#ifndef PRINTER_MQTT_H
#define PRINTER_MQTT_H

#include "printer_config.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_crt_bundle.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#include "mqtt_client.h"
#include <mqtt5_client.h>

// CONFIGURACIÓN WIFI

#define ESP_WIFI_SSID      "LorenaWiFi"
#define ESP_WIFI_PASS      "041295wifi"
#define ESP_MAXIMUM_RETRY  5

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#define ESP_WIFI_SAE_MODE 0
#define EXAMPLE_H2E_IDENTIFIER NULL

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

extern EventGroupHandle_t wifi_event_group;
extern esp_mqtt_client_handle_t mqtt_client;

// CONFIGURACIÓN MQTT

#define MQTT_SIMULATOR_STACK    3072
#define MQTT_SIMULATOR_PRIORITY 4
#define MQTT_SIMULATOR_NAME     "mqtt_sim"
#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)
#define MQTT_QOS_LEVEL         1

extern esp_mqtt_client_handle_t mqttClient;

// ESTRUCTURAS MQTT

typedef struct {
    char topic[128];
    char payload[512];
    uint8_t qos;
    bool retain;
} mqtt_message_t;

// PROTOTIPOS PÚBLICOS

// Envio de datos MQTT
static void wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void wifi_init_sta(void);

// Inicialización
void mqtt_simulator_init(void);

// Tareas
void mqtt_task(void* pvParameters);

// Formateo de datos
void prepare_mqtt_topic(char* topic, size_t size, const char* data_type);
void format_mqtt_json(char* buffer, size_t size, const mqtt_data_t* data);
void format_mqtt_status_json(char* buffer, size_t size);

// Gestión de datos
void update_printer_serial(const char* serial);
void prepare_mqtt_payload(uint8_t sts1, uint8_t sts2, 
                         const char* estado_str, const char* error_str);

// Simulación
void mqtt_publish(const char* topic, const char* payload);
void log_mqtt_message(const char* topic, const char* payload);

#endif // PRINTER_MQTT_H