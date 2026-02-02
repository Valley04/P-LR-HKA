// printer_mqtt.h
#ifndef PRINTER_MQTT_H
#define PRINTER_MQTT_H

#include "printer_config.h"

// CONFIGURACIÓN MQTT

#define MQTT_SIMULATOR_STACK    2048
#define MQTT_SIMULATOR_PRIORITY 4
#define MQTT_SIMULATOR_NAME     "mqtt_sim"

// ESTRUCTURAS MQTT

typedef struct {
    char topic[128];
    char payload[512];
    uint8_t qos;
    bool retain;
} mqtt_message_t;

// PROTOTIPOS PÚBLICOS

// Inicialización
void mqtt_simulator_init(void);

// Tareas
void mqtt_simulator_task(void* pvParameters);

// Formateo de datos
void prepare_mqtt_topic(char* topic, size_t size, const char* data_type);
void format_mqtt_json(char* buffer, size_t size, const mqtt_data_t* data);
void format_mqtt_status_json(char* buffer, size_t size);

// Gestión de datos
void update_printer_serial(const char* serial);
void prepare_mqtt_payload(uint8_t sts1, uint8_t sts2, 
                         const char* estado_str, const char* error_str);

// Simulación
void simulate_mqtt_publish(const char* topic, const char* payload);
void log_mqtt_message(const char* topic, const char* payload);

#endif // PRINTER_MQTT_H