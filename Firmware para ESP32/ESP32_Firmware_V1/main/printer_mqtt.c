// printer_mqtt.c
#include "printer_config.h"
#include "printer_mqtt.h"
#include "printer_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include <string.h>

// VARIABLES ESTÁTICAS
static const char* TAG_MQTT = "mqtt_events";
static TaskHandle_t mqtt_simulator_handle = NULL;

// FUNCIONES AUXILIARES

// Inicializar simulador MQTT
void mqtt_simulator_init(void) {
    // Inicializar estructura de datos
    memset(&mqtt_data, 0, sizeof(mqtt_data_t));
    mqtt_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    strcpy(mqtt_data.register_number, "DESCONOCIDO");
    LOG_I(TAG_MQTT, "Simulador MQTT inicializado");
}

// Preparar tópico MQTT
void prepare_mqtt_topic(char* topic, size_t size, const char* data_type) {
    snprintf(topic, size, "%s/%s/%s", 
             MQTT_TOPIC_BASE, 
             mqtt_data.register_number, 
             data_type);
}

// Formatear JSON para MQTT
void format_mqtt_json(char* buffer, size_t size, const mqtt_data_t* data) {
    snprintf(buffer, size,
        "{\"timestamp\":%lu,"
        "\"sts1\":\"0x%02X\","
        "\"sts2\":\"0x%02X\","
        "\"estado\":\"%s\","
        "\"error\":\"%s\","
        "\"cajero\":\"%s\","
        "\"ventas\":\"%s\","
        "\"ult_factura\":\"%s\","
        "\"fact_emitidas\":\"%s\","
        "\"ult_debito\":\"%s\","
        "\"cant_debito\":\"%s\","
        "\"ult_credito\":\"%s\","
        "\"cant_credito\":\"%s\","
        "\"ult_nofiscal\":\"%s\","
        "\"cant_nofiscal\":\"%s\","
        "\"cierres_z\":\"%s\","
        "\"reportes_fiscal\":\"%s\","
        "\"rif\":\"%s\","
        "\"registro\":\"%s\","
        "\"hora\":\"%s\","
        "\"fecha\":\"%s\"}",
        data->timestamp,
        data->sts1, data->sts2,
        data->estado_str ? data->estado_str : "",
        data->error_str ? data->error_str : "",
        data->atm_number,
        data->ventas,
        data->last_bill_number,
        data->bill_issue,
        data->number_last_debit,
        data->amount_debit,
        data->number_last_credit,
        data->amount_credit,
        data->number_last_notfiscal,
        data->amount_notfiscal,
        data->counter_daily_z,
        data->counter_report_fiscal,
        data->rif_cliente,
        data->register_number,
        data->hour_machine,
        data->date_machine);
}

// Formatear JSON de estado
void format_mqtt_status_json(char* buffer, size_t size) {
    format_mqtt_json(buffer, size, &mqtt_data);
}

// Actualizar serial de impresora
void update_printer_serial(const char* serial) {
    if (serial != NULL && strlen(serial) > 0) {
        strncpy(mqtt_data.register_number, serial, MAX_SERIAL_LENGTH);
        mqtt_data.register_number[MAX_SERIAL_LENGTH] = '\0';
        mqtt_data.serial_obtained = true;
        LOG_I(TAG_MQTT, "Serial actualizado: %s", mqtt_data.register_number);
    }
}

// Preparar payload MQTT
void prepare_mqtt_payload(uint8_t sts1, uint8_t sts2, 
                         const char* estado_str, const char* error_str) {
    mqtt_data.sts1 = sts1;
    mqtt_data.sts2 = sts2;
    mqtt_data.estado_str = estado_str;
    mqtt_data.error_str = error_str;
    mqtt_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    mqtt_data.needs_sync = true;
    
    LOG_I(TAG_MQTT, "=== DATOS PREPARADOS PARA MQTT ===");
    LOG_I(TAG_MQTT, "Serial: %s", mqtt_data.register_number);
    LOG_I(TAG_MQTT, "STS1: 0x%02X, STS2: 0x%02X", sts1, sts2);
    LOG_I(TAG_MQTT, "Estado: %s", estado_str);
    LOG_I(TAG_MQTT, "Error: %s", error_str);
    LOG_I(TAG_MQTT, "==================================");
}

// Simular publicación MQTT
void simulate_mqtt_publish(const char* topic, const char* payload) {
    LOG_I(TAG_MQTT, "[SIM] Publicar: %s -> %s", topic, payload);
}

// Loggear mensaje MQTT
void log_mqtt_message(const char* topic, const char* payload) {
    ESP_LOGI(TAG_MQTT, "TOPIC: %s", topic);
    ESP_LOGI(TAG_MQTT, "PAYLOAD: %s", payload);
}

// TAREA SIMULADORA MQTT

void mqtt_simulator_task(void* pvParameters) {
    LOG_I(TAG_MQTT, "Tarea simuladora MQTT iniciada");
    
    char last_serial[11] = "";
    uint32_t last_sync_time = 0;
    
    while (1) {
        if (mqtt_data.needs_sync) {
            // Detectar cambio de serial
            if (strcmp(last_serial, mqtt_data.register_number) != 0) {
                strcpy(last_serial, mqtt_data.register_number);
                LOG_I(TAG_MQTT, "¡Nuevo serial detectado!: %s", last_serial);
                
                // Tópico de descubrimiento
                char discovery_topic[128];
                snprintf(discovery_topic, sizeof(discovery_topic), 
                        "%s/%s/discovery", MQTT_TOPIC_BASE, mqtt_data.register_number);
                simulate_mqtt_publish(discovery_topic, "online");
            }
            
            // Publicar todos los campos individualmente
            char topic[128];
            char payload[64];
            
            // STS1
            prepare_mqtt_topic(topic, sizeof(topic), "sts1");
            snprintf(payload, sizeof(payload), "0x%02X", mqtt_data.sts1);
            simulate_mqtt_publish(topic, payload);
            
            // STS2
            prepare_mqtt_topic(topic, sizeof(topic), "sts2");
            snprintf(payload, sizeof(payload), "0x%02X", mqtt_data.sts2);
            simulate_mqtt_publish(topic, payload);
            
            // Estado
            if (mqtt_data.estado_str) {
                prepare_mqtt_topic(topic, sizeof(topic), "estado");
                simulate_mqtt_publish(topic, mqtt_data.estado_str);
            }
            
            // Error
            if (mqtt_data.error_str) {
                prepare_mqtt_topic(topic, sizeof(topic), "error");
                simulate_mqtt_publish(topic, mqtt_data.error_str);
            }
            
            // Cajero
            prepare_mqtt_topic(topic, sizeof(topic), "cajero");
            simulate_mqtt_publish(topic, mqtt_data.atm_number);
            
            // Ventas
            prepare_mqtt_topic(topic, sizeof(topic), "ventas");
            simulate_mqtt_publish(topic, mqtt_data.ventas);
            
            // Última factura
            prepare_mqtt_topic(topic, sizeof(topic), "ultima_factura");
            simulate_mqtt_publish(topic, mqtt_data.last_bill_number);
            
            // Facturas emitidas
            prepare_mqtt_topic(topic, sizeof(topic), "facturas_emitidas");
            simulate_mqtt_publish(topic, mqtt_data.bill_issue);
            
            // Última nota débito
            prepare_mqtt_topic(topic, sizeof(topic), "ultima_nota_debito");
            simulate_mqtt_publish(topic, mqtt_data.number_last_debit);
            
            // Cantidad notas débito
            prepare_mqtt_topic(topic, sizeof(topic), "cantidad_notas_debito");
            simulate_mqtt_publish(topic, mqtt_data.amount_debit);
            
            // Última nota crédito
            prepare_mqtt_topic(topic, sizeof(topic), "ultima_nota_credito");
            simulate_mqtt_publish(topic, mqtt_data.number_last_credit);
            
            // Cantidad notas crédito
            prepare_mqtt_topic(topic, sizeof(topic), "cantidad_notas_credito");
            simulate_mqtt_publish(topic, mqtt_data.amount_credit);
            
            // Último documento no fiscal
            prepare_mqtt_topic(topic, sizeof(topic), "ultimo_doc_nofiscal");
            simulate_mqtt_publish(topic, mqtt_data.number_last_notfiscal);
            
            // Cantidad documentos no fiscales
            prepare_mqtt_topic(topic, sizeof(topic), "cantidad_docs_nofiscal");
            simulate_mqtt_publish(topic, mqtt_data.amount_notfiscal);
            
            // Cierres Z
            prepare_mqtt_topic(topic, sizeof(topic), "cierres_z");
            simulate_mqtt_publish(topic, mqtt_data.counter_daily_z);
            
            // Reportes fiscal
            prepare_mqtt_topic(topic, sizeof(topic), "reportes_fiscal");
            simulate_mqtt_publish(topic, mqtt_data.counter_report_fiscal);
            
            // RIF
            prepare_mqtt_topic(topic, sizeof(topic), "rif");
            simulate_mqtt_publish(topic, mqtt_data.rif_cliente);
            
            // Hora
            prepare_mqtt_topic(topic, sizeof(topic), "hora");
            simulate_mqtt_publish(topic, mqtt_data.hour_machine);
            
            // Fecha
            prepare_mqtt_topic(topic, sizeof(topic), "fecha");
            simulate_mqtt_publish(topic, mqtt_data.date_machine);
            
            // Timestamp
            prepare_mqtt_topic(topic, sizeof(topic), "timestamp");
            snprintf(payload, sizeof(payload), "%lu", mqtt_data.timestamp);
            simulate_mqtt_publish(topic, payload);
            
            // Errores
            prepare_mqtt_topic(topic, sizeof(topic), "errores");
            snprintf(payload, sizeof(payload), "%d", mqtt_data.error_count);
            simulate_mqtt_publish(topic, payload);
            
            // Intentos reconexión
            prepare_mqtt_topic(topic, sizeof(topic), "reconexiones");
            snprintf(payload, sizeof(payload), "%d", mqtt_data.reconnect_attempts);
            simulate_mqtt_publish(topic, payload);
            
            // JSON completo
            char json_buffer[1024];
            format_mqtt_json(json_buffer, sizeof(json_buffer), &mqtt_data);
            prepare_mqtt_topic(topic, sizeof(topic), "json_completo");
            simulate_mqtt_publish(topic, json_buffer);
            
            // Estadísticas de intervalo
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (last_sync_time > 0) {
                uint32_t interval = current_time - last_sync_time;
                LOG_I(TAG_MQTT, "Intervalo desde última sincronización: %lu ms", interval);
            }
            last_sync_time = current_time;
            
            mqtt_data.needs_sync = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Verificar cada 5 segundos
    }
}

// Función para iniciar la tarea MQTT (para ser llamada desde app_main)
void start_mqtt_simulator(void) {
    if (mqtt_simulator_handle != NULL) {
        return; // Ya está corriendo
    }
    
    BaseType_t result = xTaskCreate(
        mqtt_simulator_task,
        MQTT_SIMULATOR_NAME,
        MQTT_SIMULATOR_STACK,
        NULL,
        MQTT_SIMULATOR_PRIORITY,
        &mqtt_simulator_handle
    );
    
    if (result == pdPASS) {
        LOG_I(TAG_MQTT, "Tarea simuladora MQTT creada exitosamente");
    } else {
        LOG_E(TAG_MQTT, "Error al crear tarea simuladora MQTT");
    }
}