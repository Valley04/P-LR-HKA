// printer_mqtt.c
#include "printer_config.h"
#include "printer_mqtt.h"
#include "printer_task.h"
#include "freertos/portmacro.h"
#include <string.h>
#include <stdio.h>

// VARIABLES ESTÁTICAS
static const char* TAG_MQTT = "mqtt_events";
static const char *TAG_WIFI = "wifi station";
static TaskHandle_t mqtt_simulator_handle = NULL;
static bool wifi_connected = false;
static bool mqtt_connected = false;

// VARIABLES GLOBALES (declaradas en printer_config.h como extern)
EventGroupHandle_t wifi_event_group;
esp_mqtt_client_handle_t mqttClient;

// Cliente MQTT
esp_mqtt_client_handle_t mqtt_client = NULL;

// FUNCIONES PARA CONEXIÓN WIFI

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        LOG_I(TAG_WIFI, "WiFi STA iniciado");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        LOG_I(TAG_WIFI, "WiFi desconectado");
        wifi_connected = false;
        mqtt_connected = false;
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        LOG_I(TAG_WIFI, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            LOG_I(TAG_MQTT, "MQTT conectado");
            mqtt_connected = true;
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            LOG_I(TAG_MQTT, "MQTT desconectado");
            mqtt_connected = false;
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            LOG_I(TAG_MQTT, "MQTT suscrito, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            LOG_I(TAG_MQTT, "MQTT desuscrito, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            LOG_I(TAG_MQTT, "MQTT publicado, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            LOG_I(TAG_MQTT, "MQTT datos recibidos: topic=%.*s, data=%.*s",
                  event->topic_len, event->topic,
                  event->data_len, event->data);
            break;
            
        case MQTT_EVENT_ERROR:
            LOG_E(TAG_MQTT, "Error MQTT");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                LOG_E(TAG_MQTT, "Error de transporte: errno=%d", 
                      event->error_handle->esp_transport_sock_errno);
            }
            mqtt_connected = false;
            break;
            
        default:
            LOG_D(TAG_MQTT, "Evento MQTT: %ld", event_id);
            break;
    }
}

void wifi_init_sta(void)
{
    LOG_I(TAG_WIFI, "Inicializando WiFi...");
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    // Inicializar WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Registrar handlers de eventos
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    // Configurar WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    LOG_I(TAG_WIFI, "WiFi iniciado, SSID: %s", ESP_WIFI_SSID);
    
    // Esperar conexión
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(10000));
    
    if (bits & WIFI_CONNECTED_BIT) {
        LOG_I(TAG_WIFI, "Conectado a AP: %s", ESP_WIFI_SSID);
        wifi_connected = true;
    } else if (bits & WIFI_FAIL_BIT) {
        LOG_E(TAG_WIFI, "Falló conexión a: %s", ESP_WIFI_SSID);
        wifi_connected = false;
    } else {
        LOG_E(TAG_WIFI, "Timeout WiFi");
        wifi_connected = false;
    }
}

// FUNCIONES AUXILIARES

// Inicializar MQTT
static void mqtt_app_start(void) {
    LOG_I(TAG_MQTT, "Iniciando cliente MQTT...");
    
    // Configuración MQTT v5 con TLS
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.hostname = "832b8689599f4045be005c116bc416f0.s1.eu.hivemq.cloud",
            .address.port = 8883,  // Puerto MQTTS
            .address.transport = MQTT_TRANSPORT_OVER_SSL,
            .verification.crt_bundle_attach = esp_crt_bundle_attach,
        },
        .credentials = {
            .username = "esp32_001",
            .authentication.password = "Esp12345",
        },
        .session = {
            .protocol_ver = MQTT_PROTOCOL_V_5,
        },
        .network = {
            .reconnect_timeout_ms = 5000,
            .disable_auto_reconnect = false,
        },
        .task = {
            .stack_size = 6144,
            .priority = 5,
        },
        .buffer = {
            .size = 2048,
            .out_size = 512,
        },
    };
    
    // Crear cliente MQTT
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        LOG_E(TAG_MQTT, "Error al inicializar cliente MQTT");
        return;
    }
    
    // Registrar handler de eventos
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, 
                                                   ESP_EVENT_ANY_ID,
                                                   mqtt_event_handler, 
                                                   NULL));
    
    // Iniciar cliente MQTT
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        LOG_E(TAG_MQTT, "Error al iniciar cliente MQTT: 0x%x", err);
        return;
    }
    
    LOG_I(TAG_MQTT, "Cliente MQTT iniciado");
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
        data->estado_str,
        data->error_str,
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
    strncpy(mqtt_data.estado_str, estado_str ? estado_str : "", sizeof(mqtt_data.estado_str) - 1);
    strncpy(mqtt_data.error_str,  error_str  ? error_str  : "", sizeof(mqtt_data.error_str) - 1);
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
void mqtt_publish(const char* topic, const char* payload) {
    int msg_id = esp_mqtt_client_publish(mqttClient, topic, payload, 0, MQTT_QOS_LEVEL, 0);
    if (msg_id != 0) LOG_I(TAG_MQTT, "[SIM] Publicar: %s -> %s", topic, payload);
    else ESP_LOGI(TAG_MQTT, "Error en envio de datos, msg_id=%d", msg_id);
}

// Loggear mensaje MQTT
void log_mqtt_message(const char* topic, const char* payload) {
    ESP_LOGI(TAG_MQTT, "TOPIC: %s", topic);
    ESP_LOGI(TAG_MQTT, "PAYLOAD: %s", payload);
}

// TAREA MQTT

void mqtt_task(void* pvParameters) {
    LOG_I(TAG_MQTT, "Tarea MQTT iniciada");

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = "mqtts://832b8689599f4045be005c116bc416f0.s1.eu.hivemq.cloud",
        .credentials.username = "esp32_001",
        .credentials.authentication.password = "Esp12345",
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = true,
        .task.stack_size = 6144,
        .buffer.size = 2048

    };

    // Crear y configurar cliente MQTT
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt5_cfg);
    esp_mqtt_client_start(client);
    
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
                mqtt_publish(discovery_topic, "online");
            }
            
            // Publicar todos los campos individualmente
            char topic[128];
            char payload[64];
            
            // STS1
            prepare_mqtt_topic(topic, sizeof(topic), "sts1");
            snprintf(payload, sizeof(payload), "0x%02X", mqtt_data.sts1);
            mqtt_publish(topic, payload);
            
            // STS2
            prepare_mqtt_topic(topic, sizeof(topic), "sts2");
            snprintf(payload, sizeof(payload), "0x%02X", mqtt_data.sts2);
            mqtt_publish(topic, payload);
            
            // Estado
            if (mqtt_data.estado_str[0] != '\0') {
                prepare_mqtt_topic(topic, sizeof(topic), "estado");
                mqtt_publish(topic, mqtt_data.estado_str);
            }
            
            // Error
            if (mqtt_data.error_str[0] != '\0') {
                prepare_mqtt_topic(topic, sizeof(topic), "error");
                mqtt_publish(topic, mqtt_data.error_str);
            }
            
            // Cajero
            prepare_mqtt_topic(topic, sizeof(topic), "cajero");
            mqtt_publish(topic, mqtt_data.atm_number);
            
            // Ventas
            prepare_mqtt_topic(topic, sizeof(topic), "ventas");
            mqtt_publish(topic, mqtt_data.ventas);
            
            // Última factura
            prepare_mqtt_topic(topic, sizeof(topic), "ultima_factura");
            mqtt_publish(topic, mqtt_data.last_bill_number);
            
            // Facturas emitidas
            prepare_mqtt_topic(topic, sizeof(topic), "facturas_emitidas");
            mqtt_publish(topic, mqtt_data.bill_issue);
            
            // Última nota débito
            prepare_mqtt_topic(topic, sizeof(topic), "ultima_nota_debito");
            mqtt_publish(topic, mqtt_data.number_last_debit);
            
            // Cantidad notas débito
            prepare_mqtt_topic(topic, sizeof(topic), "cantidad_notas_debito");
            mqtt_publish(topic, mqtt_data.amount_debit);
            
            // Última nota crédito
            prepare_mqtt_topic(topic, sizeof(topic), "ultima_nota_credito");
            mqtt_publish(topic, mqtt_data.number_last_credit);
            
            // Cantidad notas crédito
            prepare_mqtt_topic(topic, sizeof(topic), "cantidad_notas_credito");
            mqtt_publish(topic, mqtt_data.amount_credit);
            
            // Último documento no fiscal
            prepare_mqtt_topic(topic, sizeof(topic), "ultimo_doc_nofiscal");
            mqtt_publish(topic, mqtt_data.number_last_notfiscal);
            
            // Cantidad documentos no fiscales
            prepare_mqtt_topic(topic, sizeof(topic), "cantidad_docs_nofiscal");
            mqtt_publish(topic, mqtt_data.amount_notfiscal);
            
            // Cierres Z
            prepare_mqtt_topic(topic, sizeof(topic), "cierres_z");
            mqtt_publish(topic, mqtt_data.counter_daily_z);
            
            // Reportes fiscal
            prepare_mqtt_topic(topic, sizeof(topic), "reportes_fiscal");
            mqtt_publish(topic, mqtt_data.counter_report_fiscal);
            
            // RIF
            prepare_mqtt_topic(topic, sizeof(topic), "rif");
            mqtt_publish(topic, mqtt_data.rif_cliente);
            
            // Hora
            prepare_mqtt_topic(topic, sizeof(topic), "hora");
            mqtt_publish(topic, mqtt_data.hour_machine);
            
            // Fecha
            prepare_mqtt_topic(topic, sizeof(topic), "fecha");
            mqtt_publish(topic, mqtt_data.date_machine);
            
            // Timestamp
            prepare_mqtt_topic(topic, sizeof(topic), "timestamp");
            snprintf(payload, sizeof(payload), "%lu", mqtt_data.timestamp);
            mqtt_publish(topic, payload);
            
            // Errores
            prepare_mqtt_topic(topic, sizeof(topic), "errores");
            snprintf(payload, sizeof(payload), "%d", mqtt_data.error_count);
            mqtt_publish(topic, payload);
            
            // Intentos reconexión
            prepare_mqtt_topic(topic, sizeof(topic), "reconexiones");
            snprintf(payload, sizeof(payload), "%d", mqtt_data.reconnect_attempts);
            mqtt_publish(topic, payload);
            
            // JSON completo
            char json_buffer[1024];
            format_mqtt_json(json_buffer, sizeof(json_buffer), &mqtt_data);
            prepare_mqtt_topic(topic, sizeof(topic), "json_completo");
            mqtt_publish(topic, json_buffer);
            
            // Estadísticas de intervalo
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (last_sync_time > 0) {
                uint32_t interval = current_time - last_sync_time;
                LOG_I(TAG_MQTT, "Intervalo desde última sincronización: %lu ms", interval);
            }
            last_sync_time = current_time;
            
            mqtt_data.needs_sync = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS)); // Verificar cada 5 segundos
    }
}

// Función para iniciar la tarea MQTT (para ser llamada desde app_main)
void start_mqtt_simulator(void) {
    if (mqtt_simulator_handle != NULL) {
        return; // Ya está corriendo
    }

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    while (1) 
    {
        if (isConnected) 
        {            
            BaseType_t result = xTaskCreate(
                mqtt_task,
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
            break;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS));
        }
    }
}