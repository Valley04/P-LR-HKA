// printer_mqtt.c
#include "printer_mqtt.h"
#include "printer_task.h"
#include <string.h>
#include <stdio.h>
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"

// VARIABLES ESTÁTICAS
static const char* TAG_MQTT = "mqtt_events";
static const char* TAG_WIFI = "wifi station";

TaskHandle_t mqtt_task_handle = NULL;
esp_mqtt_client_handle_t mqtt_client = NULL;

volatile bool wifi_connected = false;
volatile bool mqtt_connected = false;

// FUNCIONES PARA CONEXIÓN WIFI

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connected = false;
        LOG_W(TAG_MQTT, "WiFi perdido, reintentando conexión...");
        esp_wifi_connect();

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        LOG_I(TAG_WIFI, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
    }
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            LOG_I(TAG_MQTT, "MQTT conectado");

            if (mqtt_task_handle != NULL) {
                xTaskNotifyGive(mqtt_task_handle);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            LOG_I(TAG_MQTT, "MQTT desconectado");
            break;
            
        case MQTT_EVENT_DATA:
            // Para futuras actualizaciones (en el caso de si aplica)
            break;
            
        case MQTT_EVENT_ERROR:
            mqtt_connected = false;
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                LOG_E(TAG_MQTT, "TCP error=%d", 
                      event->error_handle->esp_transport_sock_errno);
            }
            break;
            
        default:
            break;
    }
}

void wifi_init_sta(void)
{
    LOG_I(TAG_WIFI, "Inicializando WiFi...");

    // Iniciar la pila TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());

    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    esp_netif_create_default_wifi_sta();
    
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
}

// FUNCIONES AUXILIARES

// Inicializar MQTT
void mqtt_app_start(void) {
    LOG_I(TAG_MQTT, "Iniciando cliente MQTT...");
    
    // CONFIGURACIÓN CORRECTA para ESP-IDF v5.5.2
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname = "832b8689599f4045be005c116bc416f0.s1.eu.hivemq.cloud",
        .broker.address.port = 8883,
        .broker.address.transport = MQTT_TRANSPORT_OVER_SSL,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = "esp32_001",
        .credentials.authentication.password = "Esp12345",
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .buffer.size = 2048,
    };
    
    // Crear cliente MQTT
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        LOG_E(TAG_MQTT, "Error al inicializar cliente MQTT");
        return;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        LOG_E(TAG_MQTT, "Error al iniciar: %s", esp_err_to_name(err));
    }
}

// Inicializar datos MQTT
void mqtt_data_init(void) {
    memset(&mqtt_data, 0, sizeof(mqtt_data_t));

    mqtt_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    mqtt_data.needs_sync = false;

    strncpy(mqtt_data.register_number, "DESCONOCIDO", MAX_SERIAL_LENGTH);
    mqtt_data.register_number[MAX_SERIAL_LENGTH] = '\0';

    LOG_I(TAG_MQTT, "Datos MQTT inicializados");
}

// publicación MQTT
bool mqtt_publish(const char* topic, const char* payload) {
    if (!mqtt_connected || mqtt_client == NULL) {
        LOG_W(TAG_MQTT, "Publiación cancelada: MQTT Offline");
        return false;
    }
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 
                                         0, MQTT_QOS_LEVEL, 0);
    
    if (msg_id < 0) {
        LOG_E(TAG_MQTT, "Error publicando en %s: %d", topic, msg_id);
        return false;
    }
    
    LOG_I(TAG_MQTT, "Mensaje enviado con éxito [ID: %d]", msg_id);
    return true;
}

// FUNCIONES AUXILIARES

// Preparar tópico MQTT
void prepare_mqtt_topic(char* buffer, size_t size, const char* serial, const char* data_type) {
    // Ya no usamos mqtt_data directamente aquí
    snprintf(buffer, size, "%s/%s/%s", 
             MQTT_TOPIC_BASE, 
             serial, 
             data_type);
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

void format_mqtt_json(char* buffer, size_t max_len, const mqtt_data_t* data) {
    // Usamos snprintf para proteger el stack y la memoria
    snprintf(buffer, max_len,
        "{"
        "\"sts1\":%u,"
        "\"sts2\":%u,"
        "\"err_cnt\":%u,"
        "\"rec_att\":%u,"
        "\"atm\":\"%s\","
        "\"vts\":%lu,"
        "\"last_b\":\"%s\","
        "\"b_iss\":\"%s\","
        "\"db_n\":\"%s\","
        "\"db_a\":%lu,"
        "\"cr_n\":\"%s\","
        "\"cr_a\":%lu,"
        "\"nf_n\":\"%s\","
        "\"nf_a\":%lu,"
        "\"z_cnt\":\"%s\","
        "\"f_cnt\":\"%s\","
        "\"rif\":\"%s\","
        "\"ser\":\"%s\","
        "\"time\":\"%s\","
        "\"date\":\"%s\","
        "\"ts\":%lu"
        "}",
        data->sts1, data->sts2, data->error_count, data->reconnect_attempts,
        data->atm_number, data->ventas, data->last_bill_number, data->bill_issue,
        data->number_last_debit, data->amount_debit, data->number_last_credit, data->amount_credit,
        data->number_last_notfiscal, data->amount_notfiscal, data->counter_daily_z, data->counter_report_fiscal,
        data->rif_cliente, data->register_number, data->hour_machine, data->date_machine,
        data->timestamp
    );
}

// TAREA MQTT
void mqtt_task(void* pvParameters) {
    LOG_I(TAG_MQTT, "Tarea MQTT iniciada");

    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(60000); // 60 segundos de "seguro"
    static char last_serial[MAX_SERIAL_LENGTH + 1] = "";
    static char json_buffer[768];
    uint32_t notificationValue;
    int sync_count = 0;
    
    while (1) {
        // ESPERA ACTIVA/PASIVA
        BaseType_t notified = xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, xMaxBlockTime);

        // VERIFICACIÓN DE CONEXIÓN
        if (!mqtt_connected) {
            LOG_W(TAG_MQTT, "MQTT desconectado, reintentando en breve...");
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS)); // Delay de cortesía para no saturar
            continue;
        }

        // PROCESAMIENTO (Si fue notificado o el flag está arriba)
        if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            
            if (mqtt_data.needs_sync) {
                sync_count++;
                LOG_I(TAG_MQTT, "Sincronizando datos (%d)...", sync_count);

                // Detectar cambio de serial de forma segura
                if (strcmp(last_serial, mqtt_data.register_number) != 0) {
                    strncpy(last_serial, mqtt_data.register_number, MAX_SERIAL_LENGTH);
                    last_serial[MAX_SERIAL_LENGTH] = '\0';
                    
                    char discovery_topic[128];
                    snprintf(discovery_topic, sizeof(discovery_topic), 
                            "%s/%s/discovery", MQTT_TOPIC_BASE, last_serial);
                    mqtt_publish(discovery_topic, "online");
                }

                // Formateamos el JSON mientras tenemos la llave
                format_mqtt_json(json_buffer, sizeof(json_buffer), &mqtt_data);

                // Bajamos el flag y SOLTAMOS LA LLAVE rápido
                mqtt_data.needs_sync = false;
                xSemaphoreGive(mqtt_data_mutex);
                
                // PUBLICACIÓN (Fuera del Mutex para no bloquear a la impresora)
                char topic[64];                
                prepare_mqtt_topic(topic, sizeof(topic), last_serial, "json_completo");
                mqtt_publish(topic, json_buffer);
                
                LOG_I(TAG_MQTT, "Sincronización completada (%d)", sync_count);
            } else {
                xSemaphoreGive(mqtt_data_mutex);
            }
        }
    }
}

// Función para iniciar la tarea MQTT (para ser llamada desde app_main)
void start_mqtt_system(void) {
    LOG_I(TAG_MQTT, "Iniciando sistema MQTT...");

    // Creamos el Mutex
    if (mqtt_data_mutex == NULL) {
        mqtt_data_mutex = xSemaphoreCreateMutex();
        if (mqtt_data_mutex == NULL) {
            LOG_E(TAG_MQTT, "Error, no se pudo crear el Mutex");
            return;
        }
    }

    mqtt_data_init();

    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar WiFi y MQTT
    wifi_init_sta();
    mqtt_app_start();

    // 6. Crear tarea MQTT
    BaseType_t result = xTaskCreate(
        mqtt_task,
        MQTT_NAME,
        MQTT_STACK,
        NULL,
        MQTT_PRIORITY,
        &mqtt_task_handle
    );

    if (result == pdPASS) {
        LOG_I(TAG_MQTT, "Tarea MQTT creada exitosamente");
    } else {
        LOG_E(TAG_MQTT, "Error al crear tarea MQTT");
    }
}