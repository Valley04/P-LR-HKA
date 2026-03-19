#include "printer_config.h"
#include "printer_mqtt.h"
#include "ota_task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include <string.h>

#define MAX_REINTENTOS_SPI    3

static const char* TAG_OTA = "OTA_MANAGER";
static const char* OTA_SPI = "SPI_OTA";
volatile bool ota_en_progreso = false;
extern esp_mqtt_client_handle_t mqtt_client;
extern SemaphoreHandle_t mqtt_data_mutex;
extern mqtt_data_t mqtt_data;

extern esp_err_t enviar_chunk_spi(uint8_t *data, int len);
extern uint8_t transaccionar_byte_spi(uint8_t data);

esp_err_t enviar_trama_spi_robusta(uint8_t *data, int len, uint32_t num_pack, uint32_t total_packs) {
    uint32_t checksum_paquete = 0;
    for (int j = 0; j < len; j++) checksum_paquete += data[j];

    uint16_t tam_total = 15 + len; 
    uint8_t trama_completa[1050]; 
    
    trama_completa[0] = SPI_CMD_PACKET;
    memcpy(&trama_completa[1], &num_pack, 4);
    memcpy(&trama_completa[5], &total_packs, 4);
    
    uint16_t tam_actual = (uint16_t)len;
    memcpy(&trama_completa[9], &tam_actual, 2);
    
    memcpy(&trama_completa[11], data, len); // El firmware real
    memcpy(&trama_completa[11 + len], &checksum_paquete, 4);

    int intentos = 0;
    while (intentos < MAX_REINTENTOS_SPI) {
        // Envio de la trama
        enviar_chunk_spi(trama_completa, tam_actual);

        vTaskDelay(pdMS_TO_TICKS(50));

        // Verificación
        if (transaccionar_byte_spi(0x00) == ACK_BYTE) {
            return ESP_OK; // Éxito total
        }

        intentos++;
        LOG_W(OTA_SPI, "⚠️ NAK en paquete %d (Intento %d/%d). Reintentando...", 
              num_pack, intentos, MAX_REINTENTOS_SPI);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }

    return ESP_FAIL; // Se agotaron los reintentos
}

void actualizar_impresora_por_spi(const char *url_descarga, const char* serial) {
    LOG_I(OTA_SPI, "Iniciando descarga para la impresora desde: %s", url_descarga);

    uint8_t *buffer = malloc(1024);
    if (!buffer) return;

    int total_bytes_recibidos = 0;
    int ultimo_porcentaje_notificado = -1;
    int content_length = 0;
    int fallos_red = 0;
    const int MAX_FALLOS_RED = 5;

    // Comando de iniciación
    if (transaccionar_byte_spi(SPI_CMD_START) != ACK_BYTE) {
        LOG_E(OTA_SPI, "Impresora no lista para OTA");
        free(buffer);
        return;
    }

    notificar_evento_ota(serial, "iniciando", "printer", 0);

    // Bucle de persistencia de red
    while (fallos_red < MAX_FALLOS_RED) {
        esp_http_client_config_t config = {
            .url = url_descarga,
            .timeout_ms = 5000,
        };

        esp_http_client_handle_t client = esp_http_client_init(&config);

        if (total_bytes_recibidos > 0) {
            char range_header[32];
            snprintf(range_header, sizeof(range_header), "bytes=%d-", total_bytes_recibidos);
            esp_http_client_set_header(client, "Range", range_header);
        }

        if (esp_http_client_open(client, 0) != ESP_OK) {
            LOG_E(OTA_SPI, "Error abriendo conexion HTTP con el servidor");
            esp_http_client_cleanup(client);
            fallos_red++;
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        if (content_length == 0) {
            content_length = esp_http_client_fetch_headers(client);
        }
        
        uint32_t total_paquetes = (content_length + 1023) / 1024;
        bool error_sesion = false;

        while (total_bytes_recibidos < content_length) {
            int leidos = esp_http_client_read(client, (char*)buffer, 1024);

            if (leidos < 0) {
                error_sesion = true;
                break;
            } else if (leidos == 0) break;

            uint32_t num_actual = (total_bytes_recibidos / 1024) + 1;

            // Envio de paquetes
            if (enviar_trama_spi_robusta(buffer, leidos, num_actual, total_paquetes) != ESP_OK) {
                LOG_E(OTA_SPI, "Fallo crítico de hardware SPI. Abortando...");
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                goto total_cleanup;
            }

            total_bytes_recibidos += leidos;
            fallos_red = 0;

            int porcentaje_actual = (total_bytes_recibidos * 100) / content_length;
            if (porcentaje_actual != ultimo_porcentaje_notificado) {
                notificar_evento_ota(serial, "actualizando", "printer", porcentaje_actual);
                ultimo_porcentaje_notificado = porcentaje_actual;
            }
        }
        
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        if (!error_sesion) break;
    }  
    
    if (total_bytes_recibidos >= content_length) {
        transaccionar_byte_spi(SPI_CMD_END);
        notificar_evento_ota(serial, "exito", "printer", 100);
    } else {
        notificar_evento_ota(serial, "error", "printer", ultimo_porcentaje_notificado);
    }

total_cleanup:
    free(buffer);
}

static void tarea_ota_esp32(void *pvParameters) {
    ota_config_t *ota_info = (ota_config_t*) pvParameters;

    ESP_LOGI(TAG_OTA, "Iniciando proceso OTA para %s con version: %s y url: %s", ota_info->objetivo, ota_info->version, ota_info->url);

    char serial_seguro[32] = "DESCONOCIDO";
    if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(serial_seguro, mqtt_data.register_number, sizeof(serial_seguro) - 1);
        serial_seguro[sizeof(serial_seguro) - 1] = '\0';
        xSemaphoreGive(mqtt_data_mutex);
    }
    
    if (strcmp(ota_info->objetivo, "ismart") == 0) {

        notificar_evento_ota(mqtt_data.register_number, "iniciando", "ismart", 0);

        esp_http_client_config_t http_config = {
            .url = ota_info->url,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .cert_pem = NULL,
            .skip_cert_common_name_check = true,
            .timeout_ms = 10000,
            .keep_alive_enable = true
        };

        esp_https_ota_config_t ota_config = {
            .http_config = &http_config,
        };

        ESP_LOGI(TAG_OTA, "Conectando al servidor y descargando firmware...");

        esp_err_t ret = esp_https_ota(&ota_config);

        if (ret == ESP_OK) {
            notificar_evento_ota(mqtt_data.register_number, "exito", "ismart", 100);
            ESP_LOGI(TAG_OTA, "OTA completada con éxito. Reiniciando...");
            vTaskDelay(pdMS_TO_TICKS(1500));
            esp_wifi_stop();
            esp_restart();
        } else {
            notificar_evento_ota(mqtt_data.register_number, "error", "ismart", 0);
            ESP_LOGE(TAG_OTA, "Error en OTA: %s", esp_err_to_name(ret));
            ota_en_progreso = false;
        }
    }
    else if (strcmp(ota_info->objetivo, "printer") == 0) {

        actualizar_impresora_por_spi(ota_info->url, mqtt_data.register_number);

        ota_en_progreso = false;
        ESP_LOGI(TAG_OTA, "✅ OTA de impresora finalizado. Reanudando UART normal...");
    }
    else {
        ESP_LOGE(TAG_OTA, "Objetivo OTA desconocido: %s", ota_info->objetivo);
        ota_en_progreso = false; // Cancelamos el freno de mano
    }
    
    free(ota_info);
    vTaskDelete(NULL);
}

void procesar_comando_ota(const char* payload, int len) {
    ESP_LOGI(TAG_OTA, "Analizando comando OTA recibido...");

    cJSON *json = cJSON_ParseWithLength(payload, len);
    if (json == NULL) {
        ESP_LOGE(TAG_OTA, "Error al parsear JSON de OTA");
        return;
    }

    cJSON *comando = cJSON_GetObjectItem(json, "comando");
    cJSON *objetivo = cJSON_GetObjectItem(json, "objetivo");
    cJSON *url = cJSON_GetObjectItem(json, "url_descarga");
    cJSON *version = cJSON_GetObjectItem(json, "version");

    if (!cJSON_IsString(comando) || strcmp(comando->valuestring, "INICIAR_OTA") != 0) {
        ESP_LOGE(TAG_OTA, "Comando no reconocido o faltante en OTA");
        cJSON_Delete(json);
        return;
    }

    if (!cJSON_IsString(objetivo) || !cJSON_IsString(url) || !cJSON_IsString(version)) {
        ESP_LOGE(TAG_OTA, "Faltan campos obligatorios en el comando OTA");
        cJSON_Delete(json);
        return;
    }

    if ((strcmp(objetivo->valuestring, "ismart") != 0) && (strcmp(objetivo->valuestring, "printer") != 0)) {
        ESP_LOGE(TAG_OTA, "Objetivo no válido para OTA: %s", objetivo->valuestring);
        cJSON_Delete(json);
        return;
    }

    ota_en_progreso = true;
    LOG_W(TAG_OTA, "Freno de mano activado: Impresora en reposo.");

    ota_config_t *ota_info = malloc(sizeof(ota_config_t));

    if (ota_info != NULL) {
        strncpy(ota_info->url, url->valuestring, OTA_MAX_URL_LEN - 1);
        strncpy(ota_info->objetivo, objetivo->valuestring, OTA_MAX_TARGET_LEN - 1);
        strncpy(ota_info->version, version->valuestring, OTA_MAX_VERSION_LEN - 1);
        
        // Aseguramos terminadores nulos
        ota_info->url[OTA_MAX_URL_LEN - 1] = '\0';
        ota_info->objetivo[OTA_MAX_TARGET_LEN - 1] = '\0';
        ota_info->version[OTA_MAX_VERSION_LEN - 1] = '\0';
        
        // Lanzamos la tarea de descarga (8192 bytes de stack suelen ser suficientes para OTA)
        // Lanzamos la tarea y verificamos que nazca con éxito
        if (xTaskCreate(tarea_ota_esp32, "ota_esp32_task", 8192, ota_info, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG_OTA, "Error: No se pudo crear la tarea OTA en RTOS");
            free(ota_info);
            ota_en_progreso = false; // ✅ Soltamos el freno
        }
    } else {
        ESP_LOGE(TAG_OTA, "Error reservando memoria para ota_info");
        ota_en_progreso = false;
    }

    cJSON_Delete(json);
}