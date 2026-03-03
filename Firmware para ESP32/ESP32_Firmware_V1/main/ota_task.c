#include "ota_task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include <string.h>

static const char* TAG_OTA = "OTA_MANAGER";
volatile bool ota_en_progreso = false;
extern esp_mqtt_client_handle_t mqtt_client;

extern esp_err_t enviar_chunk_spi(uint8_t *data, int len);

void actualizar_impresora_por_spi(const char *url_descarga) {
    LOG_I(TAG_SPI, "Iniciando descarga para la impresora desde: %s", url_descarga);

    esp_http_client_config_t config = {
        .url = url_descarga,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (esp_http_client_open(client, 0) != ESP_OK) {
        LOG_E(TAG_SPI, "Error abriendo conexion HTTP con el servidor");
        esp_http_client_cleanup(client);
        return;
    }

    int content_length = esp_http_client_fetch_headers(client);
    LOG_I(TAG_SPI, "Tamaño del firmware de la impresora: %d bytes", content_length);

    uint8_t *buffer = malloc(1024);
    if (buffer == NULL) {
        LOG_E(TAG_SPI, "No hay memoria RAM suficiente para el buffer");
        esp_http_client_cleanup(client);
        return;
    }

    int bytes_leidos;
    int total_leidos = 0;

    while (1) {
        bytes_leidos = esp_http_client_read(client, (char*)buffer, 1024);

        if (bytes_leidos < 0) {
            LOG_E(TAG_SPI, "Error de red durante la descarga");
            break;
        } else if (bytes_leidos == 0) {
            LOG_I(TAG_SPI, "Descarga y transmision completada al 100%%");
            break;
        }

        if (enviar_chunk_spi(buffer, bytes_leidos) != ESP_OK) {
            LOG_E(TAG_SPI, "Error trasmitido chunk por SPI");
            break;
        }

        total_leidos += bytes_leidos;
        LOG_I(TAG_SPI, "Progreso: %d / %d bytes enviados...", total_leidos, content_length);
    }

    free(buffer);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static void tarea_ota_esp32(void *pvParameters) {

    ota_config_t *ota_info = (ota_config_t*) pvParameters;

    ESP_LOGI(TAG_OTA, "Iniciando proceso OTA para %s con version: %s y url: %s", ota_info->objetivo, ota_info->version, ota_info->url);

    if (strcmp(ota_info->objetivo, "ismart") == 0) {
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

        if (mqtt_client != NULL) {
            esp_mqtt_client_stop(mqtt_client);
            LOG_W(TAG_OTA, "Servicio MQTT detenido. Todo el WiFi dedicado al OTA.");
        }

        esp_err_t ret = esp_https_ota(&ota_config);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG_OTA, "OTA completada con éxito. Reiniciando...");
            esp_wifi_stop();
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            ESP_LOGE(TAG_OTA, "Error en OTA: %s", esp_err_to_name(ret));
            ota_en_progreso = false;
        }
    }
    else if (strcmp(ota_info->objetivo, "printer") == 0) {

        actualizar_impresora_por_spi(ota_info->url);

        ota_en_progreso = false;
        ESP_LOGI("OTA_MANAGER", "✅ OTA de impresora finalizado. Reanudando UART normal...");
    }
    else {
        ESP_LOGE("OTA_MANAGER", "Objetivo OTA desconocido: %s", objetivo->valuestring);
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
        xTaskCreate(tarea_ota_esp32, "ota_esp32_task", 8192, ota_info, 5, NULL);
    } else {
        ESP_LOGE(TAG_OTA, "Error reservando memoria para ota_info");
    }

    cJSON_Delete(json);
}