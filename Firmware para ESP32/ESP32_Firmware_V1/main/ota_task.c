#include "ota_task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include <string.h>

static const char* TAG_OTA = "OTA_MANAGER";

static void tarea_ota_esp32(void *pvParameters) {

    ota_config_t *ota_info = (ota_config_t*) pvParameters;

    ESP_LOGI(TAG_OTA, "Iniciando proceso OTA para iSmart con version: %s y url: %s", ota_info->version, ota_info->url);

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
        ESP_LOGI(TAG_OTA, "OTA completada con éxito. Reiniciando...");
        ESP_LOGI(TAG_OTA, "El equipo se reiniciará en 3 segundos para aplicar los cambios...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    } else {
        ESP_LOGE(TAG_OTA, "Error en OTA: %s", esp_err_to_name(ret));
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

    if (strcmp(objetivo->valuestring, "ismart") != 0) {
        ESP_LOGE(TAG_OTA, "Objetivo no válido para OTA: %s", objetivo->valuestring);
        cJSON_Delete(json);
        return;
    }

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