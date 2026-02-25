#include "ota_task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "cJSON.h"
#include <string.h>

static const char* TAG_OTA = "OTA_MANAGER";

static void tarea_ota_esp32(void *pvParameters) {

    ota_config_t *ota_info = (ota_config_t*) pvParameters;

    ESP_LOGI(TAG_OTA, "Iniciando proceso OTA para objetivo: %s, versión: %s", ota_info->objetivo, ota_info->version);

    esp_http_client_config_t ota_client_config = {
        .url = config->url,
        .cert_pem = NULL, // Para pruebas, no verificamos el certificado
        .timeout_ms = 60000,
    };

    esp_err_t ret = esp_https_ota(&ota_client_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_OTA, "OTA completada con éxito. Reiniciando...");
        esp_restart();
    } else {
        ESP_LOGE(TAG_OTA, "Error en OTA: %s", esp_err_to_name(ret));
    }

    vTaskDelete(NULL);
}