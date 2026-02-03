// ismartv1.c (versión con depuración)
#include "printer_config.h"
#include "printer_task.h"
#include "printer_mqtt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Tag para logging
static const char* TAG = "MAIN";

void app_main(void) {
    // Configurar logging completo para depuración
    esp_log_level_set("*", ESP_LOG_INFO);  // Cambiado de ERROR a INFO
    esp_log_level_set("MAIN", ESP_LOG_INFO);
    esp_log_level_set("uart_events", ESP_LOG_INFO);
    esp_log_level_set("mqtt_sim", ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Sistema Fiscal v1.0 - Iniciando...");
    ESP_LOGI(TAG, "Compilado: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "Free heap inicial: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");
    
    // 1. Inicializar comunicación UART
    ESP_LOGI(TAG, "Inicializando comunicación UART...");
    if (!printer_uart_init()) {
        ESP_LOGE(TAG, "Error crítico: No se pudo inicializar UART");
        
        // Esperar antes de reintentar
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
        
        // Reiniciar ESP
        esp_restart();
        return;
    }
    ESP_LOGI(TAG, "UART inicializado exitosamente");
    
    // 2. Iniciar tarea de impresora
    ESP_LOGI(TAG, "Iniciando tarea de impresora...");
    if (!printer_task_start()) {
        ESP_LOGE(TAG, "Error crítico: No se pudo iniciar tarea de impresora");
        
        // Intentar desinicializar UART
        printer_uart_deinit();
        
        // Esperar y reintentar
        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));
        esp_restart();
        return;
    }
    ESP_LOGI(TAG, "Tarea de impresora iniciada");
    
    start_mqtt_system();
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Sistema iniciado exitosamente");
    ESP_LOGI(TAG, "Free heap después de inicio: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Tópico base MQTT: %s/{serial}", MQTT_TOPIC_BASE);
    ESP_LOGI(TAG, "========================================");
    
    // Tarea principal (monitoreo mejorado)
    int loop_count = 0;
    while (1) {
        loop_count++;
        
        // Mostrar estado cada 10 ciclos (30 segundos)
        if (loop_count % 10 == 0) {
            printer_state_t state = printer_get_state();
            ESP_LOGI(TAG, "Ciclo %d - Estado impresora: %s", 
                    loop_count, printer_state_to_string(state));
            
            // Verificar si necesita reconexión
            if (state == PRINTER_STATE_ERROR) {
                ESP_LOGW(TAG, "Impresora en estado ERROR");
                
                if (printer_needs_reconnection()) {
                    ESP_LOGI(TAG, "Intentando reconexión automática...");
                    bool reconnect_result = printer_reconnect();
                    ESP_LOGI(TAG, "Resultado reconexión: %s", 
                            reconnect_result ? "Éxito" : "Fallo");
                }
            }
            
            // Imprimir estadísticas de memoria
            ESP_LOGI(TAG, "Memoria libre: %d bytes", esp_get_free_heap_size());
            
            // Forzar primer ciclo de comunicación
            if (loop_count == 10) {
                ESP_LOGI(TAG, "Forzando primer ciclo de comunicación...");
                // Aquí podríamos enviar una señal a la tarea
            }
        }
        
        // Esperar antes de siguiente ciclo
        vTaskDelay(pdMS_TO_TICKS(3000)); // 3 segundos
    }
}