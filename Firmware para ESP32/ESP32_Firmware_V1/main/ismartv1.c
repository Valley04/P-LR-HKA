// ismartv1.c
#include "printer_config.h"
#include "printer_task.h"
#include "printer_mqtt.h"

// Tag para logging
static const char* TAG = "MAIN";

void app_main(void) {
    
    ESP_LOGI(TAG, "Iniciando Sistema Fiscal...");

    uint32_t free_ram = esp_get_free_heap_size();
    ESP_LOGI(TAG, "RAM libre: %u bytes", free_ram);
    
    // Inicializar comunicación UART
    if (!printer_uart_init()) {
        ESP_LOGE(TAG, "Error crítico: No se pudo inicializar UART");
        
        return;
    }
    
    // Iniciar tarea de impresora
    ESP_LOGI(TAG, "Iniciando tarea de impresora...");
    if (!printer_task_start()) {
        ESP_LOGE(TAG, "Error crítico: No se pudo iniciar tarea de impresora");
        
        // Intentar desinicializar UART
        printer_uart_deinit();

        return;
    }
    
    // Lanzar tareas de forma independiente mejora el uso de IRAM y DRAM
    ESP_ERROR_CHECK(printer_task_start());
    start_mqtt_system();
    
    // Tarea principal (monitoreo ligero)
    while (1) {
        // Imprime el estado de la memoria (solo para fase inicial)
        ESP_LOGI(TAG, "Heap libre: %u bytes | IRAM libre: %u bytes", 
                 esp_get_free_heap_size(), 
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        
        vTaskDelay(pdMS_TO_TICKS(60000)); // Esperamos 1 minuto
    }
}