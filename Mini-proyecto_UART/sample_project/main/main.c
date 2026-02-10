#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

// --- CONFIGURACIÓN ---
// Cambia esto si usas otros pines
#define TXD_PIN 4
#define RXD_PIN 5

// Usamos UART1 para hablar con el PC (UART0 es para el Monitor/Log)
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE 1024

static const char *TAG = "MINI_PROYECTO";

void app_main(void) {
    // 1. Configuración del Puerto
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE, // Sin paridad para empezar fácil
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 2. Instalar Driver
    ESP_LOGI(TAG, "Iniciando UART1 en Pines TX=%d, RX=%d", TXD_PIN, RXD_PIN);
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 3. Bucle Principal
    while (1) {
        // --- ENVIAR ---
        const char *msg = "PING";
        int bytes_sent = uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
        ESP_LOGI(TAG, "📤 Enviado: %s (%d bytes)", msg, bytes_sent);

        // --- RECIBIR ---
        // Esperamos hasta 1 segundo por respuesta
        uint8_t data[128];
        int length = 0;
        
        // Verificamos si hay algo en el buffer
        uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&length);
        
        if (length > 0) {
            int len = uart_read_bytes(UART_PORT_NUM, data, length, pdMS_TO_TICKS(100));
            data[len] = 0; // Null terminator para imprimir como texto
            
            // Imprimimos lo que llegó
            ESP_LOGW(TAG, "📥 RECIBIDO: %s", (char *)data);
        } else {
            ESP_LOGE(TAG, "❌ Nadie responde...");
        }

        // Esperar 2 segundos antes de repetir
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
