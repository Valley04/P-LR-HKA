/* Esta es la primera versión del firmware para la comunicación con la impresora fiscal
   Utiliza FreeRTOS y el driver UART del ESP32 para enviar el comando ENQ (0x05h)
   y recibir el estado de la impresora cada 10 segundos.
   
   Próximos pasos:
   - Implementar el análisis de la respuesta recibida de la impresora.
   - Añadir manejo de errores y reconexión en caso de fallo de comunicación.
   - Integrar con el dashboard MQTT para notificaciones en tiempo real.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG_PRINTER = "uart_events";

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM    (3)

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

static void printer_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    uint8_t enq_cmd = 0x05; // Carácter ENQ según el manual
    int reconnect_attempts = 0;

    for (;;) {
        // Enviar el comando por UART
        uart_write_bytes(EX_UART_NUM, (const char*) &enq_cmd, 1);
        ESP_LOGI(TAG_PRINTER, "Enviado comando ENQ a la impresora.");
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, pdMS_TO_TICKS(2000))) {
            bzero(dtmp, RD_BUF_SIZE);

            switch (event.type) {
                case UART_DATA:
                    int len = uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    uart_flush_input(EX_UART_NUM);

                    // Se muestra la respuesta recibida
                    ESP_LOGI(TAG_PRINTER, "Respuesta recibida de la impresora:");
                    for (int i = 0; i < len; i++) {
                        ESP_LOGI(TAG_PRINTER, "Byte %d: 0x%02X", i, dtmp[i]);
                    }

                    // Verificar si la respuesta es válida según el protocolo
                    if (dtmp[0] == 0x02 && dtmp[3] == 0x03) {
                        ESP_LOGI(TAG_PRINTER, "Respuesta válida recibida de la impresora.");
                        interpretar_estado(dtmp[1], dtmp[2]);
                        reconnect_attempts = 0; // Resetear intentos de reconexión tras una respuesta exitosa
                    } else {
                        ESP_LOGW(TAG_PRINTER, "Respuesta inválida recibida de la impresora.");
                    }
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG_PRINTER, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG_PRINTER, "uart frame error");
                    break;
                default:
                    ESP_LOGI(TAG_PRINTER, "uart event type: %d", event.type);
                    break;
            }
        } else {
            // No se recibe respuesta de la impresora
            ESP_LOGW(TAG_PRINTER, "No se recibió respuesta de la impresora.");
            reconnect_attempts++;
            if (reconnect_attempts >= 5) {
                ESP_LOGE(TAG_PRINTER, "No se pudo comunicar con la impresora después de varios intentos.");
                // Aquí se implementará el aviso através del Dashboard MQTT en el futuro y un formato de reconexión más robusto
            }
        }

        // Esperar un tiempo antes de volver a comenzar
        // IMPORTANTE: Es vital usar vTaskDelay para no bloquear el procesador
        vTaskDelay(pdMS_TO_TICKS(5000)); // Pregunta cada 10 segundos

    }
    free(dtmp);
    vTaskDelete(NULL);
}

void interpretar_estado(uint8_t sts1, uint8_t sts2) 
{
    //Procesar STS1 (Estado)
    switch(sts1) {
        case 0x40: 
            ESP_LOGI(TAG_PRINTER, "Modo Entrenamiento y en Espera");
            break;
        case 0x41:
            ESP_LOGI(TAG_PRINTER, "Modo Entrenamiento y en medio de una Transacción Fiscal");
            break;
        case 0x42:
            ESP_LOGI(TAG_PRINTER, "Modo Entrenamiento y en medio de una Transacción No Fiscal");
            break;
        case 0x60: 
            ESP_LOGI(TAG_PRINTER, "Modo Fiscal y en Espera");
            break;
        case 0x68:
            ESP_LOGI(TAG_PRINTER, "Modo Fiscal con la MF llena y en Espera");
            break;
        case 0x61:
            ESP_LOGI(TAG_PRINTER, "Modo Fiscal y en medio de una Transacción Fiscal");
            break;
        case 0x69:
            ESP_LOGI(TAG_PRINTER, "Modo Fiscal con la MF llena y en medio de una Transacción Fiscal");
            break;
        case 0x62:
            ESP_LOGI(TAG_PRINTER, "Modo Fiscal y en medio de una Transacción No Fiscal");
            break;
        case 0x6A:
            ESP_LOGI(TAG_PRINTER, "Modo Fiscal con la MF llena y en Transacción No Fiscal");
            break;
    }

    //Procesar STS2 (Error)
    switch(sts1) {
        case 0x40: 
            ESP_LOGI(TAG_PRINTER, "Ningún error");
            break;
        case 0x48:
            ESP_LOGI(TAG_PRINTER, "Error gaveta");
            break;
        case 0x41:
            ESP_LOGI(TAG_PRINTER, "Error sin papel");
            break;
        case 0x42:
            ESP_LOGI(TAG_PRINTER, "Error mecánico de la impresora / papel");
            break;
        case 0x43:
            ESP_LOGI(TAG_PRINTER, "Error mecanico de la impresora y fin de papel");
            break;
        case 0x60:
            ESP_LOGI(TAG_PRINTER, "Error fiscal");
            break;
        case 0x64:
            ESP_LOGI(TAG_PRINTER, "Error en la memoria fiscal");
            break;
        case 0x6C:
            ESP_LOGI(TAG_PRINTER, "Error memoria fiscal llena");
            break;
    }
}

void app_main(void)
{
    //esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set(TAG_PRINTER, ESP_LOG_INFO);

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG_PRINTER, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, 18, 19, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    // Crear la tarea: (función, nombre, stack, parámetros, prioridad, handle)
    xTaskCreate(printer_task, "printer_0x05h_send", 4096, NULL, 5, NULL);
}