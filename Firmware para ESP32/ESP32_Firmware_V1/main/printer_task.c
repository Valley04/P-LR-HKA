// printer_task.c
#include "printer_config.h"
#include "printer_task.h"
#include "printer_mqtt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

// VARIABLES GLOBALES (declaradas en printer_config.h como extern)
mqtt_data_t mqtt_data;
const char* TAG_PRINTER = "printer_events";
QueueHandle_t printer_uart_queue = NULL;
SemaphoreHandle_t mqtt_data_mutex = NULL;

// VARIABLES ESTÁTICAS (solo visibles en este archivo)
static TaskHandle_t printer_task_handle = NULL;
static bool uart_initialized = false;
const uint8_t CMD_STATUS_S1[] = {0x53, 0x31};
static uint8_t rx_buffer[UART_BUFFER_SIZE];

// Contexto de la tarea
typedef struct {
    printer_state_t state;
    uint8_t reconnect_attempts;
    uint16_t error_count;
    uint32_t last_success_time;
    bool initialized;
} printer_task_context_t;

static printer_task_context_t task_ctx = {
    .state = PRINTER_STATE_IDLE,
    .reconnect_attempts = 0,
    .error_count = 0,
    .last_success_time = 0,
    .initialized = false
};

// FUNCIONES AUXILIARES (static para optimización)

// Función para enviar comando por UART
static bool uart_send_command(uint8_t command) {
    if (!uart_initialized) return false;
    
    int sent = uart_write_bytes(UART_PORT, (const char*)&command, 1);
    if (sent == 1) {
        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
        return true;
    }
    return false;
}

static bool uart_send_frame(const uint8_t *data, size_t len) {
    if (!uart_initialized || data == NULL || len == 0) return false;
    
    uint8_t stx = STX_BYTE;
    uint8_t etx = ETX_BYTE;
    uint8_t lrc = 0;

    // Calculamos el LRC
    for (size_t i = 1; i < len; i++) lrc ^= data[i];
    lrc ^= etx;

    // Enviamos datos parte por parte
    uart_write_bytes(UART_PORT, &stx, 1);
    uart_write_bytes(UART_PORT, data, len);
    uart_write_bytes(UART_PORT, &etx, 1);
    uart_write_bytes(UART_PORT, &lrc, 1);

    // Esperamos a que el hardware termine de transmitir
    esp_err_t err = uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PRINTER, "Error en transmisión UART: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

static int recibir_trama_generica(uint8_t* buffer, size_t longitud_total, uint32_t timeout_ms) {
    uint8_t byte_inicial;
    
    // Buscamos el STX (Inicio de trama)
    int r = uart_read_bytes(UART_PORT, &byte_inicial, 1, pdMS_TO_TICKS(timeout_ms));
    
    if (r <= 0 || byte_inicial != STX_BYTE) {
        return -1; // No empezó una trama válida
    }

    // Ahora pedimos el resto de la trama (4 bytes más: DATA1, DATA2, ETX, LRC)
    buffer[0] = STX_BYTE;
    size_t faltan = longitud_total - 1;
    int leidos = uart_read_bytes(UART_PORT, &buffer[1], faltan, pdMS_TO_TICKS(500));
    
    if (leidos < faltan) {
        return -2; // Trama incompleta
    }

    return longitud_total;
}

static void printer_uart_flush(void) {
    if (uart_initialized) {
        uart_flush_input(UART_PORT);
        
        // Verificar que esté vacío
        size_t available = 0;
        uart_get_buffered_data_len(UART_PORT, &available);
        if (available > 0) {
            ESP_LOGW(TAG_PRINTER, "Aún hay %d bytes en buffer después de flush", available);
        }
    }
}

// FUNCIONES PÚBLICAS (declaradas en printer_task.h)

// Inicialización UART
bool printer_uart_init(void) {
    if (uart_initialized) {
        ESP_LOGI(TAG_PRINTER, "UART ya está inicializado");
        return true;
    }
    
    ESP_LOGI(TAG_PRINTER, "Configurando UART...");
    
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Instalar driver UART
    ESP_LOGI(TAG_PRINTER, "Instalando driver UART...");
    esp_err_t err = uart_driver_install(UART_PORT, 
                                        UART_BUFFER_SIZE * 2,
                                        UART_BUFFER_SIZE * 2,
                                        UART_QUEUE_SIZE,
                                        &printer_uart_queue,
                                        0);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PRINTER, "Error al instalar UART: 0x%X", err);
        return false;
    }
    ESP_LOGI(TAG_PRINTER, "Driver UART instalado");
    
    // Configurar parámetros
    ESP_LOGI(TAG_PRINTER, "Configurando parámetros UART...");
    err = uart_param_config(UART_PORT, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PRINTER, "Error configurando UART: 0x%X", err);
        uart_driver_delete(UART_PORT);
        return false;
    }
    ESP_LOGI(TAG_PRINTER, "Parámetros UART configurados");
    
    // Configurar pines
    ESP_LOGI(TAG_PRINTER, "Configurando pines UART: TX=%d, RX=%d", 
             UART_TX_PIN, UART_RX_PIN);
    err = uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PRINTER, "Error configurando pines UART: 0x%X", err);
        uart_driver_delete(UART_PORT);
        return false;
    }
    ESP_LOGI(TAG_PRINTER, "Pines UART configurados");
    
    uart_initialized = true;
    task_ctx.initialized = true;
    task_ctx.state = PRINTER_STATE_IDLE;
    
    ESP_LOGI(TAG_PRINTER, "UART completamente inicializado y listo");
    ESP_LOGI(TAG_PRINTER, "Buffer: %d bytes, Cola: %d", 
             UART_BUFFER_SIZE, UART_QUEUE_SIZE);
    
    uart_flush_input(UART_PORT);

    ESP_LOGI(TAG_PRINTER, "UART inicializado y limpio");
    return true;
}

// Desinicialización UART
void printer_uart_deinit(void) {
    if (uart_initialized) {
        uart_driver_delete(UART_PORT);
        uart_initialized = false;
        task_ctx.initialized = false;
        LOG_I(TAG_PRINTER, "UART desinicializado");
    }
}

// Validar trama
bool validate_frame(const uint8_t* data, size_t len) {
    if (len < 5) return false; // Mínimo STX + DATA + ETX + LRC

    uint8_t lrc_recibido = data[len - 1];
    uint8_t etx_posicion = data[len - 2];
    uint8_t lrc_calculado = 0;

    if (data[0] != STX_BYTE || etx_posicion != ETX_BYTE) return false;

    // LRC se calcula desde DATA hasta ETX inclusive
    for (size_t i = 1; i < len - 1; i++) {
        lrc_calculado ^= data[i];
    }

    return (lrc_recibido == lrc_calculado);
}

// Obtener estado actual
printer_state_t printer_get_state(void) {
    return task_ctx.state;
}

// Establecer estado
void printer_set_state(printer_state_t state) {
    task_ctx.state = state;
}

// Convertir estado a string
const char* printer_state_to_string(printer_state_t state) {
    switch (state) {
        case PRINTER_STATE_IDLE: return "IDLE";
        case PRINTER_STATE_WAITING_RESPONSE: return "WAITING_RESPONSE";
        case PRINTER_STATE_PROCESSING: return "PROCESSING";
        case PRINTER_STATE_ERROR: return "ERROR";
        case PRINTER_STATE_RECONNECTING: return "RECONNECTING";
        case PRINTER_STATE_SENDING_STATUS_S1: return "SENDING_STATUS_S1";
        case PRINTER_STATE_WAITING_STATUS_S1: return "WAITING_STATUS_S1";
        default: return "UNKNOWN";
    }
}

// Interpretar estado (versión optimizada)
void guardar_estado(uint8_t sts1, uint8_t sts2) {
    mqtt_data.sts1 = sts1;
    mqtt_data.sts2 = sts2;

    mqtt_data.needs_sync = 1;
}

static uint32_t convertir_campo_a_uint32(const uint8_t* trama, int inicio, int longitud) {
    char mini_buffer[20]; // Espacio suficiente para cualquier campo numérico
    
    // Limitar la longitud para evitar desbordar nuestro mini_buffer
    if (longitud > 19) longitud = 19;

    // Copiar los datos de la trama al mini_buffer
    memcpy(mini_buffer, &trama[inicio], longitud);
    
    // Poner el caracter nulo para que strtoul sepa dónde terminar
    mini_buffer[longitud] = '\0';

    // Convertir y devolver el resultado
    return strtoul(mini_buffer, NULL, 10);
}

static void copiar_campo_string(const uint8_t* trama, int inicio, int longitud, char* destino) {
    // Copiamos los bytes exactos
    memcpy(destino, &trama[inicio], longitud);

    // Rematamos con el caracter nulo para que sea un string válido
    destino[longitud] = '\0';
}

// Procesar STATUS S1 (manteniendo TODOS los campos)
void procesar_status_s1(const uint8_t* data, size_t len) {
    if (len < STATUS_S1_MIN_LENGTH) {
        LOG_E(TAG_PRINTER, "Trama STATUS S1 muy corta: %d bytes", len);
        return;
    }
    
    // Buscar STX
    int stx_pos = -1;
    for (int i = 0; i < len; i++) {
        if (data[i] == STX_BYTE) {
            stx_pos = i;
            break;
        }
    }
    
    if (stx_pos == -1) {
        LOG_E(TAG_PRINTER, "No se encontró STX en STATUS S1");
        return;
    }
    
    const uint8_t* trama = &data[stx_pos + 1];

    if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {        
        // Extraer TODOS los campos según posiciones del protocolo
        copiar_campo_string(trama, 0, 4, mqtt_data.atm_number);
        mqtt_data.ventas = convertir_campo_a_uint32(trama, 4, 17);
        copiar_campo_string(trama, 21, 8, mqtt_data.last_bill_number);
        copiar_campo_string(trama, 29, 5, mqtt_data.bill_issue);
        copiar_campo_string(trama, 34, 8, mqtt_data.number_last_debit);
        mqtt_data.amount_debit = convertir_campo_a_uint32(trama, 42, 5);
        copiar_campo_string(trama, 47, 8, mqtt_data.number_last_credit);
        mqtt_data.amount_credit = convertir_campo_a_uint32(trama, 55, 5);
        copiar_campo_string(trama, 60, 8, mqtt_data.number_last_notfiscal);
        mqtt_data.amount_notfiscal = convertir_campo_a_uint32(trama, 68, 5);
        copiar_campo_string(trama, 73, 4, mqtt_data.counter_daily_z);
        copiar_campo_string(trama, 77, 4, mqtt_data.counter_report_fiscal);
        copiar_campo_string(trama, 81, 11, mqtt_data.rif_cliente);
        copiar_campo_string(trama, 92, 10, mqtt_data.register_number);
        copiar_campo_string(trama, 102, 6, mqtt_data.hour_machine);
        copiar_campo_string(trama, 108, 6, mqtt_data.date_machine);
        
        // Actualizar serial
        update_printer_serial(mqtt_data.register_number);
        
        // Actualizar timestamp
        mqtt_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        mqtt_data.needs_sync = true;

        // Devolvemos la llave
        xSemaphoreGive(mqtt_data_mutex);

        if (mqtt_task_handle != NULL) {
            xTaskNotifyGive(mqtt_task_handle);
        }
        LOG_I(TAG_PRINTER, "Datos de STATUS S1 actualizados");
    } else {
        LOG_W(TAG_PRINTER, "No se pudo obtener el Mutex, datos no actualizados");
    }
}

// SISTEMA DE RECONEXIÓN

bool printer_reconnect(void) {
    if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {

        task_ctx.state = PRINTER_STATE_RECONNECTING;
        
        // Desinicializar UART
        printer_uart_deinit();
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
        
        // Re-inicializar UART
        if (printer_uart_init()) {
            LOG_I(TAG_PRINTER, "Reconexión exitosa");
            // Limpiamos contadores
            task_ctx.reconnect_attempts = 0;
            task_ctx.error_count = 0;
            task_ctx.state = PRINTER_STATE_IDLE;

            // Liberamos la llave
            xSemaphoreGive(mqtt_data_mutex);
            return true;
        }

        // Si falla inicialización, liberamos llave de todos modos
        xSemaphoreGive(mqtt_data_mutex);
    }
    
    LOG_E(TAG_PRINTER, "Reconexión fallida");
    return false;
}

// Reiniciar contadores de error
void printer_reset_errors(void) {
    task_ctx.error_count = 0;
    task_ctx.reconnect_attempts = 0;
    task_ctx.state = PRINTER_STATE_IDLE;
}

// Verificar si necesita reconexión
bool printer_needs_reconnection(void) {
    if (task_ctx.error_count >= MAX_ERROR_COUNT) {
        return true;
    }
    
    if (task_ctx.state == PRINTER_STATE_ERROR) {
        return true;
    }
    
    // Verificar timeout prolongado
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((current_time - task_ctx.last_success_time) > (POLLING_INTERVAL_MS * 3)) {
        LOG_W(TAG_PRINTER, "Sin respuesta por mucho tiempo, considerando reconexión");
        return true;
    }
    
    return false;
}

// Manejar error
void printer_handle_error(printer_error_t error) {
    task_ctx.error_count++;
    mqtt_data.error_count = task_ctx.error_count;
    
    LOG_E(TAG_PRINTER, "Error %d - Total errores: %d", error, task_ctx.error_count);
    
    if (task_ctx.error_count >= MAX_ERROR_COUNT) {
        task_ctx.state = PRINTER_STATE_ERROR;
        LOG_E(TAG_PRINTER, "Límite de errores alcanzado, cambiando a estado ERROR");
    }
}

// Enviar comando ENQ
bool send_enq_command(void) {
    if (!uart_initialized) {
        return false;
    }
    
    task_ctx.state = PRINTER_STATE_WAITING_RESPONSE;
    return uart_send_command(ENQ_CMD);
}

// Recibir respuesta
int receive_response(uint8_t* buffer, size_t buffer_size, uint32_t timeout_ms) {
    return recibir_trama_generica(buffer, buffer_size, timeout_ms);
}

int ejecutar_solicitud_s1(uint8_t* buffer, int bytes_maximos) {
    ESP_LOGI(TAG_PRINTER, "Enviando comando S1...");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enviar comando S1
    if (!uart_send_frame(CMD_STATUS_S1, sizeof(CMD_STATUS_S1))) {
        ESP_LOGE(TAG_PRINTER, "Error al enviar S1");
        printer_handle_error(ERROR_COMMUNICATION_LOST);
        return 0;
        
    } 
    ESP_LOGI(TAG_PRINTER, "Comando S1 enviado, esperando ACK...");
    
    // Esperar ACK con timeout más corto
    uint8_t ack_buffer[1];
    int ack_received = uart_read_bytes(UART_PORT, ack_buffer, 1, pdMS_TO_TICKS(1000)); // 1 segundo
    
    if (ack_received > 0) {
        ESP_LOGI(TAG_PRINTER, "ACK/NAK recibido: 0x%02X", ack_buffer[0]);
    
        // Aquí validas si es ACK o NAK
        if (ack_buffer[0] != ACK_BYTE) {
            ESP_LOGI(TAG_PRINTER, "Byte recibido después de S1: 0x%02X", ack_buffer[0]);
            printer_handle_error(ERROR_NO_ACK);
            return 0;
        }
    } else {
        ESP_LOGW(TAG_PRINTER, "Timeout esperando ACK (Sin respuesta)");
    }

    ESP_LOGI(TAG_PRINTER, "ACK recibido, esperando STATUS S1...");
    printer_uart_flush();
    
    // Recibir STATUS S1 con timeout
    return receive_response(buffer, sizeof(bytes_maximos), 3000); // 3 segundos
}

// TAREA PRINCIPAL DE LA IMPRESORA
static void printer_task_main(void* pvParameters) {
    ESP_LOGI(TAG_PRINTER, "TAREA IMPRESORA INICIADA");
    task_ctx.initialized = true;
    task_ctx.state = PRINTER_STATE_IDLE;
    const uint8_t nak_msg = NAK_BYTE;
    
    while (1) {
        
        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));

        // Verificar si necesitamos reconexión
        if (printer_needs_reconnection()) {            
            if (!printer_reconnect()) {
                ESP_LOGE(TAG_PRINTER, "Reconexión fallida, esperando...");
                continue;
            }
        }        
                
        // Enviar comando ENQ
        ESP_LOGI(TAG_PRINTER, "Enviando comando ENQ...");
        if (!send_enq_command()) {
            ESP_LOGI(TAG_PRINTER, "Error al enviar ENQ");
            printer_handle_error(ERROR_COMMUNICATION_LOST);
            continue;
        }
            
        // Recibir respuesta
        int received = receive_response(rx_buffer, sizeof(rx_buffer), RESPONSE_TIMEOUT_MS);
        if (received <= 0) {
            ESP_LOGI(TAG_PRINTER, "Sin respuesta al ENQ (Timeout)");
            printer_handle_error(ERROR_NO_RESPONSE);
            continue;
        }
        
        // Validar trama
        if (!validate_frame(rx_buffer, received)) {
            uart_write_bytes(UART_PORT, &nak_msg, 1);
            ESP_LOGW(TAG_PRINTER, "Trama inválida detectada");
            printer_handle_error(ERROR_INVALID_RESPONSE);
            continue;
        } 

        // Procesar el estado recibido
        guardar_estado(rx_buffer[1], rx_buffer[2]);
            
        if (rx_buffer[1] != 0x60) {  // Modo fiscal y en espera
            ESP_LOGI(TAG_PRINTER, "Modo no permitido para solicitar S1");
            continue;            
        }

        int bytes_leidos = ejecutar_solicitud_s1(rx_buffer, received);

        if (bytes_leidos <= 0) {
            ESP_LOGW(TAG_PRINTER, "No se recibió respuesta S1 (timeout)");
            printer_handle_error(ERROR_STATUS_S1_FAILED);
            continue;
        }
    
        if (!validate_frame(rx_buffer, bytes_leidos)) {
            // Enviar NAK si la trama es inválida
            uart_write_bytes(UART_PORT, (const char[]){NAK_BYTE}, 1);
            ESP_LOGW(TAG_PRINTER, "STATUS S1 inválido → NAK");
            printer_handle_error(ERROR_INVALID_RESPONSE);
            continue;
        }

        // Enviar ACK de confirmación
        uart_write_bytes(UART_PORT, (const char[]){ACK_BYTE}, 1);
        ESP_LOGI(TAG_PRINTER, "STATUS S1 válido → ACK enviado");
        
        procesar_status_s1(rx_buffer, bytes_leidos);

        ESP_LOGI(TAG_PRINTER, "Ciclo completado exitosamente");        
    }
}

// Iniciar tarea de impresora
esp_err_t printer_task_start(void) {
    if (mqtt_data_mutex == NULL) {
        mqtt_data_mutex = xSemaphoreCreateMutex();
        if (mqtt_data_mutex == NULL) {
            ESP_LOGE(TAG_PRINTER, "No se pudo crear el Mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    if (printer_task_handle != NULL) return ESP_OK;
    
    BaseType_t result = xTaskCreate(
        printer_task_main,
        TASK_NAME,
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        &printer_task_handle
    );
    
    return (result == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}