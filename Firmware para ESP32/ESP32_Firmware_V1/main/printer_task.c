// printer_task.c
#include "printer_config.h"
#include "printer_task.h"
#include "printer_mqtt.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

// VARIABLES GLOBALES (declaradas en printer_config.h como extern)
mqtt_data_t mqtt_data;
const char* TAG_PRINTER = "printer_events";
QueueHandle_t printer_uart_queue = NULL;

// VARIABLES ESTÁTICAS (solo visibles en este archivo)
static TaskHandle_t printer_task_handle = NULL;
static bool uart_initialized = false;
const uint8_t CMD_STATUS_S1[] = {0x53, 0x31};

// Tablas de strings en FLASH para ahorrar RAM
static const char* const estado_strings[] __attribute__((section(".rodata"))) = {
    [0x40] = "Modo Entrenamiento y en Espera",
    [0x41] = "Modo Entrenamiento y en medio de una Transacción Fiscal",
    [0x42] = "Modo Entrenamiento y en medio de una Transacción No Fiscal",
    [0x60] = "Modo Fiscal y en Espera",
    [0x68] = "Modo Fiscal con la MF llena y en Espera",
    [0x61] = "Modo Fiscal y en medio de una Transacción Fiscal",
    [0x69] = "Modo Fiscal con la MF llena y en medio de una Transacción Fiscal",
    [0x62] = "Modo Fiscal y en medio de una Transacción No Fiscal",
    [0x6A] = "Modo Fiscal con la MF llena y en Transacción No Fiscal"
};

static const char* const error_strings[] __attribute__((section(".rodata"))) = {
    [0x40] = "Ningún error",
    [0x48] = "Error gaveta",
    [0x41] = "Error sin papel",
    [0x42] = "Error mecánico de la impresora / papel",
    [0x43] = "Error mecanico de la impresora y fin de papel",
    [0x60] = "Error fiscal",
    [0x64] = "Error en la memoria fiscal",
    [0x6C] = "Error memoria fiscal llena"
};

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

static bool uart_send_frame(const uint8_t *data, size_t len)
{
    if (!uart_initialized || data == NULL || len == 0) {
        return false;
    }

    uint8_t frame[UART_BUFFER_SIZE];
    size_t idx = 0;

    // STX
    frame[idx++] = STX_BYTE;

    // DATA (puede ser 1 o más bytes)
    for (size_t i = 0; i < len; i++) {
        frame[idx++] = data[i];
    }

    // ETX
    frame[idx++] = ETX_BYTE;

    // LRC = XOR desde DATA hasta ETX
    uint8_t lrc = 0;
    for (size_t i = 1; i < idx; i++) {
        lrc ^= frame[i];
    }
    frame[idx++] = lrc;

    int sent = uart_write_bytes(UART_PORT, (const char *)frame, idx);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));

    ESP_LOGI(TAG_PRINTER, "Frame enviado (%d bytes):", idx);

    return sent == idx;
}



// Función para recibir datos con timeout
static int uart_receive_with_timeout(uint8_t* buffer, size_t buffer_size, uint32_t timeout_ms) {
    if (!uart_initialized || buffer == NULL || buffer_size == 0) {
        return -1;
    }
    
    TickType_t start_time = xTaskGetTickCount();
    int total_received = 0;
    bool etx_detected = false;
    
    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms)) {
        size_t available = 0;
        uart_get_buffered_data_len(UART_PORT, &available);
        
        if (available > 0) {
            // Leer todos los bytes disponibles
            int to_read = (available < (buffer_size - total_received)) ?
                           available : (buffer_size - total_received);
            int received = uart_read_bytes(UART_PORT, buffer + total_received, to_read, 0);
            
            if (received > 0) {
                total_received += received;
                
                // Verificar si ya tenemos una trama completa
                if (total_received >= 5) {  // Mínimo STX + DATA + ETX + LRC
                    for (int i = 0; i < total_received; i++) {
                        if (buffer[i] == STX_BYTE) {
                            // Buscar ETX desde STX
                            for (int j = i + 1; j < total_received; j++) {
                                if (buffer[j] == ETX_BYTE && (j + 1) < total_received) {
                                    // Tenemos STX...ETX + al menos 1 byte para LRC
                                    return total_received;
                                }
                            }
                        }
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeña pausa
    }
    
    return total_received;
}

// Función para limpiar buffer UART
static void printer_uart_flush(void) {
    if (uart_initialized) {
        uart_flush_input(UART_PORT);
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

// Interpretar estado (versión optimizada)
void interpretar_estado(uint8_t sts1, uint8_t sts2) {
    const char* estado_str = "Estado desconocido";
    const char* error_str = "Error desconocido";
    
    // Buscar estado en tabla
    if (sts1 < sizeof(estado_strings)/sizeof(estado_strings[0]) && 
        estado_strings[sts1] != NULL) {
        estado_str = estado_strings[sts1];
    }
    
    // Buscar error en tabla
    if (sts2 < sizeof(error_strings)/sizeof(error_strings[0]) && 
        error_strings[sts2] != NULL) {
        error_str = error_strings[sts2];
    }
    
    LOG_I(TAG_PRINTER, "%s", estado_str);
    LOG_I(TAG_PRINTER, "%s", error_str);
    
    // Preparar datos para MQTT
    prepare_mqtt_payload(sts1, sts2, estado_str, error_str);
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
    size_t trama_len = len - stx_pos - 3;

    
    if (trama_len < STATUS_S1_MIN_LENGTH) {
        LOG_E(TAG_PRINTER, "Trama después de STX muy corta: %d", trama_len);
        return;
    }
    
    // Extraer TODOS los campos según posiciones del protocolo
    // Número de Cajero (posición 1-4 después de STX)
    memcpy(mqtt_data.atm_number, &trama[0], 4);
    mqtt_data.atm_number[4] = '\0';
    
    // Subtotal de Ventas (posición 5-21)
    memcpy(mqtt_data.ventas, &trama[4], 17);
    mqtt_data.ventas[17] = '\0';
    
    // Número de última factura (posición 22-29)
    memcpy(mqtt_data.last_bill_number, &trama[21], 8);
    mqtt_data.last_bill_number[8] = '\0';
    
    // Cantidad de facturas emitidas (posición 30-34)
    memcpy(mqtt_data.bill_issue, &trama[29], 5);
    mqtt_data.bill_issue[5] = '\0';
    
    // Número de la última nota de débito (posición 35-42)
    memcpy(mqtt_data.number_last_debit, &trama[34], 8);
    mqtt_data.number_last_debit[8] = '\0';
    
    // Cantidad de notas de débito del día (posición 43-47)
    memcpy(mqtt_data.amount_debit, &trama[42], 5);
    mqtt_data.amount_debit[5] = '\0';
    
    // Número de la última nota de crédito (posición 48-55)
    memcpy(mqtt_data.number_last_credit, &trama[47], 8);
    mqtt_data.number_last_credit[8] = '\0';
    
    // Cantidad de notas de crédito (posición 56-60)
    memcpy(mqtt_data.amount_credit, &trama[55], 5);
    mqtt_data.amount_credit[5] = '\0';
    
    // Número del último documento no fiscal (posición 61-68)
    memcpy(mqtt_data.number_last_notfiscal, &trama[60], 8);
    mqtt_data.number_last_notfiscal[8] = '\0';
    
    // Cantidad de documentos no fiscales (posición 69-73)
    memcpy(mqtt_data.amount_notfiscal, &trama[68], 5);
    mqtt_data.amount_notfiscal[5] = '\0';
    
    // Contador de cierres diarios (Z) (posición 74-77)
    memcpy(mqtt_data.counter_daily_z, &trama[73], 4);
    mqtt_data.counter_daily_z[4] = '\0';
    
    // Contador de reportes de memoria fiscal (posición 78-81)
    memcpy(mqtt_data.counter_report_fiscal, &trama[77], 4);
    mqtt_data.counter_report_fiscal[4] = '\0';
    
    // RIF (posición 82-92)
    memcpy(mqtt_data.rif_cliente, &trama[81], 11);
    mqtt_data.rif_cliente[11] = '\0';
    
    // Número de registro (posición 93-102) - ¡ESTE ES EL SERIAL!
    memcpy(mqtt_data.register_number, &trama[92], 10);
    mqtt_data.register_number[10] = '\0';
    
    // Hora actual (posición 103-108)
    memcpy(mqtt_data.hour_machine, &trama[102], 6);
    mqtt_data.hour_machine[6] = '\0';
    
    // Fecha actual (posición 109-114)
    memcpy(mqtt_data.date_machine, &trama[108], 6);
    mqtt_data.date_machine[6] = '\0';
    
    // Actualizar serial
    update_printer_serial(mqtt_data.register_number);
    
    // Actualizar timestamp
    mqtt_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    mqtt_data.needs_sync = true;
    
    LOG_I(TAG_PRINTER, "STATUS S1 procesado exitosamente");
    LOG_I(TAG_PRINTER, "Serial: %s", mqtt_data.register_number);
}

// Validar trama
bool validate_frame(const uint8_t* data, size_t len) {
    if (len < 5) return false; // Mínimo STX + DATA + ETX + LRC
    if (data[0] != STX_BYTE) return false;
    if (data[len-2] != ETX_BYTE) return false;

    uint8_t lrc_rx = data[len - 1];
    uint8_t lrc_calc = 0;

    // LRC se calcula desde DATA hasta ETX inclusive
    for (size_t i = 1; i < len - 1; i++) {
        lrc_calc ^= data[i];
    }

    return (lrc_rx == lrc_calc);
}

// Verificar respuesta válida
bool is_valid_response(const uint8_t* data, size_t len) {
    return validate_frame(data, len);
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

// SISTEMA DE RECONEXIÓN

bool printer_reconnect(void) {
    if (task_ctx.reconnect_attempts >= MAX_RECONNECT_ATTEMPTS) {
        LOG_E(TAG_PRINTER, "Máximo de intentos de reconexión alcanzado (%d)", 
              MAX_RECONNECT_ATTEMPTS);
        task_ctx.state = PRINTER_STATE_ERROR;
        mqtt_data.reconnect_attempts = task_ctx.reconnect_attempts;
        return false;
    }
    
    task_ctx.reconnect_attempts++;
    task_ctx.state = PRINTER_STATE_RECONNECTING;
    mqtt_data.reconnect_attempts = task_ctx.reconnect_attempts;
    
    LOG_W(TAG_PRINTER, "Intentando reconexión %d/%d...", 
          task_ctx.reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
    
    // 1. Desinicializar UART
    printer_uart_deinit();
    
    // 2. Esperar antes de reintentar
    vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    
    // 3. Re-inicializar UART
    if (printer_uart_init()) {
        LOG_I(TAG_PRINTER, "Reconexión exitosa");
        task_ctx.reconnect_attempts = 0;
        task_ctx.error_count = 0;
        task_ctx.state = PRINTER_STATE_IDLE;
        mqtt_data.reconnect_attempts = 0;
        mqtt_data.error_count = 0;
        return true;
    }
    
    LOG_E(TAG_PRINTER, "Reconexión fallida");
    task_ctx.state = PRINTER_STATE_ERROR;
    return false;
}

// Reiniciar contadores de error
void printer_reset_errors(void) {
    task_ctx.error_count = 0;
    task_ctx.reconnect_attempts = 0;
    mqtt_data.error_count = 0;
    mqtt_data.reconnect_attempts = 0;
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
    return uart_receive_with_timeout(buffer, buffer_size, timeout_ms);
}

// Imprimir estadísticas
void print_task_stats(void) {
    LOG_I(TAG_PRINTER, "=== ESTADÍSTICAS DE TAREA ===");
    LOG_I(TAG_PRINTER, "Estado: %s", printer_state_to_string(task_ctx.state));
    LOG_I(TAG_PRINTER, "Éxitos: %lu", task_ctx.last_success_time);
    LOG_I(TAG_PRINTER, "Errores: %d", task_ctx.error_count);
    LOG_I(TAG_PRINTER, "Intentos reconexión: %d", task_ctx.reconnect_attempts);
    LOG_I(TAG_PRINTER, "==============================");
}

// Obtener high water mark del stack
size_t get_task_stack_high_water_mark(void) {
    if (printer_task_handle != NULL) {
        return uxTaskGetStackHighWaterMark(printer_task_handle);
    }
    return 0;
}

// TAREA PRINCIPAL DE LA IMPRESORA

static void printer_task_main(void* pvParameters) {
    ESP_LOGI(TAG_PRINTER, "=== TAREA IMPRESORA INICIADA ===");
    ESP_LOGI(TAG_PRINTER, "Stack disponible: %d bytes", 
             uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGI(TAG_PRINTER, "Prioridad: %d", uxTaskPriorityGet(NULL));
    
    uint8_t rx_buffer[UART_BUFFER_SIZE];
    int cycle_count = 0;
    
    while (1) {
        cycle_count++;
        
        // Mostrar estado cada 5 ciclos
        if (cycle_count % 5 == 0) {
            ESP_LOGI(TAG_PRINTER, "Ciclo %d - Estado: %s, Errores: %d", 
                    cycle_count, 
                    printer_state_to_string(task_ctx.state),
                    task_ctx.error_count);
        }
        
        // Verificar si necesitamos reconexión
        if (printer_needs_reconnection()) {
            ESP_LOGW(TAG_PRINTER, "Necesita reconexión. Estado actual: %s", 
                    printer_state_to_string(task_ctx.state));
            
            if (!printer_reconnect()) {
                ESP_LOGE(TAG_PRINTER, "Reconexión fallida, esperando...");
                vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
                continue;
            }
        }
        
        // Solo proceder si estamos en estado IDLE y UART inicializado
        if (task_ctx.state != PRINTER_STATE_IDLE || !task_ctx.initialized) {
            ESP_LOGW(TAG_PRINTER, "Esperando estado IDLE. Estado actual: %s", 
                    printer_state_to_string(task_ctx.state));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // 1. Enviar comando ENQ
        ESP_LOGI(TAG_PRINTER, "Enviando comando ENQ (0x%02X)...", ENQ_CMD);
        if (send_enq_command()) {
            ESP_LOGI(TAG_PRINTER, "ENQ enviado, esperando respuesta...");
            
            // 2. Recibir respuesta
            int received = receive_response(rx_buffer, sizeof(rx_buffer), RESPONSE_TIMEOUT_MS);
            
            if (received > 0) {
                ESP_LOGI(TAG_PRINTER, "Bytes recibidos: %d", received);
                
                if (validate_frame(rx_buffer, received)) {
                    ESP_LOGI(TAG_PRINTER, "Trama válida recibida");
                    ESP_LOGI(TAG_PRINTER, "STX: 0x%02X, STS1: 0x%02X, STS2: 0x%02X, ETX: 0x%02X",
                            rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[received-2]);
                    
                    // 3. Procesar estado
                    interpretar_estado(rx_buffer[1], rx_buffer[2]);
                    
                    // 4. Si estamos en modo fiscal y espera, solicitar STATUS S1
                    if (rx_buffer[1] == 0x60) {
                        ESP_LOGI(TAG_PRINTER, "Impresora en modo fiscal, solicitando STATUS S1...");
                        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));

                        ESP_LOGI(TAG_PRINTER, "Enviando comando S1...")
                        if (uart_send_frame(CMD_STATUS_S1, sizeof(CMD_STATUS_S1))) {
                            ESP_LOGI(TAG_PRINTER, "Comando S1 enviado, esperando ACK...");

                            // Esperar a que se transmita completamente
                            uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));

                            printer_uart_flush();

                            // Esperar ACK con timeout más corto
                            uint8_t ack_buffer[1];
                            int ack_received = uart_receive_with_timeout(ack_buffer, 1, 1000); // 1 segundo

                            if (ack_received > 0) {
                                ESP_LOGI(TAG_PRINTER, "ACK recibido, esperando respuesta S1... 0x%02X", ack_buffer[0]);
                                
                                if (ack_buffer[0] == ACK_BYTE) {
                                    ESP_LOGI(TAG_PRINTER, "ACK recibido, esperando respuesta S1...");

                                    print_uart_flush();

                                    // Ahora esperar la trama S1
                                    received = receive_response(rx_buffer, sizeof(rx_buffer), STATUS_S1_TIMEOUT_MS);
                                if (received > 0) {
                                    ESP_LOGI(TAG_PRINTER, "Posible S1 recibido: %d bytes", received);
                                    
                                    // DEBUG: Mostrar bytes recibidos
                                    for (int i = 0; i < received; i++) {
                                        ESP_LOGI(TAG_PRINTER, "[%d]: 0x%02X", i, rx_buffer[i]);
                                    }
                                    
                                    if (validate_frame(rx_buffer, received)) {
                                        // Enviar ACK de confirmación
                                        uart_write_bytes(UART_PORT, (const char[]){ACK_BYTE}, 1);
                                        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
                                        ESP_LOGI(TAG_PRINTER, "STATUS S1 válido → ACK enviado");
                                        
                                        procesar_status_s1(rx_buffer, received);
                                    } else {
                                        // Enviar NAK si la trama es inválida
                                        uart_write_bytes(UART_PORT, (const char[]){NAK_BYTE}, 1);
                                        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
                                        ESP_LOGW(TAG_PRINTER, "STATUS S1 inválido → NAK");
                                        printer_handle_error(ERROR_INVALID_RESPONSE);
                                        printer_uart_flush();
                                    }
                                } else {
                                    ESP_LOGW(TAG_PRINTER, "No se recibió respuesta S1 (timeout)");
                                    printer_handle_error(ERROR_STATUS_S1_FAILED);
                                }
                            } else {
                                ESP_LOGW(TAG_PRINTER, "Se recibió 0x%02X en lugar de ACK", ack_buffer[0]);
                                printer_handle_error(ERROR_NO_ACK);
                            }
                        } else {
                            ESP_LOGW(TAG_PRINTER, "No se recibió ACK para comando S1 (timeout)");
                            printer_handle_error(ERROR_NO_ACK);
                        }
                    } else {
                        ESP_LOGE(TAG_PRINTER, "Error al enviar comando S1");
                        printer_handle_error(ERROR_COMMUNICATION_LOST);
                    }
                }

                } else {
                    uart_write_bytes(UART_PORT, (const char[]){NAK_BYTE}, 1);
                    LOG_E(TAG_PRINTER, "Trama inválida → NAK");
                    printer_handle_error(ERROR_INVALID_RESPONSE);
                    printer_uart_flush();
                }
            } else {
                ESP_LOGW(TAG_PRINTER, "No se recibió respuesta (timeout)");
                printer_handle_error(ERROR_NO_RESPONSE);
                printer_uart_flush();
            }
        } else {
            ESP_LOGE(TAG_PRINTER, "Error al enviar ENQ");
            printer_handle_error(ERROR_COMMUNICATION_LOST);
        }
        
        // Esperar antes del próximo ciclo
        ESP_LOGI(TAG_PRINTER, "Esperando %d ms antes del próximo ciclo...", POLLING_INTERVAL_MS);
        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));
    }
}

// Iniciar tarea de impresora
bool printer_task_start(void) {
    if (printer_task_handle != NULL) {
        return true; // Ya está corriendo
    }
    
    BaseType_t result = xTaskCreate(
        printer_task_main,
        TASK_NAME,
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        &printer_task_handle
    );
    
    if (result == pdPASS) {
        LOG_I(TAG_PRINTER, "Tarea de impresora creada exitosamente");
        return true;
    } else {
        LOG_E(TAG_PRINTER, "Error al crear tarea de impresora");
        return false;
    }
}

// Detener tarea de impresora
void printer_task_stop(void) {
    if (printer_task_handle != NULL) {
        vTaskDelete(printer_task_handle);
        printer_task_handle = NULL;
        LOG_I(TAG_PRINTER, "Tarea de impresora detenida");
    }
}

// Suspender tarea
void printer_task_suspend(void) {
    if (printer_task_handle != NULL) {
        vTaskSuspend(printer_task_handle);
        LOG_I(TAG_PRINTER, "Tarea de impresora suspendida");
    }
}

// Reanudar tarea
void printer_task_resume(void) {
    if (printer_task_handle != NULL) {
        vTaskResume(printer_task_handle);
        LOG_I(TAG_PRINTER, "Tarea de impresora reanudada");
    }
}