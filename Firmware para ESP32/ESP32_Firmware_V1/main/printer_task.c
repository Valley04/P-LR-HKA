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
const uint8_t CMD_STATUS_SV2[] = {'S', 'V', '2'};
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

extern volatile bool ota_en_progreso;

// FUNCIONES AUXILIARES (static para optimización)

bool esperar_ack_impresora(uint32_t timeout_ms) {
    uint8_t ack_buffer[1];
    int ack_received = uart_read_bytes(UART_PORT, ack_buffer, 1, pdMS_TO_TICKS(timeout_ms));

    if (ack_received > 0) {    
        if (ack_buffer[0] == ACK_BYTE) {
            ESP_LOGI(TAG_PRINTER, "✅ ACK recibido de la impresora.");
            return true;

        } else if (ack_buffer[0] == NAK_BYTE) {
            ESP_LOGE(TAG_PRINTER, "❌ NAK recibido. La impresora rechazó el comando.");
            return false;
        } else {
            ESP_LOGW(TAG_PRINTER, "⚠️ Byte inesperado recibido: 0x%02X", ack_buffer[0]);
            return false;
        }
    }
    ESP_LOGE(TAG_PRINTER, "⏰ Timeout esperando ACK/NAK.");
    return false;
}

bool printer_get_fw_version(void) {

    //Version para iSmart
    strncpy(mqtt_data.fw_ismart, ISMART_VERSION, sizeof(mqtt_data.fw_ismart) - 1);
    mqtt_data.fw_ismart[sizeof(mqtt_data.fw_ismart) - 1] = '\0';
    
    uart_flush(UART_PORT);

    if (!uart_send_frame(CMD_STATUS_SV2, sizeof(CMD_STATUS_SV2))) {
        LOG_E(TAG_PRINTER, "Error enviando comando de versión");
        strcpy(mqtt_data.fw_ismart, "0000000000");
        return false;
    }

    if (!esperar_ack_impresora(500)) {
        strcpy(mqtt_data.fw_printer, "0000000000");
        return false; // Abortamos temprano
    }

    int bytes_leidos = receive_response(rx_buffer, sizeof(rx_buffer), 1000);    

    if (bytes_leidos > 15 && validate_frame(rx_buffer, bytes_leidos)) {

        int etx_pos = bytes_leidos - 2;
        int fw_len = 10; // 020507GD00

        // LRC
        if (etx_pos >= fw_len) {
            int start_idx = etx_pos - fw_len;
            
            // Version para Impresora
            memset(mqtt_data.fw_printer, 0, sizeof(mqtt_data.fw_printer));
            strncpy(mqtt_data.fw_printer, (char*)&rx_buffer[start_idx], fw_len);

            ESP_LOGI(TAG_PRINTER, "✅ Versión detectada vía SV2: %s", mqtt_data.fw_printer);
            return true;
        }
    } else {
        ESP_LOGE(TAG_PRINTER, "❌ Trama SV2 inválida o error de Checksum (LRC)");
    }

    strcpy(mqtt_data.fw_printer, "0000000000");
    return false;
}

// Función para enviar comando por UART
static bool uart_send_command(uint8_t command) {
    if (!uart_initialized) return false;
    uart_flush_input(UART_PORT);
    
    int sent = uart_write_bytes(UART_PORT, (const char*)&command, 1);
    if (sent == 1) {
        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
        return true;
    }
    return false;
}

bool uart_send_frame(const uint8_t *data, size_t len) {
    if (!uart_initialized || data == NULL || len == 0) return false;
    
    uint8_t stx = STX_BYTE;
    uint8_t etx = ETX_BYTE;
    uint8_t lrc = 0;

    // Calculamos el LRC
    for (size_t i = 0; i < len; i++) lrc ^= data[i];
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

// Esta función va dentro de printer_task.c

// En printer_task.c

static int recibir_trama_generica(uint8_t* buffer, size_t longitud_maxima, uint32_t timeout_ms) {
    uint8_t byte_temporal;
    int leidos;
    int total_bytes_recibidos = 0;
    
    TickType_t tiempo_inicio = xTaskGetTickCount();
    TickType_t tiempo_limite = pdMS_TO_TICKS(timeout_ms);
    bool stx_encontrado = false;

    // Buscamos STX
    while ((xTaskGetTickCount() - tiempo_inicio) < tiempo_limite) {
        leidos = uart_read_bytes(UART_PORT, &byte_temporal, 1, pdMS_TO_TICKS(50));
        
        if (leidos > 0) {
            if (byte_temporal == STX_BYTE) { // 0x02
                buffer[0] = STX_BYTE;
                total_bytes_recibidos = 1;
                stx_encontrado = true;
                break;
            } else {
                 ESP_LOGW(TAG_PRINTER, "🗑️ Ignorando basura: 0x%02X", byte_temporal);
            }
        }
    }

    if (!stx_encontrado) return -1; // Nunca llegó el inicio

    // Llegamos hasta ETX + 1
    while (total_bytes_recibidos < longitud_maxima) {
        // Usamos un timeout corto entre caracteres (ej. 100ms)
        leidos = uart_read_bytes(UART_PORT, &byte_temporal, 1, pdMS_TO_TICKS(100));
        
        if (leidos > 0) {
            buffer[total_bytes_recibidos] = byte_temporal;

            total_bytes_recibidos++;

            // Si encontramos el ETX (0x03), hemos terminado de leer el cuerpo
            if (byte_temporal == ETX_BYTE) { // 0x03
                // Leemos 1 byte más (el LRC)
                uint8_t lrc;
                leidos = uart_read_bytes(UART_PORT, &lrc, 1, pdMS_TO_TICKS(100));
                if (leidos > 0) {
                    buffer[total_bytes_recibidos] = lrc;
                    total_bytes_recibidos++;
                    ESP_LOGI(TAG_PRINTER, "Trama completa recibida (%d bytes)", total_bytes_recibidos);
                    return total_bytes_recibidos; 
                }
            }
        } else {
            // Pasó tiempo y no llegaron más bytes, pero ya teníamos el STX...
            ESP_LOGE(TAG_PRINTER, "Se cortó la trama (Timeout entre bytes)");
            return -2;
        }
    }
    ESP_LOGE(TAG_PRINTER, "Error: BUFFER LLENO (%d bytes) sin encontrar ETX", total_bytes_recibidos);
    return -3; // Buffer lleno sin encontrar ETX
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
    const uint8_t ack_msg = ACK_BYTE;
    const uint8_t nak_msg = NAK_BYTE;

    if (len < 5) {
        uart_write_bytes(UART_PORT, &nak_msg, 1);
        return false; // Mínimo STX + DATA + ETX + LRC
    }

    uint8_t lrc_recibido = data[len - 1];
    uint8_t etx_posicion = data[len - 2];
    uint8_t lrc_calculado = 0;

    if (data[0] != STX_BYTE || etx_posicion != ETX_BYTE) {
        uart_write_bytes(UART_PORT, &nak_msg, 1);
        return false;
    }

    // LRC se calcula desde DATA hasta ETX inclusive
    for (size_t i = 1; i < len - 1; i++) {
        lrc_calculado ^= data[i];
    }

    if (lrc_recibido == lrc_calculado) {
        uart_write_bytes(UART_PORT, &ack_msg, 1);
        return true;
    } else {
        uart_write_bytes(UART_PORT, &nak_msg, 1);
        return false;
    }
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

// Procesar STATUS S1 (manteniendo TODOS los campos)
void procesar_status_s1(uint8_t* data, size_t len) {
    if (len < STATUS_S1_MIN_LENGTH) {
        LOG_E(TAG_PRINTER, "Trama STATUS S1 muy corta: %d bytes", len);
        return;
    }
    
    // Buscar STX y ETX
    int stx_pos = -1;
    int etx_pos = -1; 
    for (int i = 0; i < len; i++) {
        if (data[i] == STX_BYTE)stx_pos = i;
        if (data[i] == ETX_BYTE)etx_pos = i;
    }
    
    if (stx_pos == -1 || etx_pos == -1 || etx_pos <= stx_pos) {
        LOG_E(TAG_PRINTER, "No se encontró STX o ETX válido en STATUS S1");
        return;
    }

    char* campos[20];
    int num_campos = 0;

    campos[num_campos++] = (char*)&data[stx_pos + 3];

    for (int i = stx_pos + 3; i < etx_pos; i++) {
        if (data[i] == 0x0A) {
            data[i] = '\0'; // Terminar el campo actual
            if (num_campos < 20) {
                campos[num_campos++] = (char*)&data[i + 1]; // El siguiente campo empieza después del espacio
            }
        }
    }
    data[etx_pos] = '\0'; // Terminar el último campo
    
    if (num_campos < 16) {
        LOG_E(TAG_PRINTER, "Faltan campos. Esperados >=16, recibidos: %d", num_campos);
        return;
    }

    if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {        
        // Extraer TODOS los campos según posiciones del protocolo
        strncpy(mqtt_data.atm_number, campos[0], sizeof(mqtt_data.atm_number) - 1);
        mqtt_data.ventas = strtoul(campos[1], NULL, 10);
        strncpy(mqtt_data.last_bill_number, campos[2], sizeof(mqtt_data.last_bill_number) - 1);
        strncpy(mqtt_data.bill_issue, campos[3], sizeof(mqtt_data.bill_issue) - 1);
        strncpy(mqtt_data.number_last_debit, campos[4], sizeof(mqtt_data.number_last_debit) - 1);
        mqtt_data.amount_debit = strtoul(campos[5], NULL, 10);
        strncpy(mqtt_data.number_last_credit, campos[6], sizeof(mqtt_data.number_last_credit) - 1);
        mqtt_data.amount_credit = strtoul(campos[7], NULL, 10);
        strncpy(mqtt_data.number_last_notfiscal, campos[8], sizeof(mqtt_data.number_last_notfiscal) - 1);
        mqtt_data.amount_notfiscal = strtoul(campos[9], NULL, 10);
        strncpy(mqtt_data.counter_daily_z, campos[10], sizeof(mqtt_data.counter_daily_z) - 1);
        strncpy(mqtt_data.counter_report_fiscal, campos[11], sizeof(mqtt_data.counter_report_fiscal) - 1);
        
        strncpy(mqtt_data.rif_cliente, campos[12], sizeof(mqtt_data.rif_cliente) - 1);
        strncpy(mqtt_data.register_number, campos[13], sizeof(mqtt_data.register_number) - 1);
        strncpy(mqtt_data.hour_machine, campos[14], sizeof(mqtt_data.hour_machine) - 1);
        strncpy(mqtt_data.date_machine, campos[15], sizeof(mqtt_data.date_machine) - 1);
        
        // Forzar cierres nulos por seguridad en estructura packed
        mqtt_data.register_number[sizeof(mqtt_data.register_number) - 1] = '\0';
        mqtt_data.rif_cliente[sizeof(mqtt_data.rif_cliente) - 1] = '\0';

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
    task_ctx.last_success_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
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

    uart_flush_input(UART_PORT);
    // Enviar comando S1
    if (!uart_send_frame(CMD_STATUS_S1, sizeof(CMD_STATUS_S1))) {
        ESP_LOGE(TAG_PRINTER, "Error al enviar S1");
        printer_handle_error(ERROR_COMMUNICATION_LOST);
        return 0;
        
    } 
    ESP_LOGI(TAG_PRINTER, "Comando S1 enviado, esperando ACK...");
    
    // Esperar ACK con timeout más corto
    if (!esperar_ack_impresora(500)) {
        printer_handle_error(ERROR_INVALID_RESPONSE); 
        return 0; // Abortamos, no habrá trama
    }

    ESP_LOGI(TAG_PRINTER, "ACK recibido, esperando STATUS S1...");
    return receive_response(buffer, UART_BUFFER_SIZE, 3000); // 3 segundos
}

// TAREA PRINCIPAL DE LA IMPRESORA
static void printer_task_main(void* pvParameters) {
    ESP_LOGI(TAG_PRINTER, "TAREA IMPRESORA INICIADA");
    task_ctx.initialized = true;
    task_ctx.state = PRINTER_STATE_IDLE;
    const uint8_t nak_msg = NAK_BYTE;
    bool fw_version_obtained = false;
    
    while (1) {
        
        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));

        if (ota_en_progreso) {
            LOG_W(TAG_PRINTER, "OTA en progreso. Tarea UART en reposo hasta que se actualice el equipo...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

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
            printer_reset_errors();
            continue;
        }
            
        // Recibir respuesta
        int received = receive_response(rx_buffer, UART_BUFFER_SIZE, RESPONSE_TIMEOUT_MS);
        if (received <= 0) {
            ESP_LOGI(TAG_PRINTER, "Sin respuesta al ENQ (Timeout)");
            printer_handle_error(ERROR_NO_RESPONSE);
            printer_reset_errors();
            continue;
        }
        
        // Validar trama
        if (!validate_frame(rx_buffer, received)) {
            uart_write_bytes(UART_PORT, &nak_msg, 1);
            ESP_LOGW(TAG_PRINTER, "Trama inválida detectada");
            printer_handle_error(ERROR_INVALID_RESPONSE);
            printer_reset_errors();
            continue;
        } 

        if (rx_buffer[1] == 0x60 && !fw_version_obtained) { // Modo fiscal
            ESP_LOGI(TAG_PRINTER, "Intentando obtener versiones del sistema");
            if (printer_get_fw_version()) {
                fw_version_obtained = true;
            } else {
                ESP_LOGW(TAG_PRINTER, "No se pudo obtener versión, continuando sin ella");
            }
        }

        // Procesar el estado recibido
        guardar_estado(rx_buffer[1], rx_buffer[2]);

        int bytes_leidos = ejecutar_solicitud_s1(rx_buffer, received);

        if (bytes_leidos <= 0) {
            ESP_LOGW(TAG_PRINTER, "No se recibió respuesta S1 (timeout)");
            printer_handle_error(ERROR_STATUS_S1_FAILED);
            printer_reset_errors();
            continue;
        }
    
        if (!validate_frame(rx_buffer, bytes_leidos)) {
            printer_handle_error(ERROR_INVALID_RESPONSE);
            printer_reset_errors();
            continue;
        }
        
        procesar_status_s1(rx_buffer, bytes_leidos);

        printer_reset_errors();

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