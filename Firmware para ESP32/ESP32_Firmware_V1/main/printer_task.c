// printer_task.c
#include "printer_config.h"
#include "printer_task.h"
#include "printer_mqtt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

// VARIABLES GLOBALES (declaradas en printer_config.h como extern)
mqtt_data_t mqtt_data;
const char* TAG_PRINTER = "printer_events";
QueueHandle_t printer_spi_queue = NULL;
SemaphoreHandle_t mqtt_data_mutex = NULL;

// VARIABLES ESTÁTICAS (solo visibles en este archivo)
static TaskHandle_t printer_task_handle = NULL;
static bool spi_initialized = false;

spi_device_handle_t spi_printer = NULL;

const uint8_t CMD_STATUS_S1[] = {0x53, 0x31};
const uint8_t CMD_STATUS_SV2[] = {'S', 'V', '2'};

WORD_ALIGNED_ATTR static uint8_t rx_buffer[SPI_BUFFER_SIZE];
WORD_ALIGNED_ATTR static uint8_t tx_buffer[SPI_BUFFER_SIZE];

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

bool esperar_ack_impresora(uint32_t timeout_ms) {
    TickType_t tiempo_inicio = xTaskGetTickCount();
    TickType_t tiempo_limite = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - tiempo_inicio) < tiempo_limite) {

        // Limpiamos los buffers globales
        memset(tx_buffer, 0, SPI_BUFFER_SIZE);
        memset(rx_buffer, 0, SPI_BUFFER_SIZE);

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));

        t.length = 32;
        t.tx_buffer = tx_buffer;
        t.rx_buffer = rx_buffer;
        
        if (spi_device_transmit(spi_printer, &t) == ESP_OK) {
            uint8_t ack_rx = rx_buffer[0];
        
            // Si la lectura fue exitosa y la impresora nos devolvió algo distinto a silencio (0x00)
            if (ack_rx != 0xFF && ack_rx != 0x00) { 
                if (ack_rx == ACK_BYTE) {
                    ESP_LOGI(TAG_PRINTER, "✅ ACK recibido de la impresora.");
                    return true;
                } else if (ack_rx == NAK_BYTE) {
                    ESP_LOGE(TAG_PRINTER, "❌ NAK recibido. La impresora rechazó el comando.");
                    return false;
                } else {
                    ESP_LOGW(TAG_PRINTER, "⚠️ Byte inesperado recibido: 0x%02X", ack_rx);
                }
            }
        }
        // Si no hemos recibido el ACK, le damos un respiro a la impresora y volvemos a preguntar
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGE(TAG_PRINTER, "⏰ Timeout esperando ACK/NAK por SPI.");
    return false;
}

bool printer_get_fw_version(void) {

    //Version para iSmart
    if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(mqtt_data.fw_ismart, ISMART_VERSION, sizeof(mqtt_data.fw_ismart) - 1);
        mqtt_data.fw_ismart[sizeof(mqtt_data.fw_ismart) - 1] = '\0';
        xSemaphoreGive(mqtt_data_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    if (!spi_send_frame(CMD_STATUS_SV2, sizeof(CMD_STATUS_SV2))) {
        LOG_E(TAG_PRINTER, "Error enviando comando de versión");
        if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            strcpy(mqtt_data.fw_ismart, "000000");
            xSemaphoreGive(mqtt_data_mutex);
        }
        return false;
    }

    if (!esperar_ack_impresora(500)) {
        if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            strcpy(mqtt_data.fw_printer, "0000000000");
            xSemaphoreGive(mqtt_data_mutex);
        }
        return false; // Abortamos temprano
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    int bytes_leidos = receive_response(rx_buffer, sizeof(rx_buffer), 1000);    

    if (bytes_leidos > 15 && validate_frame(rx_buffer, bytes_leidos)) {

        int etx_pos = bytes_leidos - 2;
        int fw_len = 10; // 020507GD00

        // LRC
        if (etx_pos >= fw_len) {
            int start_idx = etx_pos - fw_len;
            
            if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Version para Impresora
                memset(mqtt_data.fw_printer, 0, sizeof(mqtt_data.fw_printer));
                strncpy(mqtt_data.fw_printer, (char*)&rx_buffer[start_idx], fw_len);
                ESP_LOGI(TAG_PRINTER, "✅ Versión detectada vía SV2: %s", mqtt_data.fw_printer);
                xSemaphoreGive(mqtt_data_mutex);
            }

            return true;
        }
    } else {
        ESP_LOGE(TAG_PRINTER, "❌ Trama SV2 inválida o error de Checksum (LRC)");
    }

    if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strcpy(mqtt_data.fw_printer, "0000000000");
        xSemaphoreGive(mqtt_data_mutex);
    }
    return false;
}

// Función para enviar comando por UART
bool spi_send_command(uint8_t command) {
    if (!spi_initialized) return false;  
    
    uint8_t *cmd_dma = heap_caps_calloc(1, 4, MALLOC_CAP_DMA); 
    if (cmd_dma == NULL) return false;

    cmd_dma[0] = command;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 32;
    t.tx_buffer = cmd_dma;
    t.rx_buffer = NULL;

    esp_err_t err = spi_device_transmit(spi_printer, &t);

    heap_caps_free(cmd_dma);

    return (err == ESP_OK);
}

bool spi_send_frame(const uint8_t *data, size_t len) {
    if (!spi_initialized || data == NULL || len == 0) return false;

    // Verificamos que la trama no desborde nuestro buffer seguro
    if (len + 3 > SPI_BUFFER_SIZE) {
        ESP_LOGE(TAG_PRINTER, "Trama demasiado larga para enviar por SPI");
        return false;
    }
    
    // Calculamos el LRC del payload
    uint8_t lrc = 0;
    for (size_t i = 0; i < len; i++) {
        lrc ^= data[i];
    }
    lrc ^= ETX_BYTE; // Incluimos el ETX en el LRC

    // EMPAQUETADO: Construimos la caja completa antes de enviarla
    tx_buffer[0] = STX_BYTE;
    memcpy(&tx_buffer[1], data, len);
    tx_buffer[1 + len] = ETX_BYTE;
    tx_buffer[2 + len] = lrc;

    // Tamaño total = 1 (STX) + len (Datos) + 1 (ETX) + 1 (LRC) = len + 3
    size_t tamano_total = len + 3;
    size_t tamano_dma = (tamano_total + 3) & ~3;

    for (size_t i = tamano_total; i < tamano_dma; i++) {
        tx_buffer[i] = 0x00;
    }

    spi_transaction_t t = {
        .length = tamano_total * 8, // En bits
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL // No esperamos respuesta en este milisegundo
    };

    esp_err_t err = spi_device_transmit(spi_printer, &t);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PRINTER, "Error en transmisión SPI: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

// FUNCIONES PÚBLICAS (declaradas en printer_task.h)

// Inicialización UART
bool printer_spi_init(void) {
    if (spi_initialized) return true;
    
    // Configuración del Bus (Pines físicos)
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_BUFFER_SIZE + 10 // Margen de seguridad para el DMA
    };

    // Configuración del Dispositivo (Cómo le hablamos a la impresora)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
        .mode = 0, // Modo SPI 0 (CPOL=0, CPHA=0)
        .spics_io_num = PIN_NUM_CS,
        .queue_size = SPI_QUEUE_SIZE,
    };

    // Inicializar el bus SPI
    esp_err_t ret = spi_bus_initialize(SPI_HOST_PORT, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PRINTER, "Error inicializando bus SPI: %s", esp_err_to_name(ret));
        return false;
    }

    // Añadir el dispositivo (Impresora) al bus
    ret = spi_bus_add_device(SPI_HOST_PORT, &devcfg, &spi_printer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PRINTER, "Error añadiendo dispositivo SPI: %s", esp_err_to_name(ret));
        spi_bus_free(SPI_HOST_PORT); // Limpiamos si falla
        return false;
    }
    
    spi_initialized = true;
    ESP_LOGI(TAG_PRINTER, "✅ Bus SPI inicializado y dispositivo anclado.");
    return true;
}

// Desinicialización UART
void printer_spi_deinit(void) {
    if (!spi_initialized) return;

    if (spi_printer != NULL) {
        spi_bus_remove_device(spi_printer);
        spi_printer = NULL;
    }

    spi_bus_free(SPI_HOST_PORT);
    
    spi_initialized = false;
    ESP_LOGI(TAG_PRINTER, "🛑 Bus SPI desinicializado (Hardware liberado).");
}

// Validar trama
bool validate_frame(const uint8_t* data, size_t len) {

    if (len < 5) {
        spi_send_command(NAK_BYTE);
        return false; // Mínimo STX + DATA + ETX + LRC
    }

    uint8_t lrc_recibido = data[len - 1];
    uint8_t etx_posicion = data[len - 2];
    uint8_t lrc_calculado = 0;

    if (data[0] != STX_BYTE || etx_posicion != ETX_BYTE) {
        spi_send_command(NAK_BYTE);
        return false;
    }

    // LRC se calcula desde DATA hasta ETX inclusive
    for (size_t i = 1; i < len - 1; i++) {
        lrc_calculado ^= data[i];
    }

    if (lrc_recibido == lrc_calculado) {
        spi_send_command(ACK_BYTE);
        return true;
    } else {
        spi_send_command(ACK_BYTE);
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
    if (xSemaphoreTake(mqtt_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        mqtt_data.sts1 = sts1;
        mqtt_data.sts2 = sts2;
        mqtt_data.needs_sync = 1;
        xSemaphoreGive(mqtt_data_mutex);
    }
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
        
        // Desinicializar SPI
        printer_spi_deinit();
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
        
        // Re-inicializar UART
        if (printer_spi_init()) {
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
    if (!spi_initialized) {
        return false;
    }
    
    task_ctx.state = PRINTER_STATE_WAITING_RESPONSE;
    return spi_send_command(ENQ_CMD);
}

// Recibir respuesta
int receive_response(uint8_t* buffer, size_t buffer_size, uint32_t timeout_ms) {
    TickType_t tiempo_inicio = xTaskGetTickCount();
    TickType_t tiempo_limite = pdMS_TO_TICKS(timeout_ms);
    
    while ((xTaskGetTickCount() - tiempo_inicio) < tiempo_limite) {
        
        // Limpiamos el buffer de transmisión para que inyecte reloj limpio
        memset(tx_buffer, 0, SPI_BUFFER_SIZE); 

        // 1. Extraemos todo el bloque de una sola vez
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));

        t.length = buffer_size * 8; 
        t.tx_buffer = tx_buffer;
        t.rx_buffer = buffer; // Los datos caen directo al buffer

        if (spi_device_transmit(spi_printer, &t) != ESP_OK) {
            return -1;
        }

        // 2. Fase de Parseo (Buscamos STX y ETX en el pajar de datos)
        int start_idx = -1;
        int etx_idx = -1;

        for (int i = 0; i < buffer_size; i++) {
            if (buffer[i] == STX_BYTE && start_idx == -1) {
                start_idx = i;
            }
            if (start_idx != -1 && buffer[i] == ETX_BYTE) {
                etx_idx = i;
                break; // Encontramos el final
            }
        }

        // 3. Si encontramos la trama completa, la recortamos
        if (start_idx != -1 && etx_idx != -1 && (etx_idx + 1 < buffer_size)) {
            int trama_len = (etx_idx - start_idx) + 2; // +1 por el ETX, +1 por el LRC
            
            // Movemos la trama útil al inicio del buffer
            memmove(buffer, &buffer[start_idx], trama_len);
            
            ESP_LOGI(TAG_PRINTER, "Trama Fiscal interceptada (%d bytes)", trama_len);
            return trama_len; // ¡Éxito absoluto!
        }
        
        // Si el Simulador aún no había enviado nada, le damos 50ms para pensar
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    return -1; // Nos rendimos
}

int ejecutar_solicitud_s1(uint8_t* buffer, int bytes_maximos) {
    ESP_LOGI(TAG_PRINTER, "Enviando comando S1...");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enviar comando S1
    if (!spi_send_frame(CMD_STATUS_S1, sizeof(CMD_STATUS_S1))) {
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
    vTaskDelay(pdMS_TO_TICKS(50));
    return receive_response(buffer, bytes_maximos, 3000); // 3 segundos
}

void enviar_byte_spi(uint8_t byte_a_enviar) {
    spi_transaction_t t = {
        .length = 8, 
        .tx_buffer = &byte_a_enviar,
        .rx_buffer = NULL // No nos importa la respuesta aquí
    };
    spi_device_transmit(spi_printer, &t);
}

// TAREA PRINCIPAL DE LA IMPRESORA
static void printer_task_main(void* pvParameters) {
    ESP_LOGI(TAG_PRINTER, "TAREA IMPRESORA INICIADA (Modo SPI)");
    task_ctx.initialized = true;
    task_ctx.state = PRINTER_STATE_IDLE;
    
    bool fw_version_obtained = false;
    
    while (1) {
        
        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));

        if (ota_en_progreso) {
            LOG_W(TAG_PRINTER, "OTA en progreso. Polling fiscal SPI en reposo...");
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
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
            
        // Recibir respuesta
        int received = receive_response(rx_buffer, SPI_BUFFER_SIZE, RESPONSE_TIMEOUT_MS);
        if (received <= 0) {
            ESP_LOGI(TAG_PRINTER, "Sin respuesta al ENQ (Timeout)");
            printer_handle_error(ERROR_NO_RESPONSE);
            continue;
        }
        
        // Validar trama
        if (!validate_frame(rx_buffer, received)) {
            ESP_LOGW(TAG_PRINTER, "Trama inválida detectada");
            printer_handle_error(ERROR_INVALID_RESPONSE);
            continue;
        } 

        // Procesar el estado recibido
        guardar_estado(rx_buffer[1], rx_buffer[2]);

        if (rx_buffer[1] == 0x60 && !fw_version_obtained) { // Modo fiscal
            ESP_LOGI(TAG_PRINTER, "Intentando obtener versiones del sistema");
            if (printer_get_fw_version()) {
                fw_version_obtained = true;
            } else {
                ESP_LOGW(TAG_PRINTER, "No se pudo obtener versión, continuando sin ella");
            }
        }

        int bytes_leidos = ejecutar_solicitud_s1(rx_buffer, SPI_BUFFER_SIZE);

        if (bytes_leidos <= 0) {
            ESP_LOGW(TAG_PRINTER, "No se recibió respuesta S1 (timeout)");
            printer_handle_error(ERROR_STATUS_S1_FAILED);
            continue;
        }
    
        if (!validate_frame(rx_buffer, bytes_leidos)) {
            printer_handle_error(ERROR_INVALID_RESPONSE);
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