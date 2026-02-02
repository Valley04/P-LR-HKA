/* Esta es la primera versión (totalmente adaptada) del firmware para la comunicación con la impresora fiscal
   Utiliza FreeRTOS y el driver UART del ESP32 para enviar el comando ENQ (0x05h) y STATUS S1 (0x4Ch)
   y recibir el estado e información de la impresora cada 10 segundos.
   
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

#define MQTT_TOPIC_BASE "v1/fiscal"
#define MAX_SERIAL_LENGTH 11

typedef struct {
    // Datos básicos
    uint8_t sts1;
    uint8_t sts2;
    char estado_str[50];
    char error_str[50];

    // Datos STATUS S1
    char atm_number[5];
    char ventas[18];
    char last_bill_number[9];
    char bill_issue[6];
    char number_last_debit[9];
    char amount_debit[6];
    char number_last_credit[9];
    char amount_credit[6];
    char number_last_notfiscal[9];
    char amount_notfiscal[6];
    char counter_daily_z[5];
    char counter_report_fiscal[5];
    char rif_cliente[12];
    char register_number[11];
    char hour_machine[7];
    char date_machine[7];

    // Metadatos
    uint32_t timestamp;
    uint8_t signal_strength; // Para futuro WiFi
    bool needs_sync;
    bool serial_obtenido;
} mqtt_data_t;

static const char *TAG_PRINTER = "uart_events";
static mqtt_data_t mqtt_data;
static QueueHandle_t uart0_queue = NULL;

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM    (3)

#define UART_BUFFER_SIZE (1024)

// Inicializar estructura MQTT
static void mqtt_data_init(mqtt_data_t *data) {
    memset(data, 0, sizeof(mqtt_data_t));
    data->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    data->needs_sync = false;
    data->serial_obtenido = false;
    // Serial por defecto hasta obtener el real
    strcpy(data->register_number, "DESCONOCIDO");
}

// Preparar tópico MQTT completo con serial dinámico
static void prepare_mqtt_topic(char *topic, size_t topic_size, const char *data_type) {
    snprintf(topic, topic_size, "%s/%s/%s", 
             MQTT_TOPIC_BASE, 
             mqtt_data.register_number, 
             data_type);
}

// Formatear datos para JSON (para futuro)
static void format_mqtt_json(char *buffer, size_t buffer_size, mqtt_data_t *data) {
    snprintf(buffer, buffer_size,
        "{\"timestamp\":%lu,\"sts1\":\"0x%02X\",\"sts2\":\"0x%02X\","
        "\"estado\":\"%s\",\"error\":\"%s\",\"cajero\":\"%s\","
        "\"ventas\":\"%s\",\"ult_factura\":\"%s\",\"fact_emitidas\":\"%s\","
        "\"rif\":\"%s\",\"registro\":\"%s\",\"hora\":\"%s\",\"fecha\":\"%s\"}",
        data->timestamp,
        data->sts1, data->sts2,
        data->estado_str, data->error_str,
        data->atm_number,
        data->ventas,
        data->last_bill_number,
        data->bill_issue,
        data->rif_cliente,
        data->register_number,
        data->hour_machine,
        data->date_machine);
}

// Actualizar serial de la impresora
static void update_printer_serial(const char *serial) {
    if (serial != NULL && strlen(serial) > 0) {
        strncpy(mqtt_data.register_number, serial, MAX_SERIAL_LENGTH);
        mqtt_data.register_number[MAX_SERIAL_LENGTH-1] = '\0';
        mqtt_data.serial_obtenido = true;
        ESP_LOGI(TAG_PRINTER, "Serial de impresora actualizado: %s", mqtt_data.register_number);
    }
}

// Preparar datos para envío MQTT
static void prepare_mqtt_payload(uint8_t sts1, uint8_t sts2, const char *estado_str, const char *error_str) {
    // Solo inicializar si no tenemos serial aún
    if (!mqtt_data.serial_obtenido) {
        mqtt_data_init(&mqtt_data);
    }
    
    mqtt_data.sts1 = sts1;
    mqtt_data.sts2 = sts2;
    
    if (estado_str) {
        strncpy(mqtt_data.estado_str, estado_str, sizeof(mqtt_data.estado_str) - 1);
    }
    
    if (error_str) {
        strncpy(mqtt_data.error_str, error_str, sizeof(mqtt_data.error_str) - 1);
    }
    
    mqtt_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    mqtt_data.needs_sync = true;
    
    // Mostrar lo que se enviaría por MQTT
    ESP_LOGI(TAG_PRINTER, "=== DATOS PREPARADOS PARA MQTT ===");
    ESP_LOGI(TAG_PRINTER, "Serial: %s", mqtt_data.register_number);
    ESP_LOGI(TAG_PRINTER, "STS1: 0x%02X, STS2: 0x%02X", sts1, sts2);
    ESP_LOGI(TAG_PRINTER, "Estado: %s", mqtt_data.estado_str);
    ESP_LOGI(TAG_PRINTER, "Error: %s", mqtt_data.error_str);
    
    // Tópicos que se usarán
    char topic[100];
    prepare_mqtt_topic(topic, sizeof(topic), "estado");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s", topic);
    
    // JSON de ejemplo
    char json_buffer[512];
    format_mqtt_json(json_buffer, sizeof(json_buffer), &mqtt_data);
    ESP_LOGI(TAG_PRINTER, "JSON: %s", json_buffer);
    ESP_LOGI(TAG_PRINTER, "==================================");
}

void interpretar_estado(uint8_t sts1, uint8_t sts2) 
{
    const char *estado_str = NULL;
    const char *error_str = NULL;

    //Procesar STS1 (Estado)
    switch(sts1) {
        case 0x40: 
            estado_str = "Modo Entrenamiento y en Espera";
            break;
        case 0x41:
            estado_str = "Modo Entrenamiento y en medio de una Transacción Fiscal";
            break;
        case 0x42:
            estado_str = "Modo Entrenamiento y en medio de una Transacción No Fiscal";
            break;
        case 0x60: 
            estado_str = "Modo Fiscal y en Espera";
            break;
        case 0x68:
            estado_str = "Modo Fiscal con la MF llena y en Espera";
            break;
        case 0x61:
            estado_str = "Modo Fiscal y en medio de una Transacción Fiscal";
            break;
        case 0x69:
            estado_str = "Modo Fiscal con la MF llena y en medio de una Transacción Fiscal";
            break;
        case 0x62:
            estado_str = "Modo Fiscal y en medio de una Transacción No Fiscal";
            break;
        case 0x6A:
            estado_str = "Modo Fiscal con la MF llena y en Transacción No Fiscal";
            break;
        default:
            estado_str = "Estado desconocido";
            break;
    }

    //Procesar STS2 (Error)
    switch(sts2) {
        case 0x40: 
            error_str = "Ningún error";
            break;
        case 0x48:
            error_str = "Error gaveta";
            break;
        case 0x41:
            error_str = "Error sin papel";
            break;
        case 0x42:
            error_str = "Error mecánico de la impresora / papel";
            break;
        case 0x43:
            error_str = "Error mecanico de la impresora y fin de papel";
            break;
        case 0x60:
            error_str = "Error fiscal";
            break;
        case 0x64:
            error_str = "Error en la memoria fiscal";
            break;
        case 0x6C:
            error_str = "Error memoria fiscal llena";
            break;
        default:
            error_str = "Error desconocido";
            break;
    }

    // Mostrar en log
    ESP_LOGI(TAG_PRINTER, "%s", estado_str);
    ESP_LOGI(TAG_PRINTER, "%s", error_str);
    
    // Preparar datos para MQTT
    prepare_mqtt_payload(sts1, sts2, estado_str, error_str);
}

void procesar_status_s1(uint8_t *data, size_t len)
{
    ESP_LOGI(TAG_PRINTER, "Procesando STATUS S1, len=%d", len);
    
    // Buscar el STX (0x02) en la respuesta
    int stx_pos = -1;
    for (int i = 0; i < len; i++) {
        if (data[i] == 0x02) {
            stx_pos = i;
            break;
        }
    }
    
    if (stx_pos == -1) {
        ESP_LOGE(TAG_PRINTER, "No se encontró STX (0x02) en la respuesta");
        return;
    }
    
    ESP_LOGI(TAG_PRINTER, "STX encontrado en posición: %d", stx_pos);
    
    // La trama comienza en stx_pos
    uint8_t *trama = &data[stx_pos];
    size_t trama_len = len - stx_pos;
    
    // Verificar que tenemos al menos STX + datos mínimos
    if (trama_len < 113) { // Longitud mínima según tabla
        ESP_LOGE(TAG_PRINTER, "Trama muy corta: %d (se esperaba al menos 113)", trama_len);
        return;
    }
    
    // Verificar ETX (debería estar al final)
    if (trama[trama_len-1] != 0x03) {
        ESP_LOGW(TAG_PRINTER, "ETX (0x03) no encontrado al final, encontrado: 0x%02X", trama[trama_len-1]);
    }
    
    // Extraer datos a estructura MQTT
    // Número de Cajero (posición 0-3 después de STX)
    memcpy(mqtt_data.atm_number, &trama[1], 4);
    mqtt_data.atm_number[4] = '\0';
    
    // Subtotal de Ventas (posición 4-20)
    memcpy(mqtt_data.ventas, &trama[5], 17);
    mqtt_data.ventas[17] = '\0';
    
    // Número de última factura (posición 21-28)
    memcpy(mqtt_data.last_bill_number, &trama[22], 8);
    mqtt_data.last_bill_number[8] = '\0';
    
    // Cantidad de facturas emitidas (posición 29-33)
    memcpy(mqtt_data.bill_issue, &trama[30], 5);
    mqtt_data.bill_issue[5] = '\0';
    
    // Número de la última nota de débito (posición 34-41)
    memcpy(mqtt_data.number_last_debit, &trama[35], 8);
    mqtt_data.number_last_debit[8] = '\0';

    // Cantidad de notas de débito del día (posición 42-46)
    memcpy(mqtt_data.amount_debit, &trama[43], 5);
    mqtt_data.amount_debit[5] = '\0';

    // Número de la última nota de crédito (posición 47-54)
    memcpy(mqtt_data.number_last_credit, &trama[48], 8);
    mqtt_data.number_last_credit[8] = '\0';

    // Cantidad de notas de crédito (posición 55-59)
    memcpy(mqtt_data.amount_credit, &trama[56], 5);
    mqtt_data.amount_credit[5] = '\0';

    // Número del último documento no fiscal (posición 60-67)
    memcpy(mqtt_data.number_last_notfiscal, &trama[61], 8);
    mqtt_data.number_last_notfiscal[8] = '\0';

    // Cantidad de documentos no fiscales (posición 68-72)
    memcpy(mqtt_data.amount_notfiscal, &trama[69], 5);
    mqtt_data.amount_notfiscal[5] = '\0';

    // Contador de cierres diarios (Z) (posición 73-76)
    memcpy(mqtt_data.counter_daily_z, &trama[74], 4);
    mqtt_data.counter_daily_z[4] = '\0';

    // Contador de reportes de memoria fiscal (posición 77-80)
    memcpy(mqtt_data.counter_report_fiscal, &trama[78], 4);
    mqtt_data.counter_report_fiscal[4] = '\0';
    
    // RIF (posición 81-91)
    memcpy(mqtt_data.rif_cliente, &trama[82], 11);
    mqtt_data.rif_cliente[11] = '\0';
    
    // Número de registro (posición 92-101)
    memcpy(mqtt_data.register_number, &trama[93], 10);
    mqtt_data.register_number[10] = '\0';

    // ACTUALIZAR SERIAL DINÁMICAMENTE
    update_printer_serial(mqtt_data.register_number);
    
    // Hora actual (posición 102-107)
    memcpy(mqtt_data.hour_machine, &trama[103], 6);
    mqtt_data.hour_machine[6] = '\0';
    
    // Fecha actual (posición 108-113)
    memcpy(mqtt_data.date_machine, &trama[109], 6);
    mqtt_data.date_machine[6] = '\0';
    
    // Actualizar timestamp
    mqtt_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    mqtt_data.needs_sync = true;

    // Mostrar datos para MQTT
    ESP_LOGI(TAG_PRINTER, "=== DATOS STATUS S1 PARA MQTT ===");
    
     // Tópicos individuales (ejemplo)
    char topic[100];
    
    prepare_mqtt_topic(topic, sizeof(topic), "cajero");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.atm_number);
    
    prepare_mqtt_topic(topic, sizeof(topic), "ventas");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.ventas);
    
    prepare_mqtt_topic(topic, sizeof(topic), "ultima_factura");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.last_bill_number);
    
    prepare_mqtt_topic(topic, sizeof(topic), "facturas_emitidas");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.bill_issue);
    
    prepare_mqtt_topic(topic, sizeof(topic), "rif");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.rif_cliente);
    
    prepare_mqtt_topic(topic, sizeof(topic), "registro");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.register_number);
    
    prepare_mqtt_topic(topic, sizeof(topic), "hora");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.hour_machine);
    
    prepare_mqtt_topic(topic, sizeof(topic), "fecha");
    ESP_LOGI(TAG_PRINTER, "Tópico: %s, Valor: %s", topic, mqtt_data.date_machine);
    
    ESP_LOGI(TAG_PRINTER, "==================================");
    
    // JSON completo de STATUS S1
    char json_buffer[1024];
    snprintf(json_buffer, sizeof(json_buffer),
        "{\"timestamp\":%lu,\"cajero\":\"%s\",\"ventas\":\"%s\","
        "\"ultima_factura\":\"%s\",\"facturas_emitidas\":\"%s\","
        "\"ult_nota_debito\":\"%s\",\"cant_notas_debito\":\"%s\","
        "\"ult_nota_credito\":\"%s\",\"cant_notas_credito\":\"%s\","
        "\"ult_doc_nofiscal\":\"%s\",\"cant_docs_nofiscal\":\"%s\","
        "\"cierres_diarios\":\"%s\",\"reportes_fiscal\":\"%s\","
        "\"rif\":\"%s\",\"registro\":\"%s\",\"hora\":\"%s\",\"fecha\":\"%s\"}",
        mqtt_data.timestamp,
        mqtt_data.atm_number,
        mqtt_data.ventas,
        mqtt_data.last_bill_number,
        mqtt_data.bill_issue,
        mqtt_data.number_last_debit,
        mqtt_data.amount_debit,
        mqtt_data.number_last_credit,
        mqtt_data.amount_credit,
        mqtt_data.number_last_notfiscal,
        mqtt_data.amount_notfiscal,
        mqtt_data.counter_daily_z,
        mqtt_data.counter_report_fiscal,
        mqtt_data.rif_cliente,
        mqtt_data.register_number,
        mqtt_data.hour_machine,
        mqtt_data.date_machine);
    
    ESP_LOGI(TAG_PRINTER, "JSON completo: %s", json_buffer);
}

bool validar_trama(uint8_t *trama, size_t len) {
    if (len < 3) return false; // Mínimo: STX + 2 bytes estado + ETX
    if (trama[0] != 0x02) return false; // STX
    if (trama[len-1] != 0x03) return false; // ETX
    return true;
}

static void mqtt_simulator_task(void *pvParameters) {
    ESP_LOGI(TAG_PRINTER, "Tarea simuladora MQTT iniciada");
    
    char last_serial[12] = "";
    uint32_t last_status_time = 0;
    
    while (1) {
        // Aquí en el futuro se conectaría a WiFi y MQTT
        // Por ahora solo mostramos logs
        
        // Verificar si hay datos para sincronizar
        if (mqtt_data.needs_sync) {
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Datos listos para enviar:");
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Serial: %s", mqtt_data.register_number);
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] STS1: 0x%02X, STS2: 0x%02X", 
                    mqtt_data.sts1, mqtt_data.sts2);
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Estado: %s", mqtt_data.estado_str);
            
            // Detectar cambio de serial
            if (strcmp(last_serial, mqtt_data.register_number) != 0) {
                strcpy(last_serial, mqtt_data.register_number);
                ESP_LOGI(TAG_PRINTER, "[MQTT SIM] ¡Nuevo serial detectado!: %s", last_serial);
                
                // Tópico especial para anunciar nueva impresora
                char discovery_topic[100];
                snprintf(discovery_topic, sizeof(discovery_topic), 
                        "%s/%s/discovery", MQTT_TOPIC_BASE, mqtt_data.register_number);
                ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Discovery: %s -> online", discovery_topic);
            }
            
            // Simular envío de tópicos individuales
            char topic[100];
            char payload[50];
            
            // Tópico: v1/fiscal/{serial}/sts1
            prepare_mqtt_topic(topic, sizeof(topic), "sts1");
            snprintf(payload, sizeof(payload), "0x%02X", mqtt_data.sts1);
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Publicar: %s -> %s", topic, payload);
            
            // Tópico: v1/fiscal/{serial}/sts2
            prepare_mqtt_topic(topic, sizeof(topic), "sts2");
            snprintf(payload, sizeof(payload), "0x%02X", mqtt_data.sts2);
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Publicar: %s -> %s", topic, payload);
            
            // Tópico: v1/fiscal/{serial}/estado
            prepare_mqtt_topic(topic, sizeof(topic), "estado");
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Publicar: %s -> %s", topic, mqtt_data.estado_str);
            
            // Tópico: v1/fiscal/{serial}/error
            prepare_mqtt_topic(topic, sizeof(topic), "error");
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Publicar: %s -> %s", topic, mqtt_data.error_str);
            
            // Tópico: v1/fiscal/{serial}/serial (redundante pero útil)
            prepare_mqtt_topic(topic, sizeof(topic), "serial");
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Publicar: %s -> %s", topic, mqtt_data.register_number);
            
            // Tópico: v1/fiscal/{serial}/timestamp
            prepare_mqtt_topic(topic, sizeof(topic), "timestamp");
            snprintf(payload, sizeof(payload), "%lu", mqtt_data.timestamp);
            ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Publicar: %s -> %s", topic, payload);
            
            // Estadísticas de intervalo
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (last_status_time > 0) {
                uint32_t interval = current_time - last_status_time;
                ESP_LOGI(TAG_PRINTER, "[MQTT SIM] Intervalo desde último estado: %lu ms", interval);
            }
            last_status_time = current_time;
            
            mqtt_data.needs_sync = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Verificar cada 5 segundos
    }
}

static void printer_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUFFER_SIZE);
    uint8_t enq_cmd = 0x05; // Carácter ENQ según el manual
    uint8_t status1 = 0x4C; // Carácter STATUS S1
    int reconnect_attempts = 0;

    mqtt_data_init(&mqtt_data);

    for (;;) {
        // Enviar el comando por UART
        uart_write_bytes(EX_UART_NUM, (const char*) &enq_cmd, 1);
        ESP_LOGI(TAG_PRINTER, "Enviado comando ENQ a la impresora.");

        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, pdMS_TO_TICKS(2000))) {
            bzero(dtmp, UART_BUFFER_SIZE);

            switch (event.type) {
                case UART_DATA:
                    int len = uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    uart_flush_input(EX_UART_NUM);

                    if (validar_trama(dtmp, len)) {
                        // Procesar respuesta
                        ESP_LOGI(TAG_PRINTER, "Respuesta válida recibida de la impresora.");
                        interpretar_estado(dtmp[1], dtmp[2]);

                        if (dtmp[1] == 0x60) {
                            // Si el estado es 0x60 (Modo Fiscal y en Espera), solicitar STATUS S1
                            vTaskDelay(pdMS_TO_TICKS(100));                             

                            // Impresora lista para recibir STATUS S1
                            uart_write_bytes(EX_UART_NUM, (const char*) &status1, 1);
                            ESP_LOGI(TAG_PRINTER, "Enviado carácter STATUS S1 (0x%02X) a la impresora.", status1);

                            vTaskDelay(pdMS_TO_TICKS(500)); // Dar tiempo para respuesta
                            
                            if (xQueueReceive(uart0_queue, (void *)&event, pdMS_TO_TICKS(2000))) {
                                // Leer la respuesta al STATUS S1
                                bzero(dtmp, UART_BUFFER_SIZE);

                                // Ejecutamos lectura de la respuesta al STATUS S1
                                int len_status = uart_read_bytes(EX_UART_NUM, dtmp, UART_BUFFER_SIZE, pdMS_TO_TICKS(5000));

                                // LIMPIAR BUFFER después de leer STATUS S1
                                uart_flush_input(EX_UART_NUM);
                                
                                if (validar_trama(dtmp, len_status)) {
                                    procesar_status_s1(dtmp, len_status);
                                }
                                else {
                                    ESP_LOGW(TAG_PRINTER, "Datos de STATUS S1 son invalidos.");
                                }
                            } else {
                                ESP_LOGW(TAG_PRINTER, "No se recibió evento tras enviar STATUS S1.");
                            }
                        }
                        reconnect_attempts = 0; // Resetear intentos de reconexión tras una respuesta exitosa
                    } else {
                        ESP_LOGE(TAG_PRINTER, "Trama inválida recibida");
                        // Limpiar buffer
                        uart_flush_input(EX_UART_NUM);
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
                
                // Preparar datos de error para MQTT
                prepare_mqtt_payload(0xFF, 0xFF, "Error comunicación", "Sin respuesta de impresora");
            }
        }

        // Esperar un tiempo antes de volver a comenzar
        // IMPORTANTE: Es vital usar vTaskDelay para no bloquear el procesador
        vTaskDelay(pdMS_TO_TICKS(10000)); // Pregunta cada 5 segundos

    }
    free(dtmp);
    vTaskDelete(NULL);
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
    uart_driver_install(EX_UART_NUM, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 20, &uart0_queue, 0);
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

    xTaskCreate(mqtt_simulator_task, "mqtt_simulator", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG_PRINTER, "Sistema iniciado. MQTT preparado con serial dinámico.");
    ESP_LOGI(TAG_PRINTER, "Base tópico: %s/{register_number}", MQTT_TOPIC_BASE);
    ESP_LOGI(TAG_PRINTER, "Serial se obtendrá dinámicamente de STATUS S1 (register_number)");

}