// printer_config.h
#ifndef PRINTER_CONFIG_H
#define PRINTER_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// CONSTANTES DE CONFIGURACIÓN

// UART
#define UART_PORT           UART_NUM_1
#define UART_BAUDRATE       9600
#define UART_TX_PIN         18
#define UART_RX_PIN         19
#define UART_BUFFER_SIZE    2048      
#define UART_QUEUE_SIZE     40      

// Protocolo Fiscal
#define ENQ_CMD             0x05
#define STX_BYTE            0x02
#define ETX_BYTE            0x03
#define ACK_BYTE            0x06
#define NAK_BYTE            0x15

extern const uint8_t CMD_STATUS_S1[];

// Tiempos (ms) - Optimizados
#define POLLING_INTERVAL_MS     30000   // 30 segundos
#define RESPONSE_TIMEOUT_MS     2000    // 2 segundos
#define STATUS_S1_TIMEOUT_MS    3000    // 3 segundos
#define ACK_TIMEOUT_MS          1000
#define RECONNECT_DELAY_MS      5000    // 5 segundos
#define COMMAND_DELAY_MS        100     // 100ms entre comandos

// Límites
#define MAX_RECONNECT_ATTEMPTS  5
#define MAX_ERROR_COUNT         10
#define MAX_SERIAL_LENGTH       10
#define STATUS_S1_MIN_LENGTH    113

// Configuración WiFi
#define ESP_WIFI_SSID      "LorenaWiFi"
#define ESP_WIFI_PASS      "041295wifi"
#define ESP_MAXIMUM_RETRY  5

// Configuración MQTT
#define MQTT_TOPIC_BASE     "v1/fiscal"
#define MQTT_QOS_LEVEL     1
#define MQTT_RETAIN        0

// TIPOS DE DATOS OPTIMIZADOS

// Estados de la comunicación
typedef enum {
    PRINTER_STATE_IDLE,
    PRINTER_STATE_WAITING_RESPONSE,
    PRINTER_STATE_PROCESSING,
    PRINTER_STATE_ERROR,
    PRINTER_STATE_RECONNECTING,
    PRINTER_STATE_SENDING_STATUS_S1,
    PRINTER_STATE_WAITING_STATUS_S1
} printer_state_t;

// Códigos de error
typedef enum {
    ERROR_NONE = 0,
    ERROR_NO_RESPONSE,
    ERROR_INVALID_RESPONSE,
    ERROR_TIMEOUT,
    ERROR_FRAME,
    ERROR_PARITY,
    ERROR_COMMUNICATION_LOST,
    ERROR_MAX_RETRIES_EXCEEDED,
    ERROR_STATUS_S1_FAILED,
    ERROR_NO_ACK
} printer_error_t;

// Estructura compacta para datos MQTT
typedef struct __attribute__((packed)) {
    // Datos básicos (6 bytes)
    uint8_t sts1;
    uint8_t sts2;
    uint16_t error_count;

    // Flags de control
    uint8_t reconnect_attempts : 4;
    uint8_t needs_sync : 1;
    uint8_t serial_obtained : 1;
    uint8_t reserved : 2;
    
    // Serial y datos numéricos del S1
    
    uint32_t timestamp;
    
    // Datos STATUS S1
    char atm_number[5];
    uint32_t ventas;
    char last_bill_number[9];
    char bill_issue[6];
    char number_last_debit[9];
    uint32_t amount_debit;
    char number_last_credit[9];
    uint32_t amount_credit;
    char number_last_notfiscal[9];
    uint32_t amount_notfiscal;
    char counter_daily_z[5];
    char counter_report_fiscal[5];
    char rif_cliente[12];
    char register_number[11];
    char hour_machine[7];
    char date_machine[7];
    
} mqtt_data_t;

// DECLARACIONES EXTERNAS

// Variables globales (definidas en printer_task.c)
extern mqtt_data_t mqtt_data;
extern const char* TAG_PRINTER;

// Variables globales WiFi/MQTT
extern EventGroupHandle_t wifi_event_group;
extern esp_mqtt_client_handle_t mqtt_client;
extern SemaphoreHandle_t mqtt_data_mutex;

// Colas (definidas en printer_task.c)
extern QueueHandle_t printer_uart_queue;

// MACROS UTILES

// Para strings en Flash (ahorra RAM)
#define STRING_IN_FLASH(str) (__extension__({ \
    static const char __c[] __attribute__((section(".rodata"))) = str; \
    __c; \
}))

// Logging optimizado
#define LOG_I(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define LOG_E(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define LOG_W(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define LOG_D(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)

// Verificación de errores
#define CHECK_ERROR(condition, error_code) \
    do { \
        if (!(condition)) { \
            return error_code; \
        } \
    } while(0)

// PROTOTIPOS DE FUNCIONES BÁSICAS

// Inicialización
bool printer_uart_init(void);
void printer_uart_deinit(void);

// Control de estado
printer_state_t printer_get_state(void);
void printer_set_state(printer_state_t state);
const char* printer_state_to_string(printer_state_t state);

// Validación
bool validate_frame(const uint8_t* data, size_t len);
bool is_valid_response(const uint8_t* data, size_t len);

// Utilidades
uint32_t get_timestamp_ms(void);
void delay_ms(uint32_t ms);

#endif // PRINTER_CONFIG_H