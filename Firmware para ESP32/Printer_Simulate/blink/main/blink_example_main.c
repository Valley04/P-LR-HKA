#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "MOCK_PRINTER"

// Constantes Fiscales HKA
#define STX 0x02
#define ETX 0x03
#define ENQ 0x05
#define ACK 0x06

// Estados de la impresora
#define STS1 0x60
#define STS2 0x40

// Los mismos pines que configuramos en el Master
#define PIN_NUM_MISO 25
#define PIN_NUM_CLK  26
#define PIN_NUM_MOSI 27
#define PIN_NUM_CS   32

// Comandos que espera recibir del Master (Basado en tu printer_config.h)
#define SPI_CMD_START  0x01
#define SPI_CMD_PACKET 0x04
#define SPI_CMD_END    0x07

#define SPI_BUFFER_SIZE 1050

// Buffers alineados en memoria (Requisito del DMA)
uint8_t *sendbuf = NULL; // Buffer para enviar
uint8_t *recvbuf = NULL; // Buffer para recibir

typedef enum {
    PENDING_NONE,
    PENDING_S1,
    PENDING_SV2
} pending_res_t;

pending_res_t pending_response = PENDING_NONE;

// Función para calcular LRC
uint8_t calcular_lrc(uint8_t *data, int len) {
    uint8_t lrc = 0;
    for (int i = 0; i < len; i++) lrc ^= data[i];
    return lrc;
}

void app_main(void) {
    ESP_LOGI(TAG, "🖨️ Iniciando Simulador Fiscal HKA sobre SPI...");

    sendbuf = heap_caps_calloc(1, SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
    recvbuf = heap_caps_calloc(1, SPI_BUFFER_SIZE, MALLOC_CAP_DMA);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3,
        .flags = 0,
    };

    // Inicializamos el bus en modo SLAVE
    esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando SPI: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "✅ Escuchando en pines: MISO:%d, MOSI:%d, CLK:%d, CS:%d", 
             PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);

    // Preparamos nuestra respuesta: Siempre devolveremos un ACK (0x06) en el primer byte
    memset(sendbuf, 0, SPI_BUFFER_SIZE);

    while (1) {
        memset(recvbuf, 0, SPI_BUFFER_SIZE);

        spi_slave_transaction_t t;
        memset(&t, 0, sizeof(t));

        t.length = 1050 * 8; // Esperamos hasta 1050 bytes (en bits)
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;

        // La tarea se bloquea aquí hasta que el Master baje el pin CS y envíe el reloj
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);

        if (ret == ESP_OK) {

            if (t.trans_len == 0) continue;

            ESP_LOGI("MOCK_PRINTER", "⚡ Bus activado! Bits: %d | Primer byte bruto: 0x%02X", t.trans_len, recvbuf[0]);

            uint32_t bytes_recibidos = t.trans_len / 8; // trans_len viene en bits

            int start_idx = -1;

            for (int i = 0; i < bytes_recibidos; i++) {
                if ((recvbuf[i] != 0x00) && (recvbuf[i] != 0xFF)) {
                    start_idx = i;
                    break;
                }
            }

            // Si todo el buffer era basura (ceros o FF), limpiamos y seguimos esperando
            if (start_idx == -1) {
                memset(sendbuf, 0, SPI_BUFFER_SIZE);

                if (pending_response == PENDING_S1) {
                    ESP_LOGI("MOCK_PRINTER", "⏳ Master leyó el ACK. Cargando trama S1...");
                    char *payload = "S100\n00000264819849029\n00000000\n00000\n00000000\n00000\n00000000\n00000\n00000000\n00000\n0020\n0017\nJ-312171197\nZ1A8779988\n164000\n310326\n";
                    int payload_len = strlen(payload);
                    
                    sendbuf[0] = STX;
                    memcpy(&sendbuf[1], payload, payload_len);
                    sendbuf[1 + payload_len] = ETX;
                    sendbuf[2 + payload_len] = calcular_lrc(&sendbuf[1], payload_len + 1); 
                    pending_response = PENDING_NONE;
                }
                else if (pending_response == PENDING_SV2) {
                    ESP_LOGI("MOCK_PRINTER", "⏳ Master leyó el ACK. Cargando trama SV2...");
                    char *payload = "SV2\nZ7C\nVE\n020507GD00\n";
                    int payload_len = strlen(payload);
                    
                    sendbuf[0] = STX;
                    memcpy(&sendbuf[1], payload, payload_len);
                    sendbuf[1 + payload_len] = ETX;
                    sendbuf[2 + payload_len] = calcular_lrc(&sendbuf[1], payload_len + 1);
                    pending_response = PENDING_NONE;
                }
                continue;
            }

            uint8_t primer_byte = recvbuf[start_idx];
            ESP_LOGI("MOCK_PRINTER", "📥 Dato útil en índice [%d]: 0x%02X", start_idx, primer_byte);

            if (primer_byte == ENQ) {
                ESP_LOGI(TAG, "📥 Recibido: ENQ -> Preparando STATUS");
                memset(sendbuf, 0, SPI_BUFFER_SIZE);

                uint8_t cuerpo[] = {0x60, 0x40, ETX};
                uint8_t lrc = calcular_lrc(cuerpo, sizeof(cuerpo));
                
                sendbuf[0] = STX;
                sendbuf[1] = STS1;
                sendbuf[2] = STS2;
                sendbuf[3] = ETX;
                sendbuf[4] = lrc;
            } else if (primer_byte == STX) {
                // Extraemos hasta 3 bytes para poder identificar tanto "S1" como "SV2"
                char cmd[5] = {0};
                if (start_idx + 3 < bytes_recibidos) {
                    memcpy(cmd, &recvbuf[start_idx + 1], 3); 
                }
                
                if (strncmp(cmd, "S1", 2) == 0) {
                    ESP_LOGI(TAG, "📥 Recibido: Comando S1 -> Preparando trama de 133 bytes");
                    memset(sendbuf, 0, SPI_BUFFER_SIZE);
                    sendbuf[0] = ACK;
                    pending_response = PENDING_S1;
                }
                else if (strncmp(cmd, "SV2", 3) == 0) {
                    ESP_LOGI(TAG, "📥 Recibido: Comando SV2 -> Preparando versiones");
                    memset(sendbuf, 0, SPI_BUFFER_SIZE);
                    sendbuf[0] = ACK;
                    pending_response = PENDING_SV2;
                }
                else {
                    ESP_LOGW(TAG, "Comando fiscal no implementado: %s", cmd);
                }
            }
            else if (primer_byte == ACK) {
                ESP_LOGI(TAG, "📥 Recibido: ACK del Master");
                memset(sendbuf, 0, SPI_BUFFER_SIZE);
            }
            else if (primer_byte == SPI_CMD_START) {
                ESP_LOGI(TAG, "📥 Recibido: CMD_START (Iniciando OTA)");
                memset(sendbuf, 0, SPI_BUFFER_SIZE);
                sendbuf[0] = ACK; // Confirmamos inicio
            } 
            else if (primer_byte == SPI_CMD_PACKET) {
                ESP_LOGI(TAG, "📦 Recibido: PACKET (%lu bytes). Checksum OK -> Enviando ACK", bytes_recibidos);
                memset(sendbuf, 0, SPI_BUFFER_SIZE);
                sendbuf[0] = ACK;
            } 
            else if (primer_byte == SPI_CMD_END) {
                ESP_LOGI(TAG, "🏁 Recibido: CMD_END (OTA Finalizado)");
                memset(sendbuf, 0, SPI_BUFFER_SIZE);
                sendbuf[0] = ACK; // Confirmamos finalización
            }
            else {
                ESP_LOGW("MOCK_PRINTER", "❓ Dato desconocido: 0x%02X en índice [%d]", primer_byte, start_idx);
            }
        }
    }
}
