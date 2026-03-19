#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "MOCK_PRINTER"

// Los mismos pines que configuramos en el Master
#define PIN_NUM_MISO 25
#define PIN_NUM_CLK  26
#define PIN_NUM_MOSI 27
#define PIN_NUM_CS   32

// Comandos que espera recibir del Master (Basado en tu printer_config.h)
#define SPI_CMD_START  0x01
#define SPI_CMD_PACKET 0x02
#define SPI_CMD_END    0x03
#define ACK_BYTE       0x06

// Buffers alineados en memoria (Requisito del DMA)
WORD_ALIGNED_ATTR uint8_t sendbuf[1050] = {0}; // Buffer para enviar
WORD_ALIGNED_ATTR uint8_t recvbuf[1050] = {0}; // Buffer para recibir

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Simulador de Impresora (Esclavo SPI)...");

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

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while (1) {
        // Limpiamos el buffer de recepción
        memset(recvbuf, 0, sizeof(recvbuf));
        
        // Preparamos nuestra respuesta: Siempre devolveremos un ACK (0x06) en el primer byte
        memset(sendbuf, 0, sizeof(sendbuf));
        sendbuf[0] = ACK_BYTE; 

        t.length = 1030 * 8; // Tamaño máximo que esperamos recibir en bits
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;

        // La tarea se bloquea aquí hasta que el Master baje el pin CS y envíe el reloj
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);

        if (ret == ESP_OK) {
            uint32_t bytes_recibidos = t.trans_len / 8; // trans_len viene en bits
            
            if (bytes_recibidos > 0) {
                uint8_t comando = recvbuf[0];

                if (comando == SPI_CMD_START) {
                    ESP_LOGI(TAG, "📥 Recibido: CMD_START (Iniciando OTA)");
                } 
                else if (comando == SPI_CMD_PACKET) {
                    // Si es un paquete, imprimimos el tamaño y los primeros bytes para verificar
                    ESP_LOGI(TAG, "📦 Recibido: PACKET (%lu bytes). Checksum simulado OK -> Enviando ACK", bytes_recibidos);
                } 
                else if (comando == SPI_CMD_END) {
                    ESP_LOGI(TAG, "🏁 Recibido: CMD_END (OTA Finalizado)");
                }
                else {
                    ESP_LOGW(TAG, "Recibida trama desconocida de %lu bytes. Primer byte: 0x%02X", bytes_recibidos, comando);
                }
            }
        }
    }
}
