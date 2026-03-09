#include "printer_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define TAG_SPI "PRINTER_SPI"

// Prines típicos de SPI en el ESP32
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 22

spi_device_handle_t spi_printer;
static bool spi_inicializado = false;

uint8_t transaccionar_byte_spi(uint8_t data) {
    if (!spi_inicializado) return 0xFF;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = data; // Guardamos el byte directamente

    spi_device_polling_transmit(spi_printer, &t);
    
    return t.rx_data[0]; // Leemos el byte directamente
}

esp_err_t enviar_chunk_spi(uint8_t *data, int len) {
    if (!spi_inicializado || len == 0 || data == NULL) return ESP_OK;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = len * 8; // Longitud en BITS
    t.tx_buffer = data; 

    return spi_device_polling_transmit(spi_printer, &t);
}

void iniciar_spi_impresora() {
    if (spi_inicializado) {
        LOG_W(TAG_SPI, "El bus SPI ya estaba inicializado. Omitiendo.");
        return;
    }

    LOG_I(TAG_SPI, "Inicializando bus SPI...");

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096; // Suficiente para nuestros chunks de 1024 bytes

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = 1 * 1000 * 1000; // 1 MHz
    devcfg.mode = 0; // SPI Mode 0 (CPOL=0, CPHA=0)
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.queue_size = 7;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        LOG_E(TAG_SPI,"Error instalando el bus SPI: %s", esp_err_to_name(ret));
        return;
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_printer);
    if (ret == ESP_OK) {
        spi_inicializado = true;
        LOG_I(TAG_SPI, "✅ Impresora conectada al bus SPI exitosamente.");
    } else {
        LOG_E(TAG_SPI, "Error añadiendo dispositivo SPI: %s", esp_err_to_name(ret));
    }
}