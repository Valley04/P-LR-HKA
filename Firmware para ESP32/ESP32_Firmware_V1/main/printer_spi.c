#include "printer_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG_SPI "PRINTER_SPI"

// Prines típicos de SPI en el ESP32
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 22

spi_device_handle_t spi_printer;

void iniciar_spi_impresora() {
    LOG_I(TAG_SPI, "Inicializando bus SPI...");

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        LOG_E(TAG_SPI,"Error instalando el bus SPI");
        return;
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_printer);
    if (ret == ESP_OK) {
        LOG_I(TAG_SPI, "Impresora conectada al bus SPI exitosamente.");
    }
}