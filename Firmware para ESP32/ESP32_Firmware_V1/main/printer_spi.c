#include "printer_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_http_client.h"
#include "esp_log.h"
#define TAG_SPI "PRINTER_SPI"

// Prines típicos de SPI en el ESP32
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 22

spi_device_handle_t spi_printer;

extern esp_err_t enviar_chunk_spi(uint8_t *data, int len);

void actualizar_impresora_por_spi(const char *url_descarga) {
    LOG_I(TAG_SPI, "Iniciando descarga para la impresora desde: %s", url_descarga);

    esp_http_client_config_t config = {
        .url = url_descarga,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (esp_http_client_open(client, 0) != ESP_OK) {
        LOG_E(TAG_SPI, "Error abriendo conexion HTTP con el servidor");
        esp_http_client_cleanup(client);
        return;
    }

    int content_length = esp_http_client_fetch_headers(client);
    LOG_I(TAG_SPI, "Tamaño del firmware de la impresora: %d bytes", content_length);

    uint8_t *buffer = malloc(1024);
    if (buffer == NULL) {
        LOG_E(TAG_SPI, "No hay memoria RAM suficiente para el buffer");
        esp_http_client_cleanup(client);
        return;
    }

    int bytes_leidos;
    int total_leidos = 0;

    while (1) {
        bytes_leidos = esp_http_client_read(client, (char*)buffer, 1024);

        if (bytes_leidos < 0) {
            LOG_E(TAG_SPI, "Error de red durante la descarga");
            break;
        } else if (bytes_leidos == 0) {
            LOG_I(TAG_SPI, "Descarga y transmision completada al 100%%");
            break;
        }

        if (enviar_chunk_spi(buffer, bytes_leidos) != ESP_OK) {
            LOG_E(TAG_SPI, "Error trasmitido chunk por SPI");
            break;
        }

        total_leidos += bytes_leidos;
        LOG_I(TAG_SPI, "Progreso: %d / %d bytes enviados...", total_leidos, content_length);
    }

    free(buffer);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

esp_err_t enviar_chunk_spi(uint8_t *data, int len) {
    if (len == 0) return ESP_OK;

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .rx_buffer = NULL
    };

    return spi_device_polling_transmit(spi_printer, &t);
}

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