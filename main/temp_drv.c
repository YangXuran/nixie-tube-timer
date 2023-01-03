#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip_encoder.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <string.h>

static const char TAG[] = "temp-drv";

#define SPI_HOST SPI2_HOST

#define PIN_NUM_MISO 18
#define PIN_NUM_MOSI 26
#define PIN_NUM_CLK 19
#define PIN_NUM_CS 23

spi_device_handle_t spi;

esp_err_t spi_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // Clock out at 4 MHz, MAX6675 allow 4.3MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .pre_cb = NULL,
    };

    ESP_LOGI(TAG, "Init SPI");

    ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    return ESP_OK;
}

float read_temp(void)
{
    spi_transaction_t t;
    uint16_t tx = 0xFFFF, rx = 0;
    float temp;

    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = &tx;
    t.rx_buffer = &rx;

    spi_device_polling_transmit(spi, &t); // Transmit!
    rx = ((rx >> 8) & 0xFF) | ((rx << 8) & 0xFF00);
    if (((rx >> 2) & 1) == 1) {
        temp = 0.0;
        ESP_LOGW(TAG, "the thermocouple input is open");
    } else {
        temp = (rx >> 3 & 0xFFF) * 0.25;
    }

    return temp;
}
