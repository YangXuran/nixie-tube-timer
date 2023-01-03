/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip_encoder.h"
#include <driver/gpio.h>
#include <string.h>

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED0_STRIP_GPIO_NUM 4
#define RMT_LED1_STRIP_GPIO_NUM 5

#define LED_NUMBERS 2
#define CHASE_SPEED_MS 100

static const char* TAG = "led-drv";

static struct {
    gpio_num_t gpio;
    rmt_channel_handle_t led_chan;
    rmt_encoder_handle_t led_encoder;
    uint8_t led_strip_pixels[LED_NUMBERS * 3];
} led_dev[] = {
    [0] = { .gpio = RMT_LED0_STRIP_GPIO_NUM, .led_chan = NULL, .led_encoder = NULL },
    [1] = { .gpio = RMT_LED1_STRIP_GPIO_NUM, .led_chan = NULL, .led_encoder = NULL }
};

static rmt_transmit_config_t tx_config = {
    .loop_count = 0, // no transfer loop
};

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t* r, uint32_t* g, uint32_t* b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

esp_err_t led_init(void)
{
    ESP_LOGI(TAG, "Init LED");
    for (int i = 0; i < 2; i++) {
        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .gpio_num = led_dev[i].gpio,
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_dev[i].led_chan));
        led_strip_encoder_config_t encoder_config = {
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
        };
        ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_dev[i].led_encoder));
        ESP_ERROR_CHECK(rmt_enable(led_dev[i].led_chan));
    }
    return ESP_OK;
}

void led_set_rgb(int ch, uint32_t red, uint32_t green, uint32_t blue)
{
    for (int i = 0; i < 2; i++) {
        led_dev[ch].led_strip_pixels[i * 3 + 0] = green;
        led_dev[ch].led_strip_pixels[i * 3 + 1] = red;
        led_dev[ch].led_strip_pixels[i * 3 + 2] = blue;
    }

    ESP_ERROR_CHECK(rmt_transmit(led_dev[ch].led_chan, led_dev[ch].led_encoder, led_dev[ch].led_strip_pixels,
        sizeof(led_dev[ch].led_strip_pixels), &tx_config));
}

void led_set_hsv(int ch, uint32_t h, uint32_t s, uint32_t v)
{
    uint32_t r = 0;
    uint32_t g = 0;
    uint32_t b = 0;
    led_strip_hsv2rgb(h, s, v, &r, &g, &b);
    led_set_rgb(ch, r, g, b);
}

void led_test(int ch)
{
    uint32_t hue = 0;
    while (1) {
        hue++;
        led_set_hsv(ch, hue, 100, 100);
        vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
    }
}
