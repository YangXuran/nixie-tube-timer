#include "led_drv.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "temp_drv.h"
#include "tube_drv.h"
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdio.h>

static const char* TAG = "app_main";

#define BTN_INPUT_GPIO 36
#define SW0_INPUT_GPIO 37
#define SW1_INPUT_GPIO 38

#define TIMER_GRANULARITY_MS 10
#define TIMEOUT_TO_TEMP_S 30
#define TIMEOUT_TO_SLEEP_S 5 * 60

#define LED_DEFAULT_HUE 270
#define LED_BRIGHTNESS 100

#define SHORT_PRESS_TIME (50 / 10)
#define LONG_PRESS_TIME (3000 / 10)

enum DEV_STATUS {
    DEV_DEFAULT = 0,
    DEV_SLEEP,
    DEV_TIMINHG,
    DEV_TEMP
};

static struct {
    uint8_t led_on;
    uint32_t timeout;
} dev_setting = {
    .led_on = 1,
    .timeout = 0
};

static struct {
    gpio_num_t gpio;
    uint32_t cnt;
    uint32_t time;
    uint32_t timeout;
    uint32_t status;
    uint32_t brightness;
    SemaphoreHandle_t sem;
} timer_dev[] = {
    [0] = { .gpio = SW0_INPUT_GPIO,
        .cnt = 0,
        .time = 0,
        .timeout = 0,
        .status = DEV_DEFAULT,
        .brightness = LED_BRIGHTNESS,
        .sem = NULL },
    [1] = { .gpio = SW1_INPUT_GPIO,
        .cnt = 0,
        .time = 0,
        .timeout = 0,
        .status = DEV_DEFAULT,
        .brightness = LED_BRIGHTNESS,
        .sem = NULL },
};

static float temp;
static nvs_handle_t nvs_fd;
static SemaphoreHandle_t btn_event_sem = NULL;

static void btn_irq_handler(void* arg)
{
    xSemaphoreGiveFromISR(btn_event_sem, NULL);
}

static int get_press_type(void)
{
    int count = 0;

    while (gpio_get_level(BTN_INPUT_GPIO) == 0) {
        count++;
        if (count > LONG_PRESS_TIME) {
            return 2;
        }
        vTaskDelay(pdMS_TO_TICKS(10UL));
    }

    if (count > SHORT_PRESS_TIME) {
        return 1;
    } else {
        return -1;
    }
}

static void btn_push_task(void* arg)
{
    gpio_config_t io_conf = {
        // disable interrupt
        .intr_type = GPIO_INTR_NEGEDGE,
        // set as output mode
        .mode = GPIO_MODE_INPUT,
        // bit mask of the pins that you want to set,e.g.GPIO18/19
        .pin_bit_mask = (1ULL << BTN_INPUT_GPIO),
        // disable pull-down mode
        .pull_down_en = 0,
        // disable pull-up mode
        .pull_up_en = 1,
    };

    btn_event_sem = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INPUT_GPIO, btn_irq_handler, BTN_INPUT_GPIO);
    while (1) {
        if (xSemaphoreTake(btn_event_sem, portMAX_DELAY)) {
            switch (get_press_type()) {
            case 2:
                ESP_LOGI(TAG, "btn long press");
                dev_setting.led_on = dev_setting.led_on ? 0 : 1;
                ESP_LOGI(TAG, "led status change to %s", dev_setting.led_on ? "on" : "off");
                nvs_set_u8(nvs_fd, "led_on", dev_setting.led_on);
            case 1:
                ESP_LOGI(TAG, "btn short press");
                timer_dev[0].status = DEV_TEMP;
                timer_dev[0].timeout = TIMEOUT_TO_TEMP_S;
                break;
            case 0:
                ESP_LOGI(TAG, "btn double click");
                break;
            case -1:
            default:
                break;
            }
        }
    }
}

static bool timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    int i;
    static uint32_t tick = 0;
    BaseType_t high_task_awoken = pdTRUE;

    for (i = 0; i < 2; i++) {
        if ((tick++ % (1000 / TIMER_GRANULARITY_MS)) == 0) {
            timer_dev[i].timeout++;
        }

        if (timer_dev[i].status == DEV_TIMINHG) {
            timer_dev[i].cnt++;
            if ((timer_dev[i].cnt % (1000 / TIMER_GRANULARITY_MS)) == 0) {
                timer_dev[i].time++;
                xSemaphoreGiveFromISR(timer_dev[i].sem, &high_task_awoken);
            }
        }
    }
    return pdTRUE;
}

static void timer_task(void* arg)
{
    gpio_config_t io_conf = {
        // disable interrupt
        .intr_type = GPIO_INTR_DISABLE,
        // set as output mode
        .mode = GPIO_MODE_INPUT,
        // bit mask of the pins that you want to set,e.g.GPIO18/19
        .pin_bit_mask = (1ULL << SW0_INPUT_GPIO) | (1ULL << SW1_INPUT_GPIO),
        // disable pull-down mode
        .pull_down_en = 0,
        // disable pull-up mode
        .pull_up_en = 1,
    };

    for (int i = 0; i < 2; i++) {
        timer_dev[i].sem = xSemaphoreCreateBinary();
    }
    gpio_config(&io_conf);

    tube_init();
    tube_test_num();

    while (1) {
        for (int i = 0; i < 2; i++) {
            if (gpio_get_level(timer_dev[i].gpio) == 0) {
                if (get_tube_init_status(1) == 0 && timer_dev[i ? 0 : 1].status == DEV_TIMINHG) {
                    ESP_LOGI(TAG, "sw%d trigger, but sw%d is timing", i, i ? 0 : 1);
                    continue;
                } else if (timer_dev[i].status != DEV_TIMINHG) {
                    timer_dev[i].status = DEV_TIMINHG;
                    timer_dev[i].cnt = 0;
                    timer_dev[i].time = 0;
                    ESP_LOGI(TAG, "sw%d start timing", i);
                }

                if (get_tube_init_status(1) == 0) {
                    if (timer_dev[i ? 0 : 1].status == DEV_TIMINHG)
                        continue;
                    else
                        tube_show_num_with_trans(0, timer_dev[i].time);
                } else {
                    tube_show_num_with_trans(i, timer_dev[i].time);
                }

                timer_dev[i].timeout = 0;
                if (i == 1 && (get_tube_init_status(i) == 0)) {
                    timer_dev[0].timeout = 0;
                }
            } else if (timer_dev[i].timeout >= TIMEOUT_TO_SLEEP_S) {
                timer_dev[i].status = DEV_SLEEP;
                if (i == 1 && (get_tube_init_status(i) == 0))
                    continue;
                else
                    tube_shutdown(i);
            } else if (timer_dev[i].timeout >= TIMEOUT_TO_TEMP_S) {
                timer_dev[i].status = DEV_TEMP;
                if (i == 1 && (get_tube_init_status(i) == 0))
                    continue;
                else
                    tube_show_num_with_trans(i, (uint8_t)temp);
            } else {
                timer_dev[i].status = DEV_DEFAULT;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10UL));
    }
}

static void led_task(void* arg)
{
    int ch;
    static uint32_t last_status[2] = { DEV_DEFAULT, DEV_DEFAULT };
    static uint32_t last_hue[2] = { 0, 0 };

    led_init();
    if (dev_setting.led_on) {
        for (int i = 0; i < LED_BRIGHTNESS; i++) {
            led_set_hsv(0, LED_DEFAULT_HUE, 100, i);
            led_set_hsv(1, LED_DEFAULT_HUE, 100, i);
            vTaskDelay(pdMS_TO_TICKS(30UL));
        }
    }
    while (1) {
        for (int i = 0; i < 2; i++) {
            if (!dev_setting.led_on) {
                led_set_hsv(i, 0, 0, 0);
                continue;
            }
            if (i == 1 && (get_tube_init_status(i) == 0))
                ch = 0;
            else
                ch = i;
            switch (timer_dev[i].status) {
            case DEV_SLEEP:
                last_status[i] = DEV_SLEEP;
                led_set_hsv(ch, 0, 0, 0);
                break;
            case DEV_TIMINHG:
                if (last_status[i] != DEV_TIMINHG) {
                    last_status[i] = DEV_TIMINHG;
                    last_hue[ch] = 150; /* green */
                }
                led_set_hsv(ch, last_hue[0], 100, timer_dev[i].brightness); /* 30s green to red */
                if (last_hue[ch] != 0)
                    last_hue[ch]--;
                break;
            case DEV_TEMP:
                last_status[i] = DEV_TEMP;
                if (i == 1 && (get_tube_init_status(i) == 0))
                    continue;
                else
                    led_set_hsv(i, LED_DEFAULT_HUE, 100, timer_dev[i].brightness); /* 30s green to red */
                break;
            default:
                last_status[i] = DEV_DEFAULT;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void read_temp_task(void* arg)
{
    spi_init();
    while (1) {
        temp = read_temp();
        vTaskDelay(pdMS_TO_TICKS(1000UL));
    }
}

void app_main(void)
{
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_cb, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    gptimer_alarm_config_t alarm_config1 = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = 1000 * TIMER_GRANULARITY_MS, // period = TIMER_GRANULARITY_MS
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_fd));
    nvs_get_u8(nvs_fd, "led_on", &dev_setting.led_on);
    ESP_LOGI(TAG, "get led setting:%s\n", dev_setting.led_on ? "on" : "off");

    /* Run btn press check task */
    xTaskCreate(btn_push_task, "btn_task", 2048, NULL, 1, NULL);

    /* Run read temperature */
    xTaskCreate(read_temp_task, "read_temp_task", 2048, NULL, 7, NULL);

    /* Run timer */
    xTaskCreate(timer_task, "timer_task", 2048, NULL, 9, NULL);

    /* Run LED */
    xTaskCreate(led_task, "led_task", 4096, NULL, 8, NULL);
    ESP_LOGI(TAG, "system initialized successfully");
}
