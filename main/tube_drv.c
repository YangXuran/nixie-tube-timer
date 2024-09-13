#include "ch423.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <rom/ets_sys.h>
#include <stdio.h>
#include <string.h>

static const char* TAG = "tube-drv";

#define I2C0_MASTER_SCL_IO 22 /*!< GPIO number used for I2C master clock */
#define I2C0_MASTER_SDA_IO 21 /*!< GPIO number used for I2C master data  */
#define I2C0_MASTER_NUM 0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C1_MASTER_SCL_IO 9 /*!< GPIO number used for I2C master clock */
#define I2C1_MASTER_SDA_IO 10 /*!< GPIO number used for I2C master data  */
#define I2C1_MASTER_NUM 1 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_BUFF_SIZE 512

#define CH423_I2C_ADDR 0x68 /*!< Slave address of the CH423 */

static uint8_t tube_status = 0;

/**
 * @brief Write command to ch423
 */
static esp_err_t ch423_write_cmd(int32_t ch, uint16_t cmd)
{
    esp_err_t err = ESP_OK;
    uint8_t buffer[I2C_BUFF_SIZE] = { 0 };

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    assert(handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, ((uint8_t)(cmd >> 8) & CH423_I2C_MASK) | CH423_I2C_ADDR1, true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, (uint8_t)cmd, true);
    if (err != ESP_OK) {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(ch, handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

end:
    i2c_cmd_link_delete_static(handle);
    return err;
}

/**
 * @brief Write command to ch423
 */
static esp_err_t ch423_write_byte(int32_t ch, uint16_t cmd)
{
    esp_err_t err = ESP_OK;
    uint8_t buffer[I2C_BUFF_SIZE] = { 0 };

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    assert(handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, (uint8_t)(cmd >> 8), true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, (uint8_t)cmd, true);
    if (err != ESP_OK) {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(ch, handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

end:
    i2c_cmd_link_delete_static(handle);
    return err;
}

/**
 * @brief Read byte from ch423
 */
static esp_err_t ch423_read_byte(int32_t ch, uint8_t* data)
{
    esp_err_t err = ESP_OK;
    uint8_t buffer[I2C_BUFF_SIZE] = { 0 };

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    assert(handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, CH423_RD_IO_CMD, true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_read_byte(handle, data, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(ch, handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

end:
    i2c_cmd_link_delete_static(handle);
    return err;
}

void ch423_write_oc(int32_t ch, uint16_t value)
{
    ch423_write_byte(ch, CH423_OC_L_CMD | (uint8_t)value);
    ch423_write_byte(ch, CH423_OC_H_CMD | (uint8_t)(value >> 8));
}

void ch423_write_io(int32_t ch, uint8_t value)
{
    ch423_write_byte(ch, CH423_SET_IO_CMD | (uint8_t)value);
}

/**
 * @brief i2c master initialization
 */
esp_err_t tube_init(void)
{
    esp_err_t err = 0;

    i2c_config_t i2c0_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C0_MASTER_SDA_IO,
        .scl_io_num = I2C0_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_config_t i2c1_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C1_MASTER_SDA_IO,
        .scl_io_num = I2C1_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C0_MASTER_NUM, &i2c0_conf);
    i2c_param_config(I2C1_MASTER_NUM, &i2c1_conf);

    err = i2c_driver_install(I2C0_MASTER_NUM, i2c0_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_driver_install(I2C1_MASTER_NUM, i2c1_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        goto end;
    }

    for (int i = 0; i < 2; i++) {
        err = ch423_write_cmd(i, CH423_SYS_CMD | BIT_IO_OE);
        if (err != ESP_OK) {
            ESP_LOGI(TAG, "nixie tube %d not find", i + 1);
        } else {
            tube_status |= 1 << i;
            ch423_write_oc(i, 0);
            ch423_write_io(i, 0);
        }
    }

end:
    return ESP_OK;
}

uint8_t get_tube_init_status(int32_t ch)
{
    return ((tube_status >> ch) & 1);
}

struct ch423_pinout {
    uint16_t oc;
    uint8_t io;
};

const struct ch423_pinout tube_map[2][12] = {
    [0][0] = { .oc = 1 << 6, .io = 0 },
    [0][1] = { .oc = 0, .io = 1 << 5 },
    [0][2] = { .oc = 0, .io = 1 << 6 },
    [0][3] = { .oc = 0, .io = 1 << 7 },
    [0][4] = { .oc = 1 << 0, .io = 0 },
    [0][5] = { .oc = 1 << 1, .io = 0 },
    [0][6] = { .oc = 1 << 2, .io = 0 },
    [0][7] = { .oc = 1 << 3, .io = 0 },
    [0][8] = { .oc = 1 << 4, .io = 0 },
    [0][9] = { .oc = 1 << 5, .io = 0 },
    [0][10] = { .oc = 0, .io = 1 << 4 }, /*dot_L*/
    [0][11] = { .oc = 1 << 7, .io = 0 }, /*dot_R*/
    [1][0] = { .oc = 1 << 9, .io = 0 },
    [1][1] = { .oc = 0, .io = 1 << 2 },
    [1][2] = { .oc = 0, .io = 1 << 1 },
    [1][3] = { .oc = 0, .io = 1 << 0 },
    [1][4] = { .oc = 1 << 15, .io = 0 },
    [1][5] = { .oc = 1 << 14, .io = 0 },
    [1][6] = { .oc = 1 << 13, .io = 0 },
    [1][7] = { .oc = 1 << 12, .io = 0 },
    [1][8] = { .oc = 1 << 11, .io = 0 },
    [1][9] = { .oc = 1 << 10, .io = 0 },
    [1][10] = { .oc = 0, .io = 1 << 3 }, /*dot_L*/
    [1][11] = { .oc = 1 << 8, .io = 0 }, /*dot_R*/
};

static struct ch423_pinout tube_buf[2] = { 0 };

void tube_clear(int32_t ch)
{
    memset(&tube_buf[ch], 0, sizeof(struct ch423_pinout));
}

void tube_set_num(int32_t ch, uint8_t tube, uint8_t num)
{
    tube_buf[ch].oc |= tube_map[tube][num].oc;
    tube_buf[ch].io |= tube_map[tube][num].io;
}

void tube_set_dot(int32_t ch, uint8_t tube, uint8_t left_right)
{
    tube_buf[ch].oc |= tube_map[tube][10 + left_right].oc;
    tube_buf[ch].io |= tube_map[tube][10 + left_right].io;
}

void tube_show(int32_t ch)
{
    ch423_write_oc(ch, tube_buf[ch].oc);
    ch423_write_io(ch, tube_buf[ch].io);
}

#define TRANSFORM_TIME 1200
#define TRANSFORM_STEP 40

void tube_show_num_with_trans(int32_t ch, uint8_t num)
{
    int32_t i = TRANSFORM_TIME, j;
    int32_t t1, t2 = 0;
    uint8_t a, b;
    static uint8_t la, lb;

    if (num > 99)
        num = 99;
    a = num / 10;
    b = num % 10;

    t1 = TRANSFORM_TIME;

    while (t1) {
        tube_clear(ch);
        if (la == a) {
            tube_set_num(ch, 0, a);
        } else {
            tube_set_num(ch, 0, la);
        }
        if (lb == b) {
            tube_set_num(ch, 1, b);
        } else {
            tube_set_num(ch, 1, lb);
        }
        tube_show(ch);
        ets_delay_us(t1);

        tube_clear(ch);
        if (la == a) {
            tube_set_num(ch, 0, a);
        } else {
            tube_set_num(ch, 0, a);
        }
        if (lb == b) {
            tube_set_num(ch, 1, b);
        } else {
            tube_set_num(ch, 1, b);
        }
        tube_show(ch);
        ets_delay_us(TRANSFORM_TIME + 1 - t1);
        t1 -= TRANSFORM_STEP;
    }

    la = a;
    lb = b;
}

void tube_show_num(int32_t ch, uint8_t num)
{
    uint8_t a, b;

    if (num > 99)
        num = 99;
    a = num / 10;
    b = num % 10;

    tube_clear(ch);
    tube_set_num(ch, 0, a);
    tube_set_num(ch, 1, b);
    tube_show(ch);
}

void tube_test_num(void)
{
    uint8_t num = 0;

    for (int ch = 0; ch < 2; ch++) {
        if (get_tube_init_status(ch)) {
            tube_show_num_with_trans(ch, num);
        }
    }
    for (int i = 0; i < 20; i++) {
        num += 11;
        for (int ch = 0; ch < 2; ch++) {
            if (get_tube_init_status(ch)) {
                tube_show_num_with_trans(ch, num);
            }
        }
        if (num == 99)
            num = 0;
    }
    for (int ch = 0; ch < 2; ch++) {
        if (get_tube_init_status(ch)) {
            tube_show_num_with_trans(ch, 0);
        }
    }
}

void tube_shutdown(int32_t ch)
{
    tube_clear(ch);
    tube_show(ch);
}
