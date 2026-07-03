#include "epd_ssd1680.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"

static const char *TAG = "epd";

/* SSD1680 commands */
#define CMD_DRIVER_OUTPUT      0x01
#define CMD_DEEP_SLEEP         0x10
#define CMD_DATA_ENTRY_MODE    0x11
#define CMD_SW_RESET           0x12
#define CMD_TEMP_SENSOR        0x18
#define CMD_MASTER_ACTIVATION  0x20
#define CMD_DISPLAY_UPDATE_1   0x21
#define CMD_DISPLAY_UPDATE_2   0x22
#define CMD_WRITE_RAM_BW       0x24
#define CMD_WRITE_RAM_RED      0x26 /* holds the "previous image" on B/W panels */
#define CMD_BORDER_WAVEFORM    0x3C
#define CMD_RAM_X_RANGE        0x44
#define CMD_RAM_Y_RANGE        0x45
#define CMD_RAM_X_COUNTER      0x4E
#define CMD_RAM_Y_COUNTER      0x4F

#define UPDATE_MODE_FULL    0xF7 /* display mode 1, load temp + LUT from OTP */
#define UPDATE_MODE_PARTIAL 0xFF /* display mode 2 (differential) */

#define BUSY_TIMEOUT_MS 10000

static spi_device_handle_t s_spi;

static esp_err_t spi_send(const uint8_t *data, size_t len, bool is_data)
{
    gpio_set_level(PIN_EPD_DC, is_data);

    while (len > 0) {
        size_t chunk = len > 4000 ? 4000 : len;
        spi_transaction_t t = {
            .length = chunk * 8,
            .tx_buffer = data,
        };
        esp_err_t err = spi_device_polling_transmit(s_spi, &t);
        if (err != ESP_OK) {
            return err;
        }
        data += chunk;
        len -= chunk;
    }
    return ESP_OK;
}

static esp_err_t write_cmd(uint8_t cmd)
{
    return spi_send(&cmd, 1, false);
}

static esp_err_t write_data(const uint8_t *data, size_t len)
{
    return spi_send(data, len, true);
}

static esp_err_t write_cmd_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    esp_err_t err = write_cmd(cmd);
    if (err == ESP_OK && len > 0) {
        err = write_data(data, len);
    }
    return err;
}

static esp_err_t wait_busy(void)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(PIN_EPD_BUSY)) {
        if (esp_timer_get_time() - start > (int64_t)BUSY_TIMEOUT_MS * 1000) {
            ESP_LOGE(TAG, "busy timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_OK;
}

static esp_err_t hw_reset(void)
{
    gpio_set_level(PIN_EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return wait_busy();
}

/* Common register setup shared by full and partial init: window covers the
 * whole panel, X increment / Y increment addressing from (0,0). */
static esp_err_t setup_window(void)
{
    esp_err_t err = ESP_OK;

    err |= write_cmd_data(CMD_DRIVER_OUTPUT,
                          (const uint8_t[]){ 0x27, 0x01, 0x00 }, 3);
    err |= write_cmd_data(CMD_DATA_ENTRY_MODE, (const uint8_t[]){ 0x03 }, 1);
    err |= write_cmd_data(CMD_RAM_X_RANGE,
                          (const uint8_t[]){ 0x00, EPD_PANEL_W / 8 - 1 }, 2);
    err |= write_cmd_data(CMD_RAM_Y_RANGE,
                          (const uint8_t[]){ 0x00, 0x00,
                                             (EPD_PANEL_H - 1) & 0xFF,
                                             (EPD_PANEL_H - 1) >> 8 }, 4);
    err |= write_cmd_data(CMD_RAM_X_COUNTER, (const uint8_t[]){ 0x00 }, 1);
    err |= write_cmd_data(CMD_RAM_Y_COUNTER, (const uint8_t[]){ 0x00, 0x00 }, 2);
    /* 0x80: source outputs start at S8 — the 128 panel sources sit at
     * S8..S135 of the SSD1680 (per the WeAct reference driver). Without
     * this the image is shifted by 8 pixels and the last rows stay blank. */
    err |= write_cmd_data(CMD_DISPLAY_UPDATE_1,
                          (const uint8_t[]){ 0x00, 0x80 }, 2);
    err |= write_cmd_data(CMD_TEMP_SENSOR, (const uint8_t[]){ 0x80 }, 1);

    return err == ESP_OK ? ESP_OK : ESP_FAIL;
}

static esp_err_t update_and_sleep(uint8_t mode)
{
    esp_err_t err = ESP_OK;

    err |= write_cmd_data(CMD_DISPLAY_UPDATE_2, &mode, 1);
    err |= write_cmd(CMD_MASTER_ACTIVATION);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    err = wait_busy();
    if (err != ESP_OK) {
        return err;
    }

    /* Deep sleep mode 1: RAM is retained, wake-up via hardware reset. */
    err = write_cmd_data(CMD_DEEP_SLEEP, (const uint8_t[]){ 0x01 }, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return err;
}

esp_err_t epd_init(void)
{
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_DC) | (1ULL << PIN_EPD_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    gpio_config_t busy_cfg = {
        .pin_bit_mask = 1ULL << PIN_EPD_BUSY,
        .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&busy_cfg));

    gpio_set_level(PIN_EPD_RST, 1);
    gpio_set_level(PIN_EPD_DC, 1);

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = PIN_SPI_SCK,
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_EPD_CS,
        .queue_size = 2,
    };
    return spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi);
}

esp_err_t epd_full_refresh(const uint8_t *fb)
{
    esp_err_t err = hw_reset();
    if (err != ESP_OK) {
        return err;
    }

    err = write_cmd(CMD_SW_RESET);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    err = wait_busy();
    if (err != ESP_OK) {
        return err;
    }

    err = setup_window();
    if (err != ESP_OK) {
        return err;
    }
    err = write_cmd_data(CMD_BORDER_WAVEFORM, (const uint8_t[]){ 0x05 }, 1);
    if (err != ESP_OK) {
        return err;
    }

    /* Write the image to both RAMs so the first partial diffs correctly. */
    err = write_cmd_data(CMD_WRITE_RAM_BW, fb, EPD_FRAMEBUFFER_SIZE);
    if (err != ESP_OK) {
        return err;
    }
    err = write_cmd_data(CMD_RAM_X_COUNTER, (const uint8_t[]){ 0x00 }, 1);
    err |= write_cmd_data(CMD_RAM_Y_COUNTER, (const uint8_t[]){ 0x00, 0x00 }, 2);
    err |= write_cmd_data(CMD_WRITE_RAM_RED, fb, EPD_FRAMEBUFFER_SIZE);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    return update_and_sleep(UPDATE_MODE_FULL);
}

esp_err_t epd_partial_refresh(const uint8_t *fb)
{
    /* Wake from deep sleep; RAM (previous image) is retained.
     * No SW reset here, per the vendor's partial update flow. */
    esp_err_t err = hw_reset();
    if (err != ESP_OK) {
        return err;
    }

    err = setup_window();
    if (err != ESP_OK) {
        return err;
    }
    /* Keep the border floating during partials to avoid border greying. */
    err = write_cmd_data(CMD_BORDER_WAVEFORM, (const uint8_t[]){ 0x80 }, 1);
    if (err != ESP_OK) {
        return err;
    }

    err = write_cmd_data(CMD_WRITE_RAM_BW, fb, EPD_FRAMEBUFFER_SIZE);
    if (err != ESP_OK) {
        return err;
    }

    err = write_cmd_data(CMD_DISPLAY_UPDATE_2,
                         (const uint8_t[]){ UPDATE_MODE_PARTIAL }, 1);
    err |= write_cmd(CMD_MASTER_ACTIVATION);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    err = wait_busy();
    if (err != ESP_OK) {
        return err;
    }

    /* Re-sync the "previous image" RAM with what is now on the glass. */
    err = write_cmd_data(CMD_RAM_X_COUNTER, (const uint8_t[]){ 0x00 }, 1);
    err |= write_cmd_data(CMD_RAM_Y_COUNTER, (const uint8_t[]){ 0x00, 0x00 }, 2);
    err |= write_cmd_data(CMD_WRITE_RAM_RED, fb, EPD_FRAMEBUFFER_SIZE);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    err = write_cmd_data(CMD_DEEP_SLEEP, (const uint8_t[]){ 0x01 }, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return err;
}
