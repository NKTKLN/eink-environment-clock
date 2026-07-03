#include <math.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <bmp280.h>
#include <ds3231.h>
#include <i2cdev.h>
#include <mhz19b.h>

#include "app_config.h"
#include "epd_ssd1680.h"
#include "net_time.h"
#include "ui.h"

static const char *TAG = "app";

static bmp280_t s_bme;
static i2c_dev_t s_rtc;
static mhz19b_dev_t s_mhz;

static bool s_bme_ok;
static bool s_rtc_ok;
static bool s_mhz_ok;

static uint8_t *s_fb;

/* ===================================================== */

static void init_bme280(void)
{
    const uint8_t addrs[] = { BMP280_I2C_ADDRESS_0, BMP280_I2C_ADDRESS_1 };

    bmp280_params_t params;
    bmp280_init_default_params(&params);

    for (size_t i = 0; i < sizeof(addrs); i++) {
        memset(&s_bme, 0, sizeof(s_bme));
        ESP_ERROR_CHECK(bmp280_init_desc(&s_bme, addrs[i], I2C_NUM_0,
                                         PIN_I2C_SDA, PIN_I2C_SCL));
        if (bmp280_init(&s_bme, &params) == ESP_OK) {
            s_bme_ok = true;
            ESP_LOGI(TAG, "BME280 found at 0x%02X (chip id 0x%02X)",
                     addrs[i], s_bme.id);
            return;
        }
        bmp280_free_desc(&s_bme);
    }
    ESP_LOGE(TAG, "BME280 not found");
}

static void init_ds3231(void)
{
    memset(&s_rtc, 0, sizeof(s_rtc));
    ESP_ERROR_CHECK(ds3231_init_desc(&s_rtc, I2C_NUM_0,
                                     PIN_I2C_SDA, PIN_I2C_SCL));

    struct tm probe;
    s_rtc_ok = ds3231_get_time(&s_rtc, &probe) == ESP_OK;
    if (s_rtc_ok) {
        ESP_LOGI(TAG, "DS3231 initialized");
    } else {
        ESP_LOGE(TAG, "DS3231 not found");
    }
}

static void init_mhz19(void)
{
    s_mhz_ok = mhz19b_init(&s_mhz, UART_NUM_1, PIN_MHZ_TX, PIN_MHZ_RX) == ESP_OK;
    if (!s_mhz_ok) {
        ESP_LOGE(TAG, "MH-Z19 UART init failed");
        return;
    }
    /* The sensor lives indoors: automatic baseline calibration off. */
    mhz19b_set_auto_calibration(&s_mhz, false);
    ESP_LOGI(TAG, "MH-Z19 initialized (warm-up takes up to 3 minutes)");
}

/* ===================================================== */

static bool get_current_time(struct tm *out)
{
    if (s_rtc_ok && ds3231_get_time(&s_rtc, out) == ESP_OK &&
        out->tm_year >= 120) {
        return true;
    }

    /* Fall back to system time (valid once SNTP has synced). */
    time_t t = time(NULL);
    localtime_r(&t, out);
    return out->tm_year >= 120;
}

static void sync_rtc_from_system_time(void)
{
    if (!s_rtc_ok) {
        return;
    }

    time_t t = time(NULL);
    struct tm now;
    localtime_r(&t, &now); /* the RTC keeps local time */

    if (ds3231_set_time(&s_rtc, &now) == ESP_OK) {
        ESP_LOGI(TAG, "DS3231 synced from NTP time");
    } else {
        ESP_LOGE(TAG, "DS3231 write failed");
    }
}

static void read_sensors(sensor_data_t *data)
{
    if (s_bme_ok) {
        float t, p, h;
        if (bmp280_read_float(&s_bme, &t, &p, &h) == ESP_OK) {
            data->temperature_c = t + BME280_TEMP_OFFSET_C;
            data->pressure_hpa = p / 100.0f;
            if (s_bme.id == BME280_CHIP_ID) {
                data->humidity_pct = h;
            }
        }
    }

    if (s_mhz_ok) {
        int16_t co2 = 0;
        if (mhz19b_read_co2(&s_mhz, &co2) == ESP_OK && co2 > 0) {
            data->co2_ppm = co2;
        }
    }

    ESP_LOGI(TAG, "T=%.1f C, H=%.0f %%, P=%.0f hPa, CO2=%d ppm",
             data->temperature_c, data->humidity_pct, data->pressure_hpa,
             data->co2_ppm);
}

static bool nearly_equal(float a, float b, float eps)
{
    if (isnan(a) && isnan(b)) {
        return true;
    }
    if (isnan(a) || isnan(b)) {
        return false;
    }
    return fabsf(a - b) <= eps;
}

/* True when the difference would not be visible on the display anyway. */
static bool sensor_data_similar(const sensor_data_t *a, const sensor_data_t *b)
{
    return nearly_equal(a->temperature_c, b->temperature_c, 0.2f) &&
           nearly_equal(a->humidity_pct, b->humidity_pct, 2.0f) &&
           nearly_equal(a->pressure_hpa, b->pressure_hpa, 2.0f) &&
           abs(a->co2_ppm - b->co2_ppm) <= 50;
}

static int minute_key(const struct tm *t)
{
    return (((t->tm_year * 12 + t->tm_mon) * 31 + t->tm_mday) * 24 +
            t->tm_hour) * 60 + t->tm_min;
}

static void update_co2_led(int co2_ppm)
{
    static bool led_on;

    if (co2_ppm > CO2_ALERT_THRESHOLD_PPM) {
        led_on = !led_on;
    } else {
        led_on = false;
    }
    gpio_set_level(PIN_LED, led_on);
}

/* ===================================================== */

void app_main(void)
{
    setenv("TZ", TIME_ZONE, 1);
    tzset();

    gpio_config_t led_cfg = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led_cfg));

    ESP_ERROR_CHECK(i2cdev_init());
    init_bme280();
    init_ds3231();
    init_mhz19();

    ESP_ERROR_CHECK(epd_init());
    s_fb = heap_caps_malloc(EPD_FRAMEBUFFER_SIZE, MALLOC_CAP_DMA);
    assert(s_fb != NULL);

    /* First frame: full (flashing) refresh gives a crisp image right away. */
    sensor_data_t data = {
        .temperature_c = NAN,
        .humidity_pct = NAN,
        .pressure_hpa = NAN,
        .co2_ppm = -1,
    };
    read_sensors(&data);

    struct tm now;
    bool time_valid = get_current_time(&now);

    ui_render(s_fb, &now, time_valid, &data);
    if (epd_full_refresh(s_fb) != ESP_OK) {
        ESP_LOGE(TAG, "display refresh failed");
    }

    if (USE_WIFI_TIME_SYNC) {
        ESP_ERROR_CHECK(net_time_start());
    }

    sensor_data_t shown_data = data;
    int shown_minute = time_valid ? minute_key(&now) : -1;
    int partials_since_full = 0;
    int64_t last_refresh_us = esp_timer_get_time();
    uint32_t tick = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        tick++;

        if (net_time_take_synced()) {
            sync_rtc_from_system_time();
        }

        if (tick % SENSOR_UPDATE_INTERVAL_S == 0) {
            read_sensors(&data);
        }

        time_valid = get_current_time(&now);

        bool time_changed =
            time_valid && minute_key(&now) != shown_minute;
        bool data_changed = !sensor_data_similar(&data, &shown_data);
        bool min_period_passed =
            esp_timer_get_time() - last_refresh_us >=
            (int64_t)DISPLAY_MIN_UPDATE_PERIOD_S * 1000000;

        if ((time_changed || data_changed) && min_period_passed) {
            ui_render(s_fb, &now, time_valid, &data);

            esp_err_t err;
            if (partials_since_full >= DISPLAY_FULL_REFRESH_EVERY_N) {
                err = epd_full_refresh(s_fb);
                partials_since_full = 0;
            } else {
                err = epd_partial_refresh(s_fb);
                partials_since_full++;
            }
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "display refresh failed: %s",
                         esp_err_to_name(err));
            }

            shown_minute = time_valid ? minute_key(&now) : -1;
            shown_data = data;
            last_refresh_us = esp_timer_get_time();
        }

        update_co2_led(data.co2_ppm);
    }
}
