#include "sensors.h"

#include <string.h>
#include <sys/time.h>

#include "esp_log.h"

#include <bmp280.h>
#include <ds3231.h>
#include <i2cdev.h>
#include <mhz19b.h>

#include "app_config.h"

static const char *TAG = "sensors";

static bmp280_t s_bme;
static i2c_dev_t s_rtc;
static mhz19b_dev_t s_mhz;

static bool s_bme_ok;
static bool s_rtc_ok;
static bool s_mhz_ok;

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
    ESP_ERROR_CHECK(ds3231_init_desc(&s_rtc, I2C_NUM_0, PIN_I2C_SDA,
                                     PIN_I2C_SCL));

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
    s_mhz_ok = mhz19b_init(&s_mhz, UART_NUM_1, PIN_MHZ_TX, PIN_MHZ_RX) ==
               ESP_OK;
    if (!s_mhz_ok) {
        ESP_LOGE(TAG, "MH-Z19 UART init failed");
        return;
    }
    /* The sensor lives indoors: automatic baseline calibration off. */
    mhz19b_set_auto_calibration(&s_mhz, false);
    ESP_LOGI(TAG, "MH-Z19 initialized (warm-up takes up to 3 minutes)");
}

void sensors_init(void)
{
    ESP_ERROR_CHECK(i2cdev_init());
    init_bme280();
    init_ds3231();
    init_mhz19();
}

void sensors_read(sensor_data_t *data, float temp_offset_c)
{
    if (s_bme_ok) {
        float t, p, h;
        if (bmp280_read_float(&s_bme, &t, &p, &h) == ESP_OK) {
            data->temperature_c = t + temp_offset_c;
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

bool sensors_calibrate_co2(void)
{
    if (!s_mhz_ok) {
        return false;
    }
    esp_err_t err = mhz19b_start_calibration(&s_mhz);
    ESP_LOGW(TAG, "MH-Z19 zero calibration: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

bool sensors_get_time(struct tm *out)
{
    if (s_rtc_ok && ds3231_get_time(&s_rtc, out) == ESP_OK &&
        out->tm_year >= 120) {
        /* The RTC does not keep the weekday reliably: recompute it. */
        struct tm tmp = *out;
        tmp.tm_isdst = -1;
        mktime(&tmp);
        out->tm_wday = tmp.tm_wday;
        return true;
    }

    /* Fall back to system time (valid once SNTP has synced). */
    time_t t = time(NULL);
    localtime_r(&t, out);
    return out->tm_year >= 120;
}

bool sensors_set_system_time_from_rtc(void)
{
    struct tm local;
    if (!s_rtc_ok || ds3231_get_time(&s_rtc, &local) != ESP_OK ||
        local.tm_year < 120) {
        return false;
    }
    local.tm_isdst = -1;
    time_t t = mktime(&local); /* interprets local time with the current TZ */
    if (t == (time_t)-1) {
        return false;
    }
    struct timeval tv = { .tv_sec = t };
    settimeofday(&tv, NULL);
    return true;
}

void sensors_sync_rtc(void)
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
