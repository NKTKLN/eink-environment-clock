#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "app_config.h"
#include "app_state.h"
#include "boot_guard.h"
#include "diag.h"
#include "log_capture.h"
#include "epd_ssd1680.h"
#include "mqtt_ha.h"
#include "net_time.h"
#include "sensors.h"
#include "settings_store.h"
#include "ui.h"
#include "updater.h"
#include "web_server.h"
#include "wifi_mgr.h"

static const char *TAG = "app";

#define TICK_MS              500
#define OTA_CONFIRM_AFTER_MS 30000
#define HEALTH_LOG_PERIOD_MS (5 * 60 * 1000)

typedef enum {
    SCREEN_NONE,
    SCREEN_CLOCK,
    SCREEN_SETUP,
    SCREEN_MESSAGE,
} screen_t;

static uint8_t *s_fb;
static uint8_t *s_shown;

/* ===================================================== */

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

/* True when the difference is not worth a panel refresh. */
static bool sensor_data_similar(const sensor_data_t *a, const sensor_data_t *b)
{
    return nearly_equal(a->temperature_c, b->temperature_c, 0.2f) &&
           nearly_equal(a->humidity_pct, b->humidity_pct, 2.0f) &&
           nearly_equal(a->pressure_hpa, b->pressure_hpa, 2.0f) &&
           abs(a->co2_ppm - b->co2_ppm) <= 50;
}

static void update_co2_led(const settings_t *s, int co2_ppm)
{
    static bool led_on;

    if (s->sensors.co2_led && co2_ppm > s->sensors.co2_threshold_ppm) {
        led_on = !led_on;
    } else {
        led_on = false;
    }
    gpio_set_level(PIN_LED, led_on);
}

static void apply_timezone(const settings_t *s)
{
    setenv("TZ", s->display.tz_posix, 1);
    tzset();
}

/* ===================================================== */

/* Renders the screen that should be visible now into s_fb. */
static screen_t render(const settings_t *s, const sensor_data_t *shown_data)
{
    int progress;
    overlay_t overlay = app_get_overlay(&progress);
    if (overlay != OVERLAY_NONE) {
        const char *title = "";
        const char *hint = NULL;
        switch (overlay) {
        case OVERLAY_OTA:
            title = ui_str(s, UI_STR_UPDATING);
            hint = ui_str(s, UI_STR_UPDATE_HINT);
            break;
        case OVERLAY_RESET:
            title = ui_str(s, UI_STR_RESETTING);
            hint = ui_str(s, UI_STR_RESET_HINT);
            break;
        default:
            title = ui_str(s, UI_STR_RESTARTING);
            break;
        }
        ui_render_message(s_fb, s, title, hint, progress);
        return SCREEN_MESSAGE;
    }

    /* First setup: the whole screen explains how to connect. */
    wifi_status_t ws;
    wifi_mgr_get_status(&ws);
    bool setup_ap = ws.ap_active && !ws.sta_connected;
    if (setup_ap && !ws.ap_after_failure) {
        ui_render_setup(s_fb, s, ws.ap_ssid, ws.ap_password, WIFI_AP_IP,
                        NULL);
        return SCREEN_SETUP;
    }

    struct tm now;
    bool time_valid = sensors_get_time(&now);
    static app_notice_t n;
    static bool notice_was_shown;
    static char strip[128];
    ui_notice_t notice = { 0 };
    bool has_notice = app_get_notice(&n, NULL);
    if (has_notice != notice_was_shown) {
        notice_was_shown = has_notice;
        if (has_notice) {
            ESP_LOGI(TAG, "message shown: \"%s\" (%s)", n.text,
                     n.remaining_s < 0 ? "until removed" : "with expiry");
        } else {
            ESP_LOGI(TAG, "message removed");
        }
    }
    if (has_notice) {
        notice.text = n.text;
        notice.fullscreen = n.fullscreen;
    }
    /* Saved network lost: keep the clock running, put the access point
     * details in the bottom strip (a user notification still wins). */
    if (setup_ap && (!has_notice || n.is_system)) {
        snprintf(strip, sizeof(strip), ui_str(s, UI_STR_SETUP_STRIP),
                 ws.ap_ssid, ws.ap_password, WIFI_AP_IP);
        notice.text = strip;
        notice.fullscreen = false;
    }
    ui_render_clock(s_fb, s, &now, time_valid, shown_data, &notice);
    return SCREEN_CLOCK;
}

void app_main(void)
{
    log_capture_start();
    app_state_init();
    ESP_ERROR_CHECK(settings_store_init());

    bool network_reset = boot_guard_check();
    if (network_reset) {
        ESP_LOGW(TAG, "%d quick power cycles: forgetting network settings",
                 BOOT_GUARD_CYCLES);
        settings_forget_network();
        app_set_overlay(OVERLAY_RESET, -1);
    }

    settings_t settings;
    settings_get(&settings);
    uint32_t settings_gen = settings_generation();
    apply_timezone(&settings);

    gpio_config_t led_cfg = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led_cfg));

    sensors_init();
    if (sensors_set_system_time_from_rtc()) {
        ESP_LOGI(TAG, "system time set from the RTC");
    }

    ESP_ERROR_CHECK(epd_init());
    s_fb = heap_caps_malloc(EPD_FRAMEBUFFER_SIZE, MALLOC_CAP_DMA);
    s_shown = malloc(EPD_FRAMEBUFFER_SIZE);
    assert(s_fb != NULL && s_shown != NULL);

    sensor_data_t data = { NAN, NAN, NAN, -1 };
    sensors_read(&data, settings.sensors.temp_offset_c);
    app_set_sensors(&data);
    sensor_data_t shown_data = data;

    ESP_ERROR_CHECK(wifi_mgr_start());
    ESP_ERROR_CHECK(web_server_start());
    updater_start();

    screen_t shown_screen = SCREEN_NONE;
    int partials_since_full = 0;
    int64_t last_refresh_ms = -1000000;
    int64_t last_sensor_ms = app_uptime_ms();
    bool guard_cleared = false;
    bool ota_confirmed = false;
    bool settings_changed_visibly = false;
    int64_t next_health_ms = 60 * 1000; /* first report once things settled */

    if (network_reset) {
        render(&settings, &shown_data);
        epd_full_refresh(s_fb);
        vTaskDelay(pdMS_TO_TICKS(3000));
        app_set_overlay(OVERLAY_NONE, -1);
    }

    while (true) {
        int64_t now_ms = app_uptime_ms();

        if (!guard_cleared && now_ms >= BOOT_GUARD_WINDOW_MS) {
            boot_guard_clear();
            guard_cleared = true;
        }
        if (!ota_confirmed && now_ms >= OTA_CONFIRM_AFTER_MS) {
            /* Ran for a while without crashing: keep this image. */
            esp_ota_mark_app_valid_cancel_rollback();
            ota_confirmed = true;
        }

        if (now_ms >= next_health_ms) {
            diag_log();
            next_health_ms = now_ms + HEALTH_LOG_PERIOD_MS;
        }

        if (settings_generation() != settings_gen) {
            settings_t old = settings;
            settings_gen = settings_generation();
            settings_get(&settings);
            if (strcmp(old.display.tz_posix, settings.display.tz_posix) != 0) {
                /* The RTC keeps local time. Read it in the old zone, then
                 * write the same instant back in the new one, so the shown
                 * time is right even without a network. */
                bool have_time = net_time_valid() ||
                                 sensors_set_system_time_from_rtc();
                apply_timezone(&settings);
                if (have_time) {
                    sensors_sync_rtc();
                }
                ESP_LOGI(TAG, "time zone changed to %s", settings.display.tz_posix);
            }
            settings_changed_visibly =
                old.display.invert != settings.display.invert ||
                old.display.rotate180 != settings.display.rotate180;
        }

        if (net_time_take_synced()) {
            sensors_sync_rtc();
        }

        if (app_take_co2_calibration()) {
            sensors_calibrate_co2();
        }

        bool sensors_updated = false;
        if (now_ms - last_sensor_ms >= SENSOR_UPDATE_INTERVAL_S * 1000) {
            last_sensor_ms = now_ms;
            sensors_read(&data, settings.sensors.temp_offset_c);
            app_set_sensors(&data);
            sensors_updated = true;
        }
        if (!sensor_data_similar(&data, &shown_data)) {
            shown_data = data;
        }

        bool urgent = app_take_redraw();
        screen_t screen = render(&settings, &shown_data);
        bool kind_changed = screen != shown_screen;
        bool differs = memcmp(s_fb, s_shown, EPD_FRAMEBUFFER_SIZE) != 0;
        bool period_ok = now_ms - last_refresh_ms >=
                         DISPLAY_MIN_UPDATE_PERIOD_S * 1000;

        if (differs && (period_ok || urgent || kind_changed)) {
            esp_err_t err;
            bool full = kind_changed || settings_changed_visibly ||
                        partials_since_full >= DISPLAY_FULL_REFRESH_EVERY_N;
            /* Progress updates during OTA must stay fast. */
            if (screen == SCREEN_MESSAGE && !kind_changed) {
                full = false;
            }
            if (full) {
                err = epd_full_refresh(s_fb);
                partials_since_full = 0;
            } else {
                err = epd_partial_refresh(s_fb);
                partials_since_full++;
            }
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "display refresh failed: %s",
                         esp_err_to_name(err));
            } else {
                ESP_LOGD(TAG, "%s refresh", full ? "full" : "partial");
            }
            memcpy(s_shown, s_fb, EPD_FRAMEBUFFER_SIZE);
            app_store_frame(s_fb);
            shown_screen = screen;
            settings_changed_visibly = false;
            last_refresh_ms = app_uptime_ms();
        }

        update_co2_led(&settings, data.co2_ppm);
        mqtt_ha_tick(sensors_updated);

        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}
