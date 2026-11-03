#include "app_state.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_app_desc.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "notify.h"

static SemaphoreHandle_t s_lock;
static char s_device_id[8];
static sensor_data_t s_sensors = { NAN, NAN, NAN, -1 };
static notify_t s_user;
static notify_t s_system;
static uint8_t s_frame[EPD_FRAMEBUFFER_SIZE];
static overlay_t s_overlay;
static int s_overlay_progress = -1;
static bool s_redraw;
static bool s_calibrate;

#define LOCK()   xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

int64_t app_uptime_ms(void)
{
    return esp_timer_get_time() / 1000;
}

void app_state_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    assert(s_lock != NULL);
    notify_init(&s_user);
    notify_init(&s_system);
    memset(s_frame, 0xFF, sizeof(s_frame));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id), "%02x%02x%02x", mac[3],
             mac[4], mac[5]);
}

const char *app_device_id(void)
{
    return s_device_id;
}

const char *app_version(void)
{
    return esp_app_get_description()->version;
}

void app_set_sensors(const sensor_data_t *d)
{
    LOCK();
    s_sensors = *d;
    UNLOCK();
}

void app_get_sensors(sensor_data_t *d)
{
    LOCK();
    *d = s_sensors;
    UNLOCK();
}

void app_notify(const char *text, int ttl_s, bool fullscreen)
{
    LOCK();
    notify_set(&s_user, text, ttl_s, fullscreen, app_uptime_ms());
    UNLOCK();
}

void app_notify_clear(void)
{
    LOCK();
    notify_clear(&s_user);
    UNLOCK();
}

void app_system_notice(const char *text, int ttl_s)
{
    LOCK();
    notify_set(&s_system, text, ttl_s, false, app_uptime_ms());
    UNLOCK();
}

bool app_get_notice(app_notice_t *out, uint32_t *seq)
{
    int64_t now = app_uptime_ms();
    LOCK();
    notify_t *n = NULL;
    if (notify_active(&s_user, now)) {
        n = &s_user;
    } else if (notify_active(&s_system, now)) {
        n = &s_system;
    }
    if (seq != NULL) {
        *seq = s_user.seq * 65599u + s_system.seq;
    }
    if (n != NULL && out != NULL) {
        strlcpy(out->text, n->text, sizeof(out->text));
        out->fullscreen = n->fullscreen;
        out->is_system = n == &s_system;
        out->remaining_s = notify_remaining_s(n, now);
    }
    UNLOCK();
    return n != NULL;
}

void app_store_frame(const uint8_t *fb)
{
    LOCK();
    memcpy(s_frame, fb, sizeof(s_frame));
    UNLOCK();
}

void app_copy_frame(uint8_t *out)
{
    LOCK();
    memcpy(out, s_frame, sizeof(s_frame));
    UNLOCK();
}

void app_set_overlay(overlay_t o, int progress)
{
    LOCK();
    s_overlay = o;
    s_overlay_progress = progress;
    UNLOCK();
}

overlay_t app_get_overlay(int *progress)
{
    LOCK();
    overlay_t o = s_overlay;
    *progress = s_overlay_progress;
    UNLOCK();
    return o;
}

void app_request_redraw(void)
{
    LOCK();
    s_redraw = true;
    UNLOCK();
}

bool app_take_redraw(void)
{
    LOCK();
    bool r = s_redraw;
    s_redraw = false;
    UNLOCK();
    return r;
}

void app_request_co2_calibration(void)
{
    LOCK();
    s_calibrate = true;
    UNLOCK();
}

bool app_take_co2_calibration(void)
{
    LOCK();
    bool r = s_calibrate;
    s_calibrate = false;
    UNLOCK();
    return r;
}

static void restart_task(void *arg)
{
    /* Long enough for the HTTP reply and one partial panel refresh. */
    vTaskDelay(pdMS_TO_TICKS((int)(intptr_t)arg));
    esp_restart();
}

void app_restart_later(overlay_t overlay, int delay_ms)
{
    app_set_overlay(overlay, -1);
    xTaskCreate(restart_task, "restart", 2048, (void *)(intptr_t)delay_ms, 5,
                NULL);
}
