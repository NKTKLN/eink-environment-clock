#include "updater.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_log.h"

#include "app_config.h"
#include "app_state.h"
#include "release.h"
#include "sensors.h"
#include "settings_store.h"
#include "wifi_mgr.h"

static const char *TAG = "updater";

#define CHECK_PERIOD_S   (24 * 3600)
#define RETRY_PERIOD_S   (30 * 60)   /* after a failed check */
#define TICK_MS          (60 * 1000)
#define MAX_API_RESPONSE (48 * 1024)

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static update_status_t s_st;
static char s_url[256];
static int64_t s_last_check_ms = -1;
static int64_t s_last_attempt_ms = -1;

#define NOTIFY_CHECK   (1u << 0)
#define NOTIFY_INSTALL (1u << 1)

#define LOCK()   xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

static void set_error(const char *msg)
{
    LOCK();
    snprintf(s_st.error, sizeof(s_st.error), "%s", msg);
    UNLOCK();
    ESP_LOGW(TAG, "%s", msg);
}

/* ===================================================== */

static esp_err_t check(void)
{
    LOCK();
    s_st.busy = UPDATE_CHECKING;
    s_st.error[0] = '\0';
    UNLOCK();
    s_last_attempt_ms = app_uptime_ms();

    esp_err_t result = ESP_FAIL;
    char *body = NULL;
    esp_http_client_config_t cfg = {
        .url = "https://api.github.com/repos/" UPDATE_REPO "/releases/latest",
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .buffer_size = 2048,
        .user_agent = "eink-environment-clock",
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        set_error("HTTP client init failed");
        goto done;
    }
    esp_http_client_set_header(client, "Accept", "application/vnd.github+json");

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        set_error("cannot reach api.github.com");
        goto done;
    }
    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status == 404) {
        /* A repository without releases: a normal answer, not an error. */
        LOCK();
        s_st.checked = true;
        s_st.available = false;
        s_st.latest[0] = '\0';
        s_st.notes[0] = '\0';
        s_st.page[0] = '\0';
        s_url[0] = '\0';
        UNLOCK();
        s_last_check_ms = app_uptime_ms();
        ESP_LOGI(TAG, "no releases published yet");
        result = ESP_OK;
        goto done;
    }
    if (status != 200) {
        char msg[48];
        snprintf(msg, sizeof(msg), "GitHub answered HTTP %d", status);
        set_error(msg);
        goto done;
    }

    body = malloc(MAX_API_RESPONSE);
    if (body == NULL) {
        set_error("out of memory");
        goto done;
    }
    int len = 0;
    while (len < MAX_API_RESPONSE - 1) {
        int r = esp_http_client_read(client, body + len,
                                     MAX_API_RESPONSE - 1 - len);
        if (r <= 0) {
            break;
        }
        len += r;
    }
    body[len] = '\0';

    static release_t rel;
    if (!release_parse(body, (size_t)len, UPDATE_ASSET, &rel)) {
        set_error("latest release has no " UPDATE_ASSET);
        goto done;
    }

    bool newer = version_compare(rel.version, app_version()) > 0;
    LOCK();
    s_st.checked = true;
    s_st.available = newer && !rel.prerelease;
    strlcpy(s_st.latest, rel.version, sizeof(s_st.latest));
    strlcpy(s_st.notes, rel.notes, sizeof(s_st.notes));
    strlcpy(s_st.page, rel.page, sizeof(s_st.page));
    s_st.size = rel.size;
    strlcpy(s_url, rel.url, sizeof(s_url));
    UNLOCK();
    s_last_check_ms = app_uptime_ms();
    ESP_LOGI(TAG, "latest release %s, running %s%s", rel.version,
             app_version(), newer ? " -> update available" : "");
    result = ESP_OK;

done:
    free(body);
    if (client != NULL) {
        esp_http_client_cleanup(client);
    }
    LOCK();
    s_st.busy = UPDATE_IDLE;
    UNLOCK();
    return result;
}

static void install(void)
{
    char url[sizeof(s_url)];
    LOCK();
    bool ok = s_st.available && s_url[0] != '\0';
    strlcpy(url, s_url, sizeof(url));
    s_st.busy = UPDATE_INSTALLING;
    s_st.progress = 0;
    s_st.error[0] = '\0';
    UNLOCK();

    if (!ok) {
        set_error("no newer release to install");
        goto fail;
    }
    ESP_LOGW(TAG, "installing %s", url);
    app_set_overlay(OVERLAY_OTA, 0);

    esp_http_client_config_t http = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 20000,
        .buffer_size = 4096,     /* GitHub redirect headers are long */
        .buffer_size_tx = 2048,
        .keep_alive_enable = true,
        .user_agent = "eink-environment-clock",
    };
    esp_https_ota_config_t ota = { .http_config = &http };
    esp_https_ota_handle_t h = NULL;
    esp_err_t err = esp_https_ota_begin(&ota, &h);
    if (err != ESP_OK) {
        set_error("download failed to start");
        goto fail;
    }

    int total = esp_https_ota_get_image_size(h);
    int last = -1;
    while ((err = esp_https_ota_perform(h)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        int done = esp_https_ota_get_image_len_read(h);
        int pct = total > 0 ? done * 100 / total : 0;
        if (pct / 10 != last / 10) {
            last = pct;
            app_set_overlay(OVERLAY_OTA, pct);
            LOCK();
            s_st.progress = pct;
            UNLOCK();
        }
    }
    if (err == ESP_OK && !esp_https_ota_is_complete_data_received(h)) {
        err = ESP_FAIL;
    }
    if (err != ESP_OK) {
        esp_https_ota_abort(h);
        set_error("download interrupted");
        goto fail;
    }
    err = esp_https_ota_finish(h); /* validates the image, sets boot slot */
    if (err != ESP_OK) {
        set_error(err == ESP_ERR_OTA_VALIDATE_FAILED
                      ? "downloaded image is not valid firmware"
                      : "could not finish the update");
        goto fail;
    }

    ESP_LOGW(TAG, "update installed, restarting");
    LOCK();
    s_st.progress = 100;
    UNLOCK();
    app_restart_later(OVERLAY_RESTART, RESTART_DELAY_MS);
    return; /* stays INSTALLING until the restart */

fail:
    app_set_overlay(OVERLAY_NONE, -1);
    app_request_redraw();
    LOCK();
    s_st.busy = UPDATE_IDLE;
    UNLOCK();
}

/* ===================================================== */

static bool online(void)
{
    wifi_status_t ws;
    wifi_mgr_get_status(&ws);
    return ws.sta_connected;
}

static bool night_window(void)
{
    struct tm now;
    if (!sensors_get_time(&now)) {
        return false; /* never auto-install without knowing the time */
    }
    return now.tm_hour >= 3 && now.tm_hour < 5;
}

static void updater_task(void *arg)
{
    /* Let Wi-Fi and SNTP settle after boot. */
    vTaskDelay(pdMS_TO_TICKS(60 * 1000));

    while (true) {
        uint32_t req = 0;
        xTaskNotifyWait(0, UINT32_MAX, &req, pdMS_TO_TICKS(TICK_MS));

        settings_t s;
        settings_get(&s);
        if (!online()) {
            continue;
        }

        if (req & NOTIFY_INSTALL) {
            install();
            continue;
        }

        int64_t now = app_uptime_ms();
        bool due = s_last_check_ms < 0 ||
                   now - s_last_check_ms >= (int64_t)CHECK_PERIOD_S * 1000;
        bool retry_ok = s_last_attempt_ms < 0 ||
                        now - s_last_attempt_ms >= (int64_t)RETRY_PERIOD_S * 1000;
        if ((req & NOTIFY_CHECK) || (s.update.check && due && retry_ok)) {
            check();
        }

        update_status_t st;
        updater_get_status(&st);
        if (s.update.auto_install && st.available && night_window()) {
            ESP_LOGW(TAG, "automatic update to %s", st.latest);
            install();
        }
    }
}

void updater_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_st.checked_ago_s = -1;
    /* TLS needs a deep stack. */
    xTaskCreate(updater_task, "updater", 8192, NULL, 3, &s_task);
}

void updater_get_status(update_status_t *out)
{
    LOCK();
    *out = s_st;
    UNLOCK();
    out->checked_ago_s = s_last_check_ms < 0
                             ? -1
                             : (app_uptime_ms() - s_last_check_ms) / 1000;
}

static esp_err_t request(uint32_t bit)
{
    if (s_task == NULL || !online()) {
        return ESP_ERR_INVALID_STATE;
    }
    LOCK();
    bool busy = s_st.busy != UPDATE_IDLE;
    if (!busy && bit == NOTIFY_CHECK) {
        s_st.busy = UPDATE_CHECKING; /* visible to the UI right away */
    }
    UNLOCK();
    if (busy) {
        return ESP_ERR_INVALID_STATE;
    }
    xTaskNotify(s_task, bit, eSetBits);
    return ESP_OK;
}

esp_err_t updater_request_check(void)
{
    return request(NOTIFY_CHECK);
}

esp_err_t updater_request_install(void)
{
    update_status_t st;
    updater_get_status(&st);
    if (!st.available) {
        return ESP_ERR_NOT_FOUND;
    }
    return request(NOTIFY_INSTALL);
}
