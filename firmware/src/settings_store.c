#include "settings_store.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "settings";

#define NVS_NAMESPACE "app"
#define NVS_KEY       "settings"

static settings_t s_settings;
static SemaphoreHandle_t s_lock;
static volatile uint32_t s_generation;

static esp_err_t save_locked(void)
{
    cJSON *json = settings_to_json(&s_settings, true);
    char *text = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (text == NULL) {
        return ESP_ERR_NO_MEM;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_str(h, NVS_KEY, text);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    free(text);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
    }
    return err;
}

static void load(void)
{
    settings_defaults(&s_settings);

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored settings, using defaults");
        return;
    }

    size_t len = 0;
    if (nvs_get_str(h, NVS_KEY, NULL, &len) == ESP_OK && len > 0) {
        char *text = malloc(len);
        if (text != NULL && nvs_get_str(h, NVS_KEY, text, &len) == ESP_OK) {
            cJSON *json = cJSON_Parse(text);
            if (json != NULL) {
                settings_apply_json(&s_settings, json);
                cJSON_Delete(json);
                ESP_LOGI(TAG, "settings loaded");
            } else {
                ESP_LOGW(TAG, "stored settings are corrupt, using defaults");
            }
        }
        free(text);
    }
    nvs_close(h);
}

esp_err_t settings_store_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    load();
    return ESP_OK;
}

void settings_get(settings_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_settings;
    xSemaphoreGive(s_lock);
}

esp_err_t settings_set(const settings_t *s)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_settings = *s;
    settings_sanitize(&s_settings);
    esp_err_t err = save_locked();
    s_generation++;
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t settings_patch_json(const cJSON *patch)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    settings_apply_json(&s_settings, patch);
    esp_err_t err = save_locked();
    s_generation++;
    xSemaphoreGive(s_lock);
    return err;
}

uint32_t settings_generation(void)
{
    return s_generation;
}

esp_err_t settings_forget_network(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    settings_t defaults;
    settings_defaults(&defaults);
    s_settings.wifi = defaults.wifi;
    s_settings.mqtt = defaults.mqtt;
    esp_err_t err = save_locked();
    s_generation++;
    xSemaphoreGive(s_lock);
    ESP_LOGW(TAG, "network settings forgotten");
    return err;
}

esp_err_t settings_factory_reset(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_erase_all(h);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    settings_defaults(&s_settings);
    s_generation++;
    xSemaphoreGive(s_lock);
    ESP_LOGW(TAG, "factory reset");
    return err;
}
