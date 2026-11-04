/* Thread-safe owner of the current settings, persisted to NVS as JSON. */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "settings.h"

/* Initializes NVS (erasing it if the layout changed) and loads settings. */
esp_err_t settings_store_init(void);

void settings_get(settings_t *out);

/* Replaces the settings, sanitizes and saves them. */
esp_err_t settings_set(const settings_t *s);

/* Merges a partial JSON object (same shape as settings_to_json) and saves. */
esp_err_t settings_patch_json(const cJSON *patch);

/* Incremented on every change; consumers compare it to re-apply. */
uint32_t settings_generation(void);

/* Forgets Wi-Fi and MQTT credentials, keeps display preferences. */
esp_err_t settings_forget_network(void);

/* Erases the whole NVS partition. The caller should restart afterwards. */
esp_err_t settings_factory_reset(void);
