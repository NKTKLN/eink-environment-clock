/*
 * Firmware updates from GitHub releases.
 *
 * A background task asks the GitHub API for the latest release once a day
 * (when enabled in the settings) and, with automatic updates on, installs a
 * newer one between 03:00 and 05:00 local time. The web interface can check
 * and install on demand. Downloads use HTTPS with the ESP-IDF certificate
 * bundle; the bootloader rolls back if the new image does not start.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    UPDATE_IDLE,
    UPDATE_CHECKING,
    UPDATE_INSTALLING,
} update_busy_t;

typedef struct {
    update_busy_t busy;
    bool checked;          /* at least one successful check */
    bool available;        /* latest > running */
    char latest[32];
    char notes[512];
    char page[160];
    int size;
    int64_t checked_ago_s; /* -1 = never */
    char error[96];        /* last error, "" if none */
    int progress;          /* 0..100 while installing */
} update_status_t;

void updater_start(void);
void updater_get_status(update_status_t *out);

/* Both return immediately; the work runs in the updater task.
 * ESP_ERR_INVALID_STATE if busy or offline. */
esp_err_t updater_request_check(void);
esp_err_t updater_request_install(void);
