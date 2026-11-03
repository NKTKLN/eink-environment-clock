/*
 * State shared between the display loop, the HTTP server and MQTT:
 * latest sensor readings, notifications, the frame on the panel and
 * requests that the main loop carries out (redraw, CO2 calibration,
 * restart). All functions are thread-safe.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "epd_geometry.h"
#include "ui.h"

void app_state_init(void);

/* Last 3 MAC bytes in hex, e.g. "a1b2c3". */
const char *app_device_id(void);
const char *app_version(void);

void app_set_sensors(const sensor_data_t *d);
void app_get_sensors(sensor_data_t *d);

/* User notification (web, MQTT). ttl_s <= 0 = until cleared. */
void app_notify(const char *text, int ttl_s, bool fullscreen);
void app_notify_clear(void);

/* Lower-priority system message (e.g. "Connected: 192.168.1.42"),
 * shown only while no user notification is active. */
void app_system_notice(const char *text, int ttl_s);

typedef struct {
    char text[256];
    bool fullscreen;
    bool is_system;
    int remaining_s; /* -1 = no expiry */
} app_notice_t;

/* Returns false if nothing is showing. *seq changes whenever the visible
 * notice changes, so the display loop knows when to redraw. */
bool app_get_notice(app_notice_t *out, uint32_t *seq);

/* Copy of the frame currently on the panel, for /api/screen.bmp. */
void app_store_frame(const uint8_t *fb);
void app_copy_frame(uint8_t *out);

/* Overlay messages that replace the clock face. */
typedef enum {
    OVERLAY_NONE,
    OVERLAY_OTA,
    OVERLAY_RESET,
    OVERLAY_RESTART,
} overlay_t;

void app_set_overlay(overlay_t o, int progress);
overlay_t app_get_overlay(int *progress);

/* One-shot requests handled by the main loop. */
void app_request_redraw(void);
bool app_take_redraw(void);
void app_request_co2_calibration(void);
bool app_take_co2_calibration(void);

/* Shows the overlay, waits so the HTTP reply / panel refresh can finish,
 * then restarts. Runs in its own task, returns immediately. */
void app_restart_later(overlay_t overlay, int delay_ms);

int64_t app_uptime_ms(void);
