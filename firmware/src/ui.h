/*
 * Screen rendering. Pure functions of their inputs (no ESP-IDF calls), so
 * every layout is unit-tested and previewed on the host.
 */
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "settings.h"

typedef struct {
    float temperature_c; /* NAN when unavailable */
    float humidity_pct;  /* NAN when unavailable */
    float pressure_hpa;  /* NAN when unavailable */
    int co2_ppm;         /* -1 when unavailable */
} sensor_data_t;

typedef struct {
    const char *text; /* NULL or "" = no notification */
    bool fullscreen;
} ui_notice_t;

/* Main clock face, honoring layout, enabled metrics, units, language,
 * invert/rotate. notice may be NULL. time_valid = false renders
 * placeholders instead of time/date. */
void ui_render_clock(uint8_t *fb, const settings_t *s, const struct tm *now,
                     bool time_valid, const sensor_data_t *data,
                     const ui_notice_t *notice);

/* Access point screen: Wi-Fi QR code, network name, password, address.
 * failed_ssid != NULL means a saved network could not be reached. */
void ui_render_setup(uint8_t *fb, const settings_t *s, const char *ap_ssid,
                     const char *ap_password, const char *ap_ip,
                     const char *failed_ssid);

/* Centered system message ("Resetting...", OTA progress). progress < 0
 * hides the progress bar. */
void ui_render_message(uint8_t *fb, const settings_t *s, const char *title,
                       const char *subtitle, int progress);

/* ---- helpers exposed for tests and other modules ---- */

#define UI_LINE_MAX 160 /* bytes per wrapped line, UTF-8 */

/* Word-wraps UTF-8 text into at most max_lines lines of max_chars code
 * points. Breaks at spaces, splits words longer than a line and ends the
 * last line with "…" when text is cut. Returns the number of lines and
 * sets *truncated. */
int ui_wrap(const char *text, int max_chars, int max_lines,
            char lines[][UI_LINE_MAX], bool *truncated);

/* Formats a metric in the configured units. Returns false (and "---") when
 * the reading is unavailable. *unit receives a static string. */
bool ui_format_metric(metric_id_t id, const sensor_data_t *d,
                      const settings_t *s, char *buf, size_t size,
                      const char **unit);

/* "WIFI:T:WPA;S:<ssid>;P:<pass>;;" with \ ; , : " escaped. */
void ui_wifi_qr_payload(char *buf, size_t size, const char *ssid,
                        const char *password);

/* Localized "Connected: <ip>" style strings used by the app. */
typedef enum {
    UI_STR_CONNECTED,
    UI_STR_RESETTING,
    UI_STR_RESET_HINT,
    UI_STR_UPDATING,
    UI_STR_UPDATE_HINT,
    UI_STR_RESTARTING,
    UI_STR_SETUP_STRIP, /* printf: AP name, password, address */
    UI_STR_COUNT,
} ui_str_t;

const char *ui_str(const settings_t *s, ui_str_t id);
