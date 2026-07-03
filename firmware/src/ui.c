#include "ui.h"

#include <stdio.h>

#include "app_config.h"
#include "epd_canvas.h"
#include "icons.h"

/* Layout mirrors the original Arduino sketch: time + date centered in the
 * left 185px, sensor metrics (icon + value + small unit) on the right. */
#define LEFT_ZONE_W   185
#define METRIC_X_ICON 190
#define METRIC_X_TEXT (METRIC_X_ICON + 24)

static const char *MONTHS[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

static void draw_centered(uint8_t *fb, const char *s, int y, int scale)
{
    int x = (LEFT_ZONE_W - canvas_text_width(s, scale)) / 2;
    canvas_text(fb, x, y, s, scale, true);
}

static void draw_time_zone(uint8_t *fb, const struct tm *now, bool time_valid)
{
    char time_str[8];
    char date_str[16];

    if (time_valid) {
        int hour = now->tm_hour;
        if (USE_12_HOUR_FORMAT) {
            hour %= 12;
            if (hour == 0) {
                hour = 12;
            }
        }
        snprintf(time_str, sizeof(time_str), "%02d:%02d", hour, now->tm_min);
        snprintf(date_str, sizeof(date_str), "%d %s %d", now->tm_mday,
                 MONTHS[now->tm_mon], now->tm_year + 1900);
    } else {
        snprintf(time_str, sizeof(time_str), "--:--");
        snprintf(date_str, sizeof(date_str), "--.--.----");
    }

    draw_centered(fb, time_str, 32, 5);
    draw_centered(fb, date_str, 82, 2);
}

static void draw_metric(uint8_t *fb, int y, const uint8_t *icon,
                        const char *value, const char *unit)
{
    canvas_draw_xbm(fb, METRIC_X_ICON, y, icon, 16, 16, true);

    int x = canvas_text(fb, METRIC_X_TEXT, y, value, 2, true);
    canvas_text(fb, x + 1, y + 6, unit, 1, true);
}

static void format_float(char *buf, size_t size, float value, int decimals)
{
    if (isnan(value)) {
        snprintf(buf, size, "---");
    } else {
        snprintf(buf, size, "%.*f", decimals, value);
    }
}

static void draw_sensor_zone(uint8_t *fb, const sensor_data_t *data)
{
    char buf[12];

    format_float(buf, sizeof(buf), data->temperature_c, 1);
    draw_metric(fb, 22, ICON_TEMPERATURE_16X16, buf, "\xF8""C");

    format_float(buf, sizeof(buf), data->humidity_pct, 0);
    draw_metric(fb, 46, ICON_DROPLET_16X16, buf, "%");

    format_float(buf, sizeof(buf), data->pressure_hpa, 0);
    draw_metric(fb, 70, ICON_GAUGE_16X16, buf, "hPa");

    if (data->co2_ppm > 0) {
        snprintf(buf, sizeof(buf), "%d", data->co2_ppm);
    } else {
        snprintf(buf, sizeof(buf), "---");
    }
    draw_metric(fb, 94, ICON_SEEDLING_16X16, buf, "ppm");
}

void ui_render(uint8_t *fb, const struct tm *now, bool time_valid,
               const sensor_data_t *data)
{
    canvas_clear(fb, true);
    draw_time_zone(fb, now, time_valid);
    draw_sensor_zone(fb, data);
}
