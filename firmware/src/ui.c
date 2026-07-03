#include "ui.h"

#include <stdio.h>

#include "app_config.h"
#include "epd_canvas.h"
#include "icons.h"

/* Layout: left zone (time + date), right zone (sensor metrics). */
#define LEFT_ZONE_W   184
#define METRIC_X_ICON 192
#define METRIC_X_TEXT 214

static const char *MONTHS[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

static void draw_centered(uint8_t *fb, const char *s, int y,
                          canvas_font_t font, int scale)
{
    int x = (LEFT_ZONE_W - canvas_text_width(s, font, scale)) / 2;
    canvas_text(fb, x, y, s, font, scale, true);
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

    draw_centered(fb, time_str, 14, FONT_24, 3); /* 36x72 digits */
    draw_centered(fb, date_str, 96, FONT_12, 2);
}

/* Value in a large font with a small unit suffix; draws "---" when the
 * value is missing. with_degree prepends a degree mark to the unit. */
static void draw_metric(uint8_t *fb, int y, const uint8_t *icon,
                        const char *value, const char *unit, bool with_degree)
{
    canvas_draw_xbm(fb, METRIC_X_ICON, y, icon, 16, 16, true);

    int x = canvas_text(fb, METRIC_X_TEXT, y, value, FONT_16, 1, true);
    x += 2;

    if (with_degree) {
        /* Small hollow square as a degree mark. */
        canvas_fill_rect(fb, x, y + 1, 4, 4, true);
        canvas_fill_rect(fb, x + 1, y + 2, 2, 2, false);
        x += 5;
    }
    canvas_text(fb, x, y + 8, unit, FONT_8, 1, true);
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
    draw_metric(fb, 8, ICON_TEMPERATURE_16X16, buf, "C", true);

    format_float(buf, sizeof(buf), data->humidity_pct, 0);
    draw_metric(fb, 38, ICON_DROPLET_16X16, buf, "%", false);

    format_float(buf, sizeof(buf), data->pressure_hpa, 0);
    draw_metric(fb, 68, ICON_GAUGE_16X16, buf, "hPa", false);

    if (data->co2_ppm > 0) {
        snprintf(buf, sizeof(buf), "%d", data->co2_ppm);
    } else {
        snprintf(buf, sizeof(buf), "---");
    }
    draw_metric(fb, 98, ICON_SEEDLING_16X16, buf, "ppm", false);
}

void ui_render(uint8_t *fb, const struct tm *now, bool time_valid,
               const sensor_data_t *data)
{
    canvas_clear(fb, true);
    draw_time_zone(fb, now, time_valid);
    draw_sensor_zone(fb, data);
}
