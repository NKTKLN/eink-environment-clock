#include "ui.h"

#include <stdio.h>
#include <string.h>

#include "epd_canvas.h"
#include "icons.h"
#include "third_party/qrcodegen.h"

#define MARGIN       4
#define ICON_SIZE    16
#define CLASSIC_SPLIT 185 /* left zone width when the metrics column is on */

/* ===================================================== */
/* Strings                                               */
/* ===================================================== */

static const char *MONTHS[LANG_COUNT][12] = {
    { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" },
    { "янв", "фев", "мар", "апр", "мая", "июн",
      "июл", "авг", "сен", "окт", "ноя", "дек" },
};

static const char *WEEKDAYS[LANG_COUNT][7] = {
    { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" },
    { "Вс", "Пн", "Вт", "Ср", "Чт", "Пт", "Сб" },
};

static const char *STRINGS[LANG_COUNT][UI_STR_COUNT] = {
    {
        [UI_STR_CONNECTED] = "Connected: %s",
        [UI_STR_RESETTING] = "Resetting",
        [UI_STR_RESET_HINT] = "Wi-Fi and MQTT settings cleared",
        [UI_STR_UPDATING] = "Updating",
        [UI_STR_UPDATE_HINT] = "Do not unplug the clock",
        [UI_STR_RESTARTING] = "Restarting",
        [UI_STR_SETUP_STRIP] = "No Wi-Fi. Setup: %s · password %s · %s",
    },
    {
        [UI_STR_CONNECTED] = "Подключено: %s",
        [UI_STR_RESETTING] = "Сброс",
        [UI_STR_RESET_HINT] = "Настройки Wi-Fi и MQTT удалены",
        [UI_STR_UPDATING] = "Обновление",
        [UI_STR_UPDATE_HINT] = "Не отключайте питание",
        [UI_STR_RESTARTING] = "Перезапуск",
        [UI_STR_SETUP_STRIP] = "Нет Wi-Fi. Настройка: %s · пароль %s · %s",
    },
};

const char *ui_str(const settings_t *s, ui_str_t id)
{
    return STRINGS[s->display.language][id];
}

typedef struct {
    const char *title;
    const char *title_failed;
    const char *network;
    const char *password;
    const char *open;
} setup_strings_t;

static const setup_strings_t SETUP_STRINGS[LANG_COUNT] = {
    { "WI-FI SETUP", "NO CONNECTION: ", "Network", "Password", "Then open" },
    { "НАСТРОЙКА WI-FI", "НЕТ СВЯЗИ: ", "Сеть", "Пароль", "Затем откройте" },
};

/* ===================================================== */
/* Text helpers                                          */
/* ===================================================== */

/* Visible width: the last glyph has no trailing spacing column. */
static int text_w(const char *s, int scale)
{
    int w = canvas_text_width(s, scale);
    return w > 0 ? w - scale : 0;
}

static void text_centered(uint8_t *fb, const char *s, int x0, int w, int y,
                          int scale, bool black)
{
    canvas_text(fb, x0 + (w - text_w(s, scale)) / 2, y, s, scale, black);
}

int ui_wrap(const char *text, int max_chars, int max_lines,
            char lines[][UI_LINE_MAX], bool *truncated)
{
    int n = 0;
    int cur = 0; /* code points in the current line */
    bool trunc = false;
    bool force_break = false;
    const char *p = text != NULL ? text : "";

    if (max_chars < 2 || max_lines < 1) {
        *truncated = *p != '\0';
        return 0;
    }

#define NEW_LINE()                     \
    do {                               \
        if (n == max_lines) {          \
            trunc = true;              \
        } else {                       \
            lines[n++][0] = '\0';      \
            cur = 0;                   \
        }                              \
    } while (0)

#define APPEND(src, bytes)                                            \
    do {                                                              \
        size_t _len = strlen(lines[n - 1]);                           \
        size_t _b = (bytes);                                          \
        if (_len + _b < UI_LINE_MAX - 4) {                            \
            memcpy(lines[n - 1] + _len, (src), _b);                   \
            lines[n - 1][_len + _b] = '\0';                           \
        }                                                             \
    } while (0)

    while (*p != '\0' && !trunc) {
        if (*p == ' ' || *p == '\t' || *p == '\r') {
            p++;
            continue;
        }
        if (*p == '\n') {
            force_break = true;
            p++;
            continue;
        }

        const char *w = p;
        int wlen = 0;
        while (*p != '\0' && *p != ' ' && *p != '\n' && *p != '\t' &&
               *p != '\r') {
            canvas_utf8_next(&p);
            wlen++;
        }

        if (n > 0 && !force_break && cur > 0 && cur + 1 + wlen <= max_chars) {
            APPEND(" ", 1);
            APPEND(w, (size_t)(p - w));
            cur += 1 + wlen;
            continue;
        }

        if (n == 0 || cur > 0 || force_break) {
            NEW_LINE();
            if (trunc) {
                break;
            }
        }
        force_break = false;

        /* Long words are split across lines. */
        const char *q = w;
        int remaining = wlen;
        while (remaining > 0) {
            int take = remaining < max_chars ? remaining : max_chars;
            size_t b = canvas_utf8_prefix(q, take);
            APPEND(q, b);
            cur = take;
            q += b;
            remaining -= take;
            if (remaining > 0) {
                NEW_LINE();
                if (trunc) {
                    break;
                }
            }
        }
    }

    if (trunc && n > 0) {
        char *last = lines[n - 1];
        int len = canvas_strlen(last);
        if (len > max_chars - 1) {
            last[canvas_utf8_prefix(last, max_chars - 1)] = '\0';
        }
        size_t b = strlen(last);
        while (b > 0 && last[b - 1] == ' ') {
            last[--b] = '\0';
        }
        strcat(last, "…");
    }

#undef NEW_LINE
#undef APPEND

    *truncated = trunc;
    return n;
}

/* ===================================================== */
/* Formatting                                            */
/* ===================================================== */

static void format_time(char *buf, size_t size, const settings_t *s,
                        const struct tm *now, bool valid)
{
    if (!valid) {
        snprintf(buf, size, "--:--");
        return;
    }
    int hour = now->tm_hour;
    if (!s->display.hour24) {
        hour %= 12;
        if (hour == 0) {
            hour = 12;
        }
    }
    snprintf(buf, size, "%02d:%02d", hour, now->tm_min);
}

static void format_date(char *buf, size_t size, const settings_t *s,
                        const struct tm *now, bool valid)
{
    language_t lang = s->display.language;
    int year = now->tm_year + 1900;

    switch (s->display.date_format) {
    case DATE_WEEKDAY:
        if (valid) {
            snprintf(buf, size, "%s, %d %s", WEEKDAYS[lang][now->tm_wday % 7],
                     now->tm_mday, MONTHS[lang][now->tm_mon % 12]);
        } else {
            snprintf(buf, size, "--, -- ---");
        }
        break;
    case DATE_DMY_DOTS:
        if (valid) {
            snprintf(buf, size, "%02d.%02d.%04d", now->tm_mday,
                     now->tm_mon + 1, year);
        } else {
            snprintf(buf, size, "--.--.----");
        }
        break;
    case DATE_ISO:
        if (valid) {
            snprintf(buf, size, "%04d-%02d-%02d", year, now->tm_mon + 1,
                     now->tm_mday);
        } else {
            snprintf(buf, size, "----------");
        }
        break;
    case DATE_DMY_TEXT:
    default:
        if (valid) {
            snprintf(buf, size, "%d %s %d", now->tm_mday,
                     MONTHS[lang][now->tm_mon % 12], year);
        } else {
            snprintf(buf, size, "--.--.----");
        }
        break;
    }
}

bool ui_format_metric(metric_id_t id, const sensor_data_t *d,
                      const settings_t *s, char *buf, size_t size,
                      const char **unit)
{
    bool ru = s->display.language == LANG_RU;
    float v = NAN;
    int decimals = 0;

    switch (id) {
    case METRIC_TEMPERATURE:
        v = d->temperature_c;
        decimals = 1;
        if (s->display.temp_unit == TEMP_UNIT_F) {
            v = v * 9.0f / 5.0f + 32.0f;
            *unit = "°F";
        } else {
            *unit = "°C";
        }
        break;
    case METRIC_HUMIDITY:
        v = d->humidity_pct;
        *unit = "%";
        break;
    case METRIC_PRESSURE:
        v = d->pressure_hpa;
        if (s->display.pressure_unit == PRESSURE_MMHG) {
            v *= 0.750062f;
            *unit = ru ? "мм" : "mmHg";
        } else {
            *unit = ru ? "гПа" : "hPa";
        }
        break;
    case METRIC_CO2:
    default:
        v = d->co2_ppm > 0 ? (float)d->co2_ppm : NAN;
        *unit = "ppm";
        break;
    }

    if (isnan(v)) {
        snprintf(buf, size, "---");
        return false;
    }
    snprintf(buf, size, "%.*f", decimals, v);
    /* "-0.0" looks odd on a clock */
    if (strcmp(buf, "-0.0") == 0 || strcmp(buf, "-0") == 0) {
        memmove(buf, buf + 1, strlen(buf));
    }
    return true;
}

static const uint8_t *metric_icon(metric_id_t id)
{
    switch (id) {
    case METRIC_TEMPERATURE:
        return ICON_TEMPERATURE_16X16;
    case METRIC_HUMIDITY:
        return ICON_DROPLET_16X16;
    case METRIC_PRESSURE:
        return ICON_GAUGE_16X16;
    case METRIC_CO2:
    default:
        return ICON_SEEDLING_16X16;
    }
}

static bool co2_alarm(metric_id_t id, const sensor_data_t *d,
                      const settings_t *s)
{
    return id == METRIC_CO2 && d->co2_ppm > s->sensors.co2_threshold_ppm;
}

static int enabled_metrics(const settings_t *s, metric_id_t *out)
{
    int n = 0;
    for (int i = 0; i < METRIC_COUNT; i++) {
        if (s->display.metrics[i].enabled) {
            out[n++] = s->display.metrics[i].id;
        }
    }
    return n;
}

/* ===================================================== */
/* Building blocks                                       */
/* ===================================================== */

/* Icon + value + small unit, as in the original firmware. Returns width. */
static int metric_row_width(const char *value, const char *unit,
                            int vscale, int uscale)
{
    return ICON_SIZE + 6 + text_w(value, vscale) + uscale +
           (uscale > 0 ? text_w(unit, uscale) : 0);
}

/* Draws an inline metric with its top at y. Height = max(16, 8*vscale).
 * Values above the CO2 threshold are drawn white on a black plate. */
static void draw_metric_row(uint8_t *fb, int x, int y, metric_id_t id,
                            const sensor_data_t *d, const settings_t *s,
                            int vscale, int uscale)
{
    char value[16];
    const char *unit;
    ui_format_metric(id, d, s, value, sizeof(value), &unit);

    int h = 7 * vscale > ICON_SIZE ? 7 * vscale : ICON_SIZE;
    bool alarm = co2_alarm(id, d, s);
    if (alarm) {
        int w = metric_row_width(value, unit, vscale, uscale);
        canvas_fill_rect(fb, x - 3, y - 3, w + 6, h + 6, true);
    }

    canvas_draw_xbm(fb, x, y + (h - ICON_SIZE) / 2, metric_icon(id),
                    ICON_SIZE, ICON_SIZE, !alarm);
    int vy = y + (h - 7 * vscale) / 2;
    int tx = canvas_text(fb, x + ICON_SIZE + 6, vy, value, vscale, !alarm);
    if (uscale > 0) {
        /* unit sits on the value's baseline */
        canvas_text(fb, tx + 1 - vscale + uscale,
                    vy + 7 * vscale - 7 * uscale, unit, uscale, !alarm);
    }
}

/* Big stacked metric: icon, large value, unit below. */
static void draw_metric_big(uint8_t *fb, int x0, int w, int y0, int h,
                            metric_id_t id, const sensor_data_t *d,
                            const settings_t *s, int vscale)
{
    char value[16];
    const char *unit;
    ui_format_metric(id, d, s, value, sizeof(value), &unit);

    while (vscale > 1 && text_w(value, vscale) > w - 8) {
        vscale--;
    }
    int block = ICON_SIZE + 6 + 7 * vscale + 6 + 7;
    int y = y0 + (h - block) / 2;
    bool alarm = co2_alarm(id, d, s);

    if (alarm) {
        canvas_fill_rect(fb, x0 + 2, y - 4, w - 4, block + 8, true);
    }
    canvas_draw_xbm(fb, x0 + (w - ICON_SIZE) / 2, y, metric_icon(id),
                    ICON_SIZE, ICON_SIZE, !alarm);
    y += ICON_SIZE + 6;
    text_centered(fb, value, x0, w, y, vscale, !alarm);
    y += 7 * vscale + 6;
    text_centered(fb, unit, x0, w, y, 1, !alarm);
}

/* Time (+ date) block, as large as fits into the given box. */
static void draw_clock_block(uint8_t *fb, const settings_t *s,
                             const struct tm *now, bool valid, int x0, int w,
                             int y0, int h, int max_scale)
{
    char time_str[8];
    char date_str[32];
    format_time(time_str, sizeof(time_str), s, now, valid);
    format_date(date_str, sizeof(date_str), s, now, valid);
    bool date = s->display.show_date;

    int tscale = max_scale;
    int dscale = 2;
    for (; tscale > 1; tscale--) {
        dscale = tscale >= 7 ? 3 : 2;
        if (text_w(date_str, dscale) > w - 2 * MARGIN) {
            dscale = 2;
        }
        int block = 7 * tscale + (date ? 3 * tscale + 7 * dscale : 0);
        if (text_w(time_str, tscale) <= w - 2 * MARGIN && block <= h - 8) {
            break;
        }
    }
    while (dscale > 1 && text_w(date_str, dscale) > w - 2 * MARGIN) {
        dscale--;
    }

    int block = 7 * tscale + (date ? 3 * tscale + 7 * dscale : 0);
    int y = y0 + (h - block) / 2;
    text_centered(fb, time_str, x0, w, y, tscale, true);
    if (date) {
        text_centered(fb, date_str, x0, w, y + 10 * tscale, dscale, true);
    }
}

/* Vertical list of metrics centered in a column. */
static void draw_metric_column(uint8_t *fb, const settings_t *s,
                               const sensor_data_t *d, const metric_id_t *ids,
                               int n, int x0, int w, int y0, int h)
{
    if (n == 1) {
        draw_metric_big(fb, x0, w, y0, h, ids[0], d, s, 3);
        return;
    }

    int gap = n >= 3 ? 8 : 18;
    int block = n * ICON_SIZE + (n - 1) * gap;
    while (block > h - 4 && gap > 2) {
        gap--;
        block = n * ICON_SIZE + (n - 1) * gap;
    }
    int y = y0 + (h - block) / 2;
    for (int i = 0; i < n; i++) {
        draw_metric_row(fb, x0 + 5, y, ids[i], d, s, 2, 1);
        y += ICON_SIZE + gap;
    }
}

/* Grid of big metrics (sensors-only layout). */
static void draw_metric_grid(uint8_t *fb, const settings_t *s,
                             const sensor_data_t *d, const metric_id_t *ids,
                             int n, int y0, int h)
{
    if (n == 1) {
        draw_metric_big(fb, 0, CANVAS_W, y0, h, ids[0], d, s, 6);
        return;
    }

    int rows = n <= 2 ? 1 : 2;
    int cell_h = h / rows;
    for (int i = 0; i < n; i++) {
        int row = i / 2;
        int in_row = (row == rows - 1 && n % 2 == 1) ? 1 : 2;
        int col = i % 2;
        int cell_w = CANVAS_W / in_row;
        int cx = col * cell_w;
        int cy = y0 + row * cell_h;

        char value[16];
        const char *unit;
        ui_format_metric(ids[i], d, s, value, sizeof(value), &unit);

        int vscale = rows == 1 ? 4 : 3;
        int uscale = 2;
        while (metric_row_width(value, unit, vscale, uscale) > cell_w - 10) {
            if (uscale > 1) {
                uscale--;
            } else if (vscale > 1) {
                vscale--;
            } else {
                break;
            }
        }
        int w = metric_row_width(value, unit, vscale, uscale);
        int rh = 7 * vscale > ICON_SIZE ? 7 * vscale : ICON_SIZE;
        draw_metric_row(fb, cx + (cell_w - w) / 2, cy + (cell_h - rh) / 2,
                        ids[i], d, s, vscale, uscale);
    }
}

/* Big CO2 on top, time and the other metrics in a strip below. */
static void draw_co2_layout(uint8_t *fb, const settings_t *s,
                            const struct tm *now, bool valid,
                            const sensor_data_t *d, int h)
{
    int bottom_h = 30;
    int top_h = h - bottom_h;

    char value[16];
    const char *unit;
    ui_format_metric(METRIC_CO2, d, s, value, sizeof(value), &unit);
    bool alarm = co2_alarm(METRIC_CO2, d, s);

    int vscale = 7;
    while (vscale > 2 && (text_w(value, vscale) + 4 + text_w(unit, 2) >
                              CANVAS_W - 2 * MARGIN - ICON_SIZE - 8 ||
                          7 * vscale > top_h - 12)) {
        vscale--;
    }
    int w = ICON_SIZE + 8 + text_w(value, vscale) + 4 + text_w(unit, 2);
    int x = (CANVAS_W - w) / 2;
    int y = (top_h - 7 * vscale) / 2;
    if (alarm) {
        canvas_fill_rect(fb, x - 6, y - 5, w + 12, 7 * vscale + 10, true);
    }
    canvas_draw_xbm(fb, x, y + (7 * vscale - ICON_SIZE) / 2,
                    ICON_SEEDLING_16X16, ICON_SIZE, ICON_SIZE, !alarm);
    int tx = canvas_text(fb, x + ICON_SIZE + 8, y, value, vscale, !alarm);
    canvas_text(fb, tx - vscale + 4, y + 7 * vscale - 14, unit, 2, !alarm);

    /* Bottom strip: time + other enabled metrics, equally spaced. */
    canvas_fill_rect(fb, MARGIN, top_h, CANVAS_W - 2 * MARGIN, 1, true);
    metric_id_t ids[METRIC_COUNT];
    int n_all = enabled_metrics(s, ids);
    metric_id_t others[METRIC_COUNT];
    int n = 0;
    for (int i = 0; i < n_all; i++) {
        if (ids[i] != METRIC_CO2) {
            others[n++] = ids[i];
        }
    }

    int cells = n + 1;
    int cell_w = CANVAS_W / cells;
    int cy = top_h + (bottom_h - ICON_SIZE) / 2 + 1;

    char time_str[8];
    format_time(time_str, sizeof(time_str), s, now, valid);
    text_centered(fb, time_str, 0, cell_w, cy + 1, 2, true);

    for (int i = 0; i < n; i++) {
        char v[16];
        const char *u;
        ui_format_metric(others[i], d, s, v, sizeof(v), &u);
        int uscale = 1;
        if (metric_row_width(v, u, 2, 1) > cell_w - 4) {
            uscale = 0;
        }
        int rw = metric_row_width(v, u, 2, uscale);
        draw_metric_row(fb, (i + 1) * cell_w + (cell_w - rw) / 2, cy,
                        others[i], d, s, 2, uscale);
    }
}

/* ===================================================== */
/* Notifications                                         */
/* ===================================================== */

/* Height of the bottom strip needed for the text (0 = no strip). */
static int notice_strip(const ui_notice_t *notice, char lines[2][UI_LINE_MAX],
                        int *nlines, int *scale)
{
    if (notice == NULL || notice->text == NULL || notice->text[0] == '\0' ||
        notice->fullscreen) {
        return 0;
    }

    bool trunc;
    int max1 = (CANVAS_W - 2 * MARGIN) / (CANVAS_CHAR_W * 2);
    *nlines = ui_wrap(notice->text, max1, 1, lines, &trunc);
    if (!trunc) {
        *scale = 2;
        return 7 * 2 + 8;
    }

    int max2 = (CANVAS_W - 2 * MARGIN) / CANVAS_CHAR_W;
    *nlines = ui_wrap(notice->text, max2, 2, lines, &trunc);
    *scale = 1;
    return *nlines == 1 ? 7 + 8 : 7 + 3 + 7 + 8;
}

static void draw_notice_strip(uint8_t *fb, int strip_h,
                              char lines[2][UI_LINE_MAX], int nlines,
                              int scale)
{
    int y0 = CANVAS_H - strip_h;
    canvas_fill_rect(fb, 0, y0, CANVAS_W, strip_h, true);
    int y = y0 + 4;
    for (int i = 0; i < nlines; i++) {
        text_centered(fb, lines[i], 0, CANVAS_W, y, scale, false);
        y += 10 * scale;
    }
}

static void draw_notice_fullscreen(uint8_t *fb, const char *text)
{
    char lines[10][UI_LINE_MAX];
    int n = 0;
    int scale = 4;
    bool trunc = true;

    canvas_fill_rect(fb, 2, 2, CANVAS_W - 4, CANVAS_H - 4, true);
    canvas_fill_rect(fb, 6, 6, CANVAS_W - 12, CANVAS_H - 12, false);

    int inner_w = CANVAS_W - 24;
    int inner_h = CANVAS_H - 24;
    for (; scale >= 1; scale--) {
        int max_chars = (inner_w + scale) / (CANVAS_CHAR_W * scale);
        int max_lines = (inner_h + 3 * scale) / (10 * scale);
        if (max_lines > 10) {
            max_lines = 10;
        }
        n = ui_wrap(text, max_chars, max_lines, lines, &trunc);
        if (!trunc || scale == 1) {
            break;
        }
    }
    if (scale < 1) {
        scale = 1;
    }

    int block = n * 7 * scale + (n - 1) * 3 * scale;
    int y = (CANVAS_H - block) / 2;
    for (int i = 0; i < n; i++) {
        text_centered(fb, lines[i], 0, CANVAS_W, y, scale, true);
        y += 10 * scale;
    }
}

/* ===================================================== */

static void finish(uint8_t *fb, const settings_t *s)
{
    if (s->display.invert) {
        canvas_invert(fb);
    }
    if (s->display.rotate180) {
        canvas_rotate180(fb);
    }
}

void ui_render_clock(uint8_t *fb, const settings_t *s, const struct tm *now,
                     bool time_valid, const sensor_data_t *data,
                     const ui_notice_t *notice)
{
    canvas_clear(fb, true);

    if (notice != NULL && notice->fullscreen && notice->text != NULL &&
        notice->text[0] != '\0') {
        draw_notice_fullscreen(fb, notice->text);
        finish(fb, s);
        return;
    }

    char lines[2][UI_LINE_MAX];
    int nlines = 0;
    int nscale = 1;
    int strip_h = notice_strip(notice, lines, &nlines, &nscale);
    int h = CANVAS_H - strip_h;

    metric_id_t ids[METRIC_COUNT];
    int n = enabled_metrics(s, ids);
    layout_t layout = s->display.layout;
    if (layout == LAYOUT_SENSORS && n == 0) {
        layout = LAYOUT_CLOCK;
    }
    if (layout == LAYOUT_CLASSIC && n == 0) {
        layout = LAYOUT_CLOCK;
    }

    switch (layout) {
    case LAYOUT_CLOCK:
        draw_clock_block(fb, s, now, time_valid, 0, CANVAS_W, 0, h, 8);
        break;
    case LAYOUT_SENSORS:
        draw_metric_grid(fb, s, data, ids, n, 0, h);
        break;
    case LAYOUT_CO2:
        draw_co2_layout(fb, s, now, time_valid, data, h);
        break;
    case LAYOUT_CLASSIC:
    default:
        draw_clock_block(fb, s, now, time_valid, 0, CLASSIC_SPLIT, 0, h, 5);
        draw_metric_column(fb, s, data, ids, n, CLASSIC_SPLIT,
                           CANVAS_W - CLASSIC_SPLIT, 0, h);
        break;
    }

    if (strip_h > 0) {
        draw_notice_strip(fb, strip_h, lines, nlines, nscale);
    }
    finish(fb, s);
}

/* ===================================================== */

void ui_wifi_qr_payload(char *buf, size_t size, const char *ssid,
                        const char *password)
{
    size_t o = 0;
    const char *parts[] = { "WIFI:T:WPA;S:", ssid, ";P:", password, ";;" };

    for (int i = 0; i < 5 && o + 1 < size; i++) {
        bool escape = (i == 1 || i == 3);
        for (const char *p = parts[i]; *p && o + 1 < size; p++) {
            if (escape && strchr("\\;,:\"", *p) != NULL) {
                if (o + 2 >= size) {
                    break;
                }
                buf[o++] = '\\';
            }
            buf[o++] = *p;
        }
    }
    buf[o] = '\0';
}

static void draw_qr(uint8_t *fb, const char *payload, int x, int y,
                    int max_px, int *out_w)
{
    uint8_t qr[qrcodegen_BUFFER_LEN_FOR_VERSION(6)];
    uint8_t tmp[qrcodegen_BUFFER_LEN_FOR_VERSION(6)];
    *out_w = 0;

    if (!qrcodegen_encodeText(payload, tmp, qr, qrcodegen_Ecc_LOW,
                              qrcodegen_VERSION_MIN, 6, qrcodegen_Mask_AUTO,
                              true)) {
        return;
    }

    int size = qrcodegen_getSize(qr);
    int scale = max_px / size;
    if (scale < 1) {
        scale = 1;
    }
    int px = size * scale;
    int y0 = y + (max_px - px) / 2;
    for (int j = 0; j < size; j++) {
        for (int i = 0; i < size; i++) {
            if (qrcodegen_getModule(qr, i, j)) {
                canvas_fill_rect(fb, x + i * scale, y0 + j * scale, scale,
                                 scale, true);
            }
        }
    }
    *out_w = px;
}

void ui_render_setup(uint8_t *fb, const settings_t *s, const char *ap_ssid,
                     const char *ap_password, const char *ap_ip,
                     const char *failed_ssid)
{
    const setup_strings_t *t = &SETUP_STRINGS[s->display.language];
    canvas_clear(fb, true);

    char payload[160];
    ui_wifi_qr_payload(payload, sizeof(payload), ap_ssid, ap_password);
    int qr_w;
    draw_qr(fb, payload, 6, 6, CANVAS_H - 12, &qr_w);

    int x = 6 + qr_w + 8;
    int w = CANVAS_W - x; /* "EinkClock-XXXX" at x2 needs exactly 166 px */

    char title[96];
    if (failed_ssid != NULL && failed_ssid[0] != '\0') {
        snprintf(title, sizeof(title), "%s%s", t->title_failed, failed_ssid);
    } else {
        snprintf(title, sizeof(title), "%s", t->title);
    }
    int max_chars = w / CANVAS_CHAR_W;
    if (canvas_strlen(title) > max_chars) {
        title[canvas_utf8_prefix(title, max_chars - 1)] = '\0';
        strcat(title, "…");
    }
    canvas_text(fb, x, 6, title, 1, true);
    canvas_fill_rect(fb, x, 17, w, 1, true);

    struct {
        const char *label;
        const char *value;
    } rows[] = {
        { t->network, ap_ssid },
        { t->password, ap_password },
        { t->open, ap_ip },
    };
    int y = 24;
    for (int i = 0; i < 3; i++) {
        canvas_text(fb, x, y, rows[i].label, 1, true);
        int vs = text_w(rows[i].value, 2) <= w ? 2 : 1;
        canvas_text(fb, x, y + 10, rows[i].value, vs, true);
        y += 34;
    }

    finish(fb, s);
}

void ui_render_message(uint8_t *fb, const settings_t *s, const char *title,
                       const char *subtitle, int progress)
{
    canvas_clear(fb, true);

    int tscale = 3;
    while (tscale > 1 && text_w(title, tscale) > CANVAS_W - 2 * MARGIN) {
        tscale--;
    }
    int block = 7 * tscale + 14 + (progress >= 0 ? 22 : 0);
    int y = (CANVAS_H - block) / 2;

    canvas_draw_xbm(fb, (CANVAS_W - ICON_SIZE) / 2, y - 26,
                    ICON_SEEDLING_16X16, ICON_SIZE, ICON_SIZE, true);
    text_centered(fb, title, 0, CANVAS_W, y, tscale, true);
    y += 7 * tscale + 10;
    if (subtitle != NULL) {
        text_centered(fb, subtitle, 0, CANVAS_W, y, 1, true);
    }
    if (progress >= 0) {
        if (progress > 100) {
            progress = 100;
        }
        int bw = 200;
        int bx = (CANVAS_W - bw) / 2;
        int by = y + 16;
        canvas_draw_rect(fb, bx, by, bw, 10, true);
        canvas_fill_rect(fb, bx + 2, by + 2, (bw - 4) * progress / 100, 6,
                         true);
    }

    finish(fb, s);
}
