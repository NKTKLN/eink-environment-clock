#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unity.h>

#include "epd_canvas.h"
#include "ui.h"

static uint8_t fb[EPD_FRAMEBUFFER_SIZE];
static settings_t s;
static struct tm now;
static sensor_data_t data;

void setUp(void)
{
    settings_defaults(&s);
    memset(&now, 0, sizeof(now));
    now.tm_year = 126;
    now.tm_mon = 10;
    now.tm_mday = 3;
    now.tm_wday = 2;
    now.tm_hour = 14;
    now.tm_min = 37;
    data = (sensor_data_t){ 23.4f, 41.0f, 1008.0f, 812 };
}
void tearDown(void) {}

/* ---- helpers ---- */

typedef struct {
    int x0, y0, x1, y1; /* inclusive, x0 > x1 when empty */
    int count;
} bbox_t;

static bbox_t ink_box(int x0, int y0, int x1, int y1)
{
    bbox_t b = { CANVAS_W, CANVAS_H, -1, -1, 0 };
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            if (canvas_get_pixel(fb, x, y)) {
                if (x < b.x0) b.x0 = x;
                if (y < b.y0) b.y0 = y;
                if (x > b.x1) b.x1 = x;
                if (y > b.y1) b.y1 = y;
                b.count++;
            }
        }
    }
    return b;
}

static bbox_t whole(void) { return ink_box(0, 0, CANVAS_W - 1, CANVAS_H - 1); }

/* PREVIEW_DIR=... pio test -e native writes every rendered screen as PBM. */
static void preview(const char *name)
{
    const char *dir = getenv("PREVIEW_DIR");
    if (dir == NULL) {
        return;
    }
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.pbm", dir, name);
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        return;
    }
    fprintf(f, "P4\n%d %d\n", CANVAS_W, CANVAS_H);
    for (int y = 0; y < CANVAS_H; y++) {
        for (int x = 0; x < CANVAS_W; x += 8) {
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) {
                if (canvas_get_pixel(fb, x + b, y)) {
                    byte |= (uint8_t)(0x80 >> b);
                }
            }
            fputc(byte, f);
        }
    }
    fclose(f);
}

static void render(const ui_notice_t *n, const char *name)
{
    ui_render_clock(fb, &s, &now, true, &data, n);
    preview(name);
}

static void enable_only(int mask)
{
    for (int i = 0; i < METRIC_COUNT; i++) {
        s.display.metrics[i].enabled = (mask >> s.display.metrics[i].id) & 1;
    }
}

/* ---- wrap ---- */

static void test_wrap_basic(void)
{
    char l[4][UI_LINE_MAX];
    bool t;
    int n = ui_wrap("open the window please", 10, 4, l, &t);
    TEST_ASSERT_EQUAL(3, n);
    TEST_ASSERT_FALSE(t);
    TEST_ASSERT_EQUAL_STRING("open the", l[0]);
    TEST_ASSERT_EQUAL_STRING("window", l[1]);
    TEST_ASSERT_EQUAL_STRING("please", l[2]);
}

static void test_wrap_truncates_with_ellipsis(void)
{
    char l[2][UI_LINE_MAX];
    bool t;
    int n = ui_wrap("один два три четыре пять шесть", 9, 2, l, &t);
    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_TRUE(t);
    TEST_ASSERT_EQUAL_STRING("один два", l[0]);
    TEST_ASSERT_EQUAL_STRING("три…", l[1]);
    TEST_ASSERT_TRUE(canvas_strlen(l[1]) <= 9);
}

static void test_wrap_long_word_and_newline(void)
{
    char l[4][UI_LINE_MAX];
    bool t;
    int n = ui_wrap("abcdefghij\nxy", 4, 4, l, &t);
    TEST_ASSERT_EQUAL(4, n);
    TEST_ASSERT_EQUAL_STRING("abcd", l[0]);
    TEST_ASSERT_EQUAL_STRING("efgh", l[1]);
    TEST_ASSERT_EQUAL_STRING("ij", l[2]);
    TEST_ASSERT_EQUAL_STRING("xy", l[3]);

    n = ui_wrap("", 4, 4, l, &t);
    TEST_ASSERT_EQUAL(0, n);
    TEST_ASSERT_FALSE(t);
}

static void test_wrap_exact_fit_not_truncated(void)
{
    char l[1][UI_LINE_MAX];
    bool t;
    int n = ui_wrap("abcd efgh", 9, 1, l, &t);
    TEST_ASSERT_EQUAL(1, n);
    TEST_ASSERT_FALSE(t);
}

/* ---- formatting ---- */

static void test_metric_units(void)
{
    char b[16];
    const char *u;
    TEST_ASSERT_TRUE(ui_format_metric(METRIC_TEMPERATURE, &data, &s, b, sizeof(b), &u));
    TEST_ASSERT_EQUAL_STRING("23.4", b);
    TEST_ASSERT_EQUAL_STRING("°C", u);

    s.display.temp_unit = TEMP_UNIT_F;
    ui_format_metric(METRIC_TEMPERATURE, &data, &s, b, sizeof(b), &u);
    TEST_ASSERT_EQUAL_STRING("74.1", b);

    s.display.pressure_unit = PRESSURE_MMHG;
    s.display.language = LANG_RU;
    ui_format_metric(METRIC_PRESSURE, &data, &s, b, sizeof(b), &u);
    TEST_ASSERT_EQUAL_STRING("756", b);
    TEST_ASSERT_EQUAL_STRING("мм", u);

    data.co2_ppm = -1;
    TEST_ASSERT_FALSE(ui_format_metric(METRIC_CO2, &data, &s, b, sizeof(b), &u));
    TEST_ASSERT_EQUAL_STRING("---", b);

    data.temperature_c = -0.04f;
    s.display.temp_unit = TEMP_UNIT_C;
    ui_format_metric(METRIC_TEMPERATURE, &data, &s, b, sizeof(b), &u);
    TEST_ASSERT_EQUAL_STRING("0.0", b);
}

static void test_wifi_qr_escaping(void)
{
    char b[128];
    ui_wifi_qr_payload(b, sizeof(b), "My;Net", "p:a\\ss");
    TEST_ASSERT_EQUAL_STRING("WIFI:T:WPA;S:My\\;Net;P:p\\:a\\\\ss;;", b);
}

/* ---- layouts ---- */

static void test_classic_default_matches_original_positions(void)
{
    render(NULL, "classic_4");
    bbox_t left = ink_box(0, 0, 184, CANVAS_H - 1);
    bbox_t right = ink_box(185, 0, CANVAS_W - 1, CANVAS_H - 1);
    TEST_ASSERT_TRUE(left.count > 0);
    TEST_ASSERT_TRUE(right.count > 0);
    /* time block vertically centered around the middle */
    TEST_ASSERT_INT_WITHIN(6, CANVAS_H / 2, (left.y0 + left.y1) / 2);
    /* original firmware: first metric row at y=22, last ends at 110 */
    TEST_ASSERT_INT_WITHIN(2, 22, right.y0);
    TEST_ASSERT_INT_WITHIN(2, 109, right.y1);
}

static void test_metrics_recentered_when_disabled(void)
{
    enable_only((1 << METRIC_TEMPERATURE) | (1 << METRIC_CO2));
    render(NULL, "classic_2");
    bbox_t right = ink_box(185, 0, CANVAS_W - 1, CANVAS_H - 1);
    TEST_ASSERT_INT_WITHIN(3, CANVAS_H / 2, (right.y0 + right.y1) / 2);

    enable_only(1 << METRIC_CO2);
    render(NULL, "classic_1");
    right = ink_box(185, 0, CANVAS_W - 1, CANVAS_H - 1);
    TEST_ASSERT_INT_WITHIN(3, CANVAS_H / 2, (right.y0 + right.y1) / 2);
    TEST_ASSERT_INT_WITHIN(4, 185 + (CANVAS_W - 185) / 2, (right.x0 + right.x1) / 2);
}

static void test_no_metrics_gives_full_width_clock(void)
{
    enable_only(0);
    render(NULL, "classic_0");
    bbox_t b = whole();
    TEST_ASSERT_INT_WITHIN(4, CANVAS_W / 2, (b.x0 + b.x1) / 2);
    TEST_ASSERT_TRUE(b.x1 - b.x0 > 200); /* much bigger than the 150px time */
}

static void test_other_layouts_render(void)
{
    s.display.layout = LAYOUT_CLOCK;
    render(NULL, "layout_clock");
    TEST_ASSERT_INT_WITHIN(4, CANVAS_W / 2, (whole().x0 + whole().x1) / 2);

    s.display.layout = LAYOUT_SENSORS;
    render(NULL, "layout_sensors_4");
    enable_only(0x7);
    render(NULL, "layout_sensors_3");
    enable_only(0x1);
    render(NULL, "layout_sensors_1");
    TEST_ASSERT_TRUE(whole().count > 0);

    enable_only(0xF);
    s.display.layout = LAYOUT_CO2;
    render(NULL, "layout_co2");
    data.co2_ppm = 1450;
    render(NULL, "layout_co2_alarm");
    /* alarm plate: the area around the big value is mostly black */
    bbox_t top = ink_box(40, 10, CANVAS_W - 41, 60);
    TEST_ASSERT_TRUE(top.count > (CANVAS_W - 80) * 50 / 2);

    data.co2_ppm = -1;
    data.temperature_c = NAN;
    render(NULL, "layout_co2_nodata");
    TEST_ASSERT_TRUE(whole().count > 0);
}

static void test_co2_alarm_highlight_classic(void)
{
    data.co2_ppm = 1500;
    render(NULL, "classic_alarm");
    /* last row is CO2; its plate fills the background left of the icon */
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 189, 100));
}

static void test_notification_strip(void)
{
    ui_notice_t n = { "Meeting until 15:00", false };
    render(&n, "notice_short");
    /* strip is black across the full width at the bottom */
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 1, CANVAS_H - 2));
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, CANVAS_W - 2, CANVAS_H - 2));
    /* content above stays clear of the strip */
    bbox_t above = ink_box(0, 0, CANVAS_W - 1, CANVAS_H - 24);
    TEST_ASSERT_TRUE(above.count > 0);
    TEST_ASSERT_FALSE(canvas_get_pixel(fb, 2, CANVAS_H - 23));

    n.text = "Проветрите комнату: CO2 выше 1200 ppm уже 20 минут, "
             "окно в переговорной закрыто, кондиционер выключен, и ещё много текста";
    render(&n, "notice_long_ru");
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 1, CANVAS_H - 2));

    s.display.layout = LAYOUT_SENSORS;
    n.text = "Open the window";
    render(&n, "notice_sensors");

    /* the "saved network lost" strip must fit without truncation */
    s.display.layout = LAYOUT_CLASSIC;
    char strip[128];
    char lines[2][UI_LINE_MAX];
    bool trunc;
    for (int lang = 0; lang < LANG_COUNT; lang++) {
        s.display.language = (language_t)lang;
        snprintf(strip, sizeof(strip), ui_str(&s, UI_STR_SETUP_STRIP),
                 "EinkClock-A1B2", "48213077", "192.168.4.1");
        ui_wrap(strip, (CANVAS_W - 8) / CANVAS_CHAR_W, 2, lines, &trunc);
        TEST_ASSERT_FALSE_MESSAGE(trunc, strip);
        n.text = strip;
        render(&n, lang == LANG_RU ? "notice_setup_strip_ru" : "notice_setup_strip_en");
    }
}

static void test_notification_fullscreen(void)
{
    ui_notice_t n = { "Откройте окно", true };
    render(&n, "notice_fullscreen");
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 3, 3)); /* frame */
    bbox_t in = ink_box(10, 10, CANVAS_W - 11, CANVAS_H - 11);
    TEST_ASSERT_INT_WITHIN(4, CANVAS_W / 2, (in.x0 + in.x1) / 2);
    TEST_ASSERT_INT_WITHIN(4, CANVAS_H / 2, (in.y0 + in.y1) / 2);

    n.text = "A much longer fullscreen message that has to wrap over several "
             "lines and use a smaller font so that every word stays visible";
    render(&n, "notice_fullscreen_long");
}

static void test_invert_and_rotate(void)
{
    uint8_t plain[EPD_FRAMEBUFFER_SIZE];
    render(NULL, "plain");
    memcpy(plain, fb, sizeof(fb));

    s.display.invert = true;
    render(NULL, "inverted");
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 0, 0));
    canvas_invert(fb);
    TEST_ASSERT_EQUAL_MEMORY(plain, fb, sizeof(fb));

    s.display.invert = false;
    s.display.rotate180 = true;
    render(NULL, "rotated");
    canvas_rotate180(fb);
    TEST_ASSERT_EQUAL_MEMORY(plain, fb, sizeof(fb));
}

static void test_languages_and_date_formats(void)
{
    s.display.language = LANG_RU;
    s.display.date_format = DATE_WEEKDAY;
    s.display.hour24 = true;
    render(NULL, "ru_weekday");
    s.display.date_format = DATE_ISO;
    render(NULL, "ru_iso");
    s.display.show_date = false;
    render(NULL, "no_date");
    ui_render_clock(fb, &s, &now, false, &data, NULL);
    preview("time_invalid");
    TEST_ASSERT_TRUE(whole().count > 0);
}

static void test_setup_screen(void)
{
    ui_render_setup(fb, &s, "EinkClock-A1B2", "48213077", "192.168.4.1", NULL);
    preview("setup_en");
    /* QR on the left, roughly square and as tall as the screen allows */
    bbox_t qr = ink_box(0, 0, 125, CANVAS_H - 1);
    TEST_ASSERT_INT_WITHIN(2, qr.x1 - qr.x0, qr.y1 - qr.y0);
    TEST_ASSERT_TRUE(qr.y1 - qr.y0 > 90);
    /* text never runs past the right edge */
    TEST_ASSERT_TRUE(whole().x1 <= CANVAS_W - 1);

    s.display.language = LANG_RU;
    ui_render_setup(fb, &s, "EinkClock-A1B2", "48213077", "192.168.4.1",
                    "My very long home network name");
    preview("setup_ru_failed");
}

static void test_message_screen(void)
{
    ui_render_message(fb, &s, ui_str(&s, UI_STR_UPDATING),
                      ui_str(&s, UI_STR_UPDATE_HINT), 42);
    preview("message_ota");
    TEST_ASSERT_INT_WITHIN(4, CANVAS_W / 2, (whole().x0 + whole().x1) / 2);
    s.display.language = LANG_RU;
    ui_render_message(fb, &s, ui_str(&s, UI_STR_RESETTING),
                      ui_str(&s, UI_STR_RESET_HINT), -1);
    preview("message_reset_ru");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_wrap_basic);
    RUN_TEST(test_wrap_truncates_with_ellipsis);
    RUN_TEST(test_wrap_long_word_and_newline);
    RUN_TEST(test_wrap_exact_fit_not_truncated);
    RUN_TEST(test_metric_units);
    RUN_TEST(test_wifi_qr_escaping);
    RUN_TEST(test_classic_default_matches_original_positions);
    RUN_TEST(test_metrics_recentered_when_disabled);
    RUN_TEST(test_no_metrics_gives_full_width_clock);
    RUN_TEST(test_other_layouts_render);
    RUN_TEST(test_co2_alarm_highlight_classic);
    RUN_TEST(test_notification_strip);
    RUN_TEST(test_notification_fullscreen);
    RUN_TEST(test_invert_and_rotate);
    RUN_TEST(test_languages_and_date_formats);
    RUN_TEST(test_setup_screen);
    RUN_TEST(test_message_screen);
    return UNITY_END();
}
