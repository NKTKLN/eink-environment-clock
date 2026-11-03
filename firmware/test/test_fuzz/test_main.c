/*
 * Randomised robustness tests for everything that parses outside input.
 * Deterministic (fixed seed) so failures reproduce. Buffers are allocated
 * with the exact input size so AddressSanitizer catches any over-read:
 * run with `pio test -e native-asan -f test_fuzz`.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unity.h>

#include "dns_packet.h"
#include "epd_canvas.h"
#include "ha_command.h"
#include "log_ring.h"
#include "notify.h"
#include "release.h"
#include "settings.h"
#include "ui.h"

#ifndef FUZZ_ITERATIONS
#define FUZZ_ITERATIONS 4000
#endif

static uint64_t rng = 0x9E3779B97F4A7C15ull;
static uint32_t rnd(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 7;
    rng ^= rng << 17;
    return (uint32_t)rng;
}
static uint32_t rnd_n(uint32_t n) { return n ? rnd() % n : 0; }

void setUp(void) {}
void tearDown(void) {}

/* Random bytes biased towards the characters parsers care about. */
static void random_bytes(char *buf, size_t len)
{
    static const char interesting[] = "{}[]\":,\\0123456789-+.eE /#truefalsnul\xd0\xb6\xe2\x80\xa6\xf0\x9f\x98\x80";
    for (size_t i = 0; i < len; i++) {
        buf[i] = rnd_n(3) ? interesting[rnd_n(sizeof(interesting) - 1)]
                          : (char)rnd();
    }
}

/* Flip, insert, delete or truncate bytes of a valid document. */
static char *mutate(const char *src, size_t *out_len)
{
    size_t len = strlen(src);
    size_t cap = len + 64;
    char *b = malloc(cap + 1);
    memcpy(b, src, len);
    int edits = 1 + (int)rnd_n(8);
    for (int e = 0; e < edits && len > 0; e++) {
        size_t at = rnd_n((uint32_t)len);
        switch (rnd_n(4)) {
        case 0: b[at] = (char)rnd(); break;
        case 1: if (len < cap) { memmove(b + at + 1, b + at, len - at); random_bytes(b + at, 1); len++; } break;
        case 2: memmove(b + at, b + at + 1, len - at - 1); len--; break;
        default: len = at; break;
        }
    }
    b[len] = '\0';
    *out_len = len;
    return b;
}

static int sign(int x) { return (x > 0) - (x < 0); }

/* ===================================================== */

static void check_settings_invariants(const settings_t *s)
{
    TEST_ASSERT_TRUE((unsigned)s->display.language < LANG_COUNT);
    TEST_ASSERT_TRUE((unsigned)s->display.layout < LAYOUT_COUNT);
    TEST_ASSERT_TRUE((unsigned)s->display.date_format < DATE_FORMAT_COUNT);
    TEST_ASSERT_TRUE((unsigned)s->display.temp_unit < TEMP_UNIT_COUNT);
    TEST_ASSERT_TRUE((unsigned)s->display.pressure_unit < PRESSURE_UNIT_COUNT);
    bool seen[METRIC_COUNT] = { false };
    for (int i = 0; i < METRIC_COUNT; i++) {
        int id = (int)s->display.metrics[i].id;
        TEST_ASSERT_TRUE(id >= 0 && id < METRIC_COUNT);
        TEST_ASSERT_FALSE(seen[id]);
        seen[id] = true;
    }
    TEST_ASSERT_TRUE(s->sensors.co2_threshold_ppm >= CO2_THRESHOLD_MIN &&
                     s->sensors.co2_threshold_ppm <= CO2_THRESHOLD_MAX);
    TEST_ASSERT_TRUE(s->sensors.temp_offset_c >= -TEMP_OFFSET_LIMIT &&
                     s->sensors.temp_offset_c <= TEMP_OFFSET_LIMIT);
    TEST_ASSERT_TRUE(s->display.connect_notice_s >= -1 &&
                     s->display.connect_notice_s <= CONNECT_NOTICE_MAX);
    TEST_ASSERT_TRUE(s->mqtt.port > 0);
    TEST_ASSERT_TRUE(memchr(s->name, 0, sizeof(s->name)) != NULL);
    TEST_ASSERT_TRUE(memchr(s->wifi.ssid, 0, sizeof(s->wifi.ssid)) != NULL);
    TEST_ASSERT_TRUE(memchr(s->display.tz_posix, 0, sizeof(s->display.tz_posix)) != NULL);
    TEST_ASSERT_NULL(strpbrk(s->mqtt.prefix, "#+"));
    TEST_ASSERT_TRUE(s->name[0] != '\0');
    TEST_ASSERT_TRUE(s->mqtt.discovery_prefix[0] != '\0');
}

static void test_fuzz_settings_json(void)
{
    settings_t base;
    settings_defaults(&base);
    strcpy(base.wifi.ssid, "home");
    cJSON *j = settings_to_json(&base, true);
    char *valid = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);

    for (int i = 0; i < FUZZ_ITERATIONS; i++) {
        size_t len;
        char *doc = mutate(valid, &len);
        cJSON *parsed = cJSON_ParseWithLength(doc, len);
        settings_t s;
        settings_defaults(&s);
        settings_apply_json(&s, parsed);
        check_settings_invariants(&s);

        /* whatever was accepted must survive a save/load cycle */
        cJSON *out = settings_to_json(&s, true);
        settings_t back;
        settings_defaults(&back);
        settings_apply_json(&back, out);
        TEST_ASSERT_EQUAL(s.display.layout, back.display.layout);
        TEST_ASSERT_EQUAL_STRING(s.wifi.password, back.wifi.password);
        TEST_ASSERT_EQUAL(s.sensors.co2_threshold_ppm, back.sensors.co2_threshold_ppm);
        cJSON_Delete(out);
        cJSON_Delete(parsed);
        free(doc);
    }
    cJSON_free(valid);
}

/* Wrong types everywhere must be ignored, not crash or corrupt. */
static void test_fuzz_settings_wrong_types(void)
{
    const char *values[] = { "null", "true", "1e309", "-1", "\"x\"", "[]", "{}", "[1,2]", "99999999999" };
    const char *keys[] = { "name", "wifi", "display", "sensors", "mqtt", "update" };
    const char *inner[] = { "layout", "metrics", "tz_posix", "port", "co2_threshold_ppm",
                            "temp_offset_c", "connect_notice_s", "language", "password" };
    char doc[256];
    for (int i = 0; i < FUZZ_ITERATIONS; i++) {
        snprintf(doc, sizeof(doc), "{\"%s\":{\"%s\":%s},\"%s\":%s}",
                 keys[rnd_n(6)], inner[rnd_n(9)], values[rnd_n(9)],
                 keys[rnd_n(6)], values[rnd_n(9)]);
        cJSON *p = cJSON_Parse(doc);
        settings_t s;
        settings_defaults(&s);
        settings_apply_json(&s, p);
        check_settings_invariants(&s);
        cJSON_Delete(p);
    }
}

static void test_fuzz_notify_parse(void)
{
    for (int i = 0; i < FUZZ_ITERATIONS; i++) {
        size_t len = rnd_n(600);
        char *buf = malloc(len ? len : 1); /* exact size, no terminator */
        if (rnd_n(2) && len > 2) {
            static const char *docs[] = { "{\"text\":\"", "{\"ttl\":", "{\"fullscreen\":", "{" };
            const char *d = docs[rnd_n(4)];
            size_t dl = strlen(d) < len ? strlen(d) : len;
            memcpy(buf, d, dl);
            random_bytes(buf + dl, len - dl);
        } else {
            random_bytes(buf, len);
        }
        char text[NOTIFY_TEXT_MAX];
        int ttl;
        bool fs;
        notify_parse(buf, (int)len, text, sizeof(text), &ttl, &fs);
        TEST_ASSERT_TRUE(strlen(text) < sizeof(text));
        TEST_ASSERT_TRUE(ttl <= NOTIFY_TTL_MAX);

        notify_t n;
        notify_init(&n);
        notify_set(&n, text, ttl, fs, 1000);
        TEST_ASSERT_TRUE(strlen(n.text) < NOTIFY_TEXT_MAX);
        free(buf);
    }
}

static void test_fuzz_ha_command(void)
{
    const char *suffixes[] = { "notify", "notification/set", "command", "layout/set",
                               "co2_threshold/set", "state", "", "x" };
    for (int i = 0; i < FUZZ_ITERATIONS; i++) {
        char topic[96];
        snprintf(topic, sizeof(topic), "%s/%s",
                 rnd_n(5) ? "eink-clock/a1b2c3" : "homeassistant", suffixes[rnd_n(8)]);
        size_t tlen = strlen(topic);
        if (rnd_n(4) == 0 && tlen > 0) {
            tlen = rnd_n((uint32_t)tlen + 1);
        }
        char *t = malloc(tlen ? tlen : 1);
        memcpy(t, topic, tlen);

        size_t dlen = rnd_n(400);
        char *d = malloc(dlen ? dlen : 1);
        random_bytes(d, dlen);

        ha_command_t c;
        ha_parse_command("eink-clock/a1b2c3", "homeassistant", t, (int)tlen, d,
                         (int)dlen, &c);
        TEST_ASSERT_TRUE(c.action <= HA_ACT_HA_ONLINE);
        TEST_ASSERT_TRUE(strlen(c.text) < sizeof(c.text));
        if (c.action == HA_ACT_THRESHOLD) {
            TEST_ASSERT_TRUE(c.threshold >= CO2_THRESHOLD_MIN && c.threshold <= CO2_THRESHOLD_MAX);
        }
        if (c.action == HA_ACT_LAYOUT) {
            TEST_ASSERT_TRUE((unsigned)c.layout < LAYOUT_COUNT);
        }
        free(t);
        free(d);
    }
}

static void test_fuzz_dns(void)
{
    for (int i = 0; i < FUZZ_ITERATIONS * 4; i++) {
        const int cap = 512;
        uint8_t *buf = malloc(cap);
        int len = (int)rnd_n(cap + 1);
        for (int k = 0; k < len; k++) {
            buf[k] = (uint8_t)(rnd_n(3) ? rnd_n(64) : rnd());
        }
        if (len >= 12 && rnd_n(2)) { /* make it look like a query */
            buf[2] = 0x01; buf[3] = 0; buf[4] = 0; buf[5] = 1;
        }
        int out = dns_build_reply(buf, len, cap, 0x0104A8C0);
        TEST_ASSERT_TRUE(out == -1 || (out >= 12 && out <= cap));
        free(buf);
    }
}

static void test_fuzz_release(void)
{
    const char *valid =
        "{\"tag_name\":\"v1.3.0\",\"html_url\":\"https://x\",\"prerelease\":false,"
        "\"body\":\"notes\",\"assets\":[{\"name\":\"firmware.bin\","
        "\"browser_download_url\":\"https://github.com/o/r/releases/download/v1.3.0/firmware.bin\",\"size\":1240000}]}";
    for (int i = 0; i < FUZZ_ITERATIONS; i++) {
        size_t len;
        char *doc = mutate(valid, &len);
        char *exact = malloc(len ? len : 1);
        memcpy(exact, doc, len);
        release_t r;
        if (release_parse(exact, len, "firmware.bin", &r)) {
            TEST_ASSERT_TRUE(strlen(r.url) > 0 && strlen(r.url) < sizeof(r.url));
            TEST_ASSERT_TRUE(strlen(r.version) < sizeof(r.version));
        }
        free(exact);
        free(doc);
    }
}

static void test_fuzz_version_compare(void)
{
    char a[24], b[24];
    for (int i = 0; i < FUZZ_ITERATIONS * 4; i++) {
        const char *pre[] = { "", "-rc.1", "-rc.10", "-alpha", "-1", "+build", "-rc.1+b", "x", "-" };
        snprintf(a, sizeof(a), "%s%u.%u.%u%s", rnd_n(4) ? "" : "v", rnd_n(3), rnd_n(12), rnd_n(3), pre[rnd_n(9)]);
        snprintf(b, sizeof(b), "%u.%u.%u%s", rnd_n(3), rnd_n(12), rnd_n(3), pre[rnd_n(9)]);
        TEST_ASSERT_EQUAL(sign(version_compare(a, b)), -sign(version_compare(b, a)));
        TEST_ASSERT_EQUAL(0, version_compare(a, a));
    }
}

static void test_fuzz_utf8_and_wrap(void)
{
    char lines[10][UI_LINE_MAX];
    for (int i = 0; i < FUZZ_ITERATIONS; i++) {
        size_t len = rnd_n(400);
        char *s = malloc(len + 1);
        random_bytes(s, len);
        for (size_t k = 0; k < len; k++) {
            if (s[k] == 0) s[k] = ' ';
        }
        s[len] = '\0';

        int n = canvas_strlen(s);
        TEST_ASSERT_TRUE(n >= 0 && (size_t)n <= len);
        TEST_ASSERT_TRUE(canvas_utf8_prefix(s, (int)rnd_n(500)) <= len);

        int max_chars = 2 + (int)rnd_n(60);
        int max_lines = 1 + (int)rnd_n(10);
        bool trunc;
        int got = ui_wrap(s, max_chars, max_lines, lines, &trunc);
        TEST_ASSERT_TRUE(got >= 0 && got <= max_lines);
        for (int l = 0; l < got; l++) {
            TEST_ASSERT_TRUE(strlen(lines[l]) < UI_LINE_MAX);
            TEST_ASSERT_TRUE(canvas_strlen(lines[l]) <= max_chars);
        }
        free(s);
    }
}

/* Random settings, readings and messages must render without touching
 * memory outside the framebuffer. */
static void test_fuzz_render(void)
{
    uint8_t *fb = malloc(EPD_FRAMEBUFFER_SIZE);
    for (int i = 0; i < FUZZ_ITERATIONS / 4; i++) {
        settings_t s;
        settings_defaults(&s);
        s.display.layout = (layout_t)rnd_n(LAYOUT_COUNT);
        s.display.language = (language_t)rnd_n(LANG_COUNT);
        s.display.date_format = (date_format_t)rnd_n(DATE_FORMAT_COUNT);
        s.display.temp_unit = (temp_unit_t)rnd_n(TEMP_UNIT_COUNT);
        s.display.pressure_unit = (pressure_unit_t)rnd_n(PRESSURE_UNIT_COUNT);
        s.display.hour24 = rnd_n(2);
        s.display.show_date = rnd_n(2);
        s.display.invert = rnd_n(2);
        s.display.rotate180 = rnd_n(2);
        for (int m = 0; m < METRIC_COUNT; m++) {
            s.display.metrics[m].enabled = rnd_n(2);
        }
        const float extremes[] = { NAN, -40.0f, 0.0f, 85.0f, 99999.0f, -99999.0f, 1013.25f };
        sensor_data_t d = { extremes[rnd_n(7)], extremes[rnd_n(7)], extremes[rnd_n(7)],
                            (int)rnd_n(3) == 0 ? -1 : (int)rnd_n(100000) };
        struct tm now = { .tm_year = 126, .tm_mon = (int)rnd_n(12), .tm_mday = 1 + (int)rnd_n(31),
                          .tm_hour = (int)rnd_n(24), .tm_min = (int)rnd_n(60), .tm_wday = (int)rnd_n(7) };

        size_t len = rnd_n(NOTIFY_TEXT_MAX);
        char *text = malloc(len + 1);
        random_bytes(text, len);
        for (size_t k = 0; k < len; k++) if (text[k] == 0) text[k] = 'x';
        text[len] = '\0';
        ui_notice_t notice = { len ? text : NULL, rnd_n(2) };

        ui_render_clock(fb, &s, &now, rnd_n(4) != 0, &d, rnd_n(3) ? &notice : NULL);
        if (rnd_n(4) == 0) {
            ui_render_setup(fb, &s, "EinkClock-ABCD", "12345678", "192.168.4.1",
                            rnd_n(2) ? text : NULL);
        }
        free(text);
    }
    free(fb);
}

/* The ring buffer against a trivial reference model. */
static void test_fuzz_log_ring(void)
{
    char buf[97];
    log_ring_t r;
    log_ring_init(&r, buf, sizeof(buf));
    char *model = malloc(1 << 20);
    size_t model_len = 0;
    uint32_t pos = 0;
    for (int i = 0; i < FUZZ_ITERATIONS * 4 && model_len < (1 << 20) - 300; i++) {
        if (rnd_n(2)) {
            char data[300];
            size_t n = rnd_n(sizeof(data));
            random_bytes(data, n);
            log_ring_write(&r, data, n);
            memcpy(model + model_len, data, n);
            model_len += n;
        } else {
            char out[160];
            int dropped;
            uint32_t before = pos;
            size_t n = log_ring_read(&r, &pos, out, rnd_n(sizeof(out)), &dropped);
            uint32_t start = pos - (uint32_t)n;
            TEST_ASSERT_TRUE(start >= before || dropped);
            if (n > 0) {
                TEST_ASSERT_EQUAL_MEMORY(model + start, out, n);
            }
            TEST_ASSERT_TRUE(pos <= model_len);
            TEST_ASSERT_TRUE(model_len - start <= sizeof(buf) || n == 0);
        }
    }
    free(model);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_fuzz_settings_json);
    RUN_TEST(test_fuzz_settings_wrong_types);
    RUN_TEST(test_fuzz_notify_parse);
    RUN_TEST(test_fuzz_ha_command);
    RUN_TEST(test_fuzz_dns);
    RUN_TEST(test_fuzz_release);
    RUN_TEST(test_fuzz_version_compare);
    RUN_TEST(test_fuzz_utf8_and_wrap);
    RUN_TEST(test_fuzz_render);
    RUN_TEST(test_fuzz_log_ring);
    return UNITY_END();
}
