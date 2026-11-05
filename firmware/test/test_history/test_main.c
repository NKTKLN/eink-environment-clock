#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <cJSON.h>
#include <unity.h>

#include "history.h"

static history_t h;
static const int64_t STEP = HISTORY_INTERVAL_S * 1000LL;

void setUp(void) { history_init(&h); }
void tearDown(void) {}

static void add(float t, float hum, float p, int co2, int64_t ms)
{
    sensor_data_t d = { t, hum, p, co2 };
    history_add(&h, &d, ms);
}

typedef struct {
    char text[64 * 1024];
    size_t len;
    int calls;
} sink_t;

static void sink(void *ctx, const char *text, size_t len)
{
    sink_t *s = ctx;
    TEST_ASSERT_TRUE(s->len + len < sizeof(s->text));
    memcpy(s->text + s->len, text, len);
    s->len += len;
    s->text[s->len] = '\0';
    s->calls++;
}

static cJSON *as_json(int64_t now_ms, sink_t *s)
{
    static history_sample_t buf[HISTORY_LEN];
    size_t n = history_copy(&h, buf, HISTORY_LEN);
    memset(s, 0, sizeof(*s));
    history_write_json(buf, n, history_age_s(&h, now_ms), sink, s);
    cJSON *j = cJSON_Parse(s->text);
    TEST_ASSERT_NOT_NULL_MESSAGE(j, s->text);
    return j;
}

static void test_empty_until_first_bucket_closes(void)
{
    add(20, 40, 1000, 600, 0);
    add(21, 41, 1001, 610, 30000);
    TEST_ASSERT_EQUAL(0, h.count);
    TEST_ASSERT_EQUAL(-1, history_age_s(&h, 30000));

    sink_t s;
    cJSON *j = as_json(30000, &s);
    TEST_ASSERT_EQUAL(HISTORY_INTERVAL_S, cJSON_GetObjectItem(j, "interval_s")->valueint);
    TEST_ASSERT_EQUAL(0, cJSON_GetArraySize(cJSON_GetObjectItem(j, "co2_ppm")));
    cJSON_Delete(j);
}

static void test_bucket_is_averaged(void)
{
    add(20.0f, 40, 1000, 600, 0);
    add(21.0f, 42, 1002, 700, 30000);
    add(22.0f, 44, 1004, 800, STEP); /* closes the first bucket */
    TEST_ASSERT_EQUAL(1, h.count);

    history_sample_t s;
    history_copy(&h, &s, 1);
    TEST_ASSERT_EQUAL(205, s.temp_c10);
    TEST_ASSERT_EQUAL(410, s.hum_pct10);
    TEST_ASSERT_EQUAL(10010, s.press_hpa10);
    TEST_ASSERT_EQUAL(650, s.co2_ppm);
    TEST_ASSERT_EQUAL(5, history_age_s(&h, STEP + 5000));
}

static void test_missing_values_become_null(void)
{
    add(NAN, 40, NAN, -1, 0);
    add(-3.5f, 40, NAN, -1, STEP);

    sink_t s;
    cJSON *j = as_json(STEP, &s);
    TEST_ASSERT_TRUE(cJSON_IsNull(cJSON_GetArrayItem(cJSON_GetObjectItem(j, "temperature_c"), 0)));
    TEST_ASSERT_TRUE(cJSON_IsNull(cJSON_GetArrayItem(cJSON_GetObjectItem(j, "pressure_hpa"), 0)));
    TEST_ASSERT_TRUE(cJSON_IsNull(cJSON_GetArrayItem(cJSON_GetObjectItem(j, "co2_ppm"), 0)));
    TEST_ASSERT_EQUAL_INT(400, (int)lround(cJSON_GetArrayItem(cJSON_GetObjectItem(j, "humidity_pct"), 0)->valuedouble * 10));
    cJSON_Delete(j);

    /* negative tenths are printed with the sign once */
    add(-3.5f, 40, NAN, -1, 2 * STEP);
    j = as_json(2 * STEP, &s);
    TEST_ASSERT_EQUAL_INT(-35, (int)lround(cJSON_GetArrayItem(cJSON_GetObjectItem(j, "temperature_c"), 1)->valuedouble * 10));
    cJSON_Delete(j);
}

static void test_skipped_buckets_are_gaps(void)
{
    add(20, 40, 1000, 600, 0);
    add(25, 45, 1005, 900, 3 * STEP + 1000); /* two buckets with no reading */
    TEST_ASSERT_EQUAL(3, h.count);

    history_sample_t s[3];
    history_copy(&h, s, 3);
    TEST_ASSERT_EQUAL(200, s[0].temp_c10);
    TEST_ASSERT_EQUAL(HISTORY_NO_TEMP, s[1].temp_c10);
    TEST_ASSERT_EQUAL(HISTORY_NO_U16, s[2].co2_ppm);
    TEST_ASSERT_EQUAL(1, history_age_s(&h, 3 * STEP + 1000));
}

static void test_ring_keeps_the_newest_day(void)
{
    for (int i = 0; i <= HISTORY_LEN + 10; i++) {
        add(10.0f + (float)i / 10.0f, 50, 1000, 400 + i, (int64_t)i * STEP);
    }
    TEST_ASSERT_EQUAL(HISTORY_LEN, h.count);

    static history_sample_t buf[HISTORY_LEN];
    size_t n = history_copy(&h, buf, HISTORY_LEN);
    TEST_ASSERT_EQUAL(HISTORY_LEN, n);
    TEST_ASSERT_EQUAL(400 + 10, buf[0].co2_ppm);
    TEST_ASSERT_EQUAL(400 + HISTORY_LEN + 9, buf[n - 1].co2_ppm);

    /* the full day still parses, written in a modest number of pieces */
    sink_t s;
    cJSON *j = as_json((int64_t)(HISTORY_LEN + 10) * STEP, &s);
    TEST_ASSERT_EQUAL(HISTORY_LEN, cJSON_GetArraySize(cJSON_GetObjectItem(j, "pressure_hpa")));
    TEST_ASSERT_LESS_THAN(200, s.calls);
    cJSON_Delete(j);
}

static void test_out_of_range_readings_are_ignored(void)
{
    add(500.0f, 140, 0, 30000, 0);
    add(20, 40, 1000, 600, STEP);
    history_sample_t s;
    history_copy(&h, &s, 1);
    TEST_ASSERT_EQUAL(HISTORY_NO_TEMP, s.temp_c10);
    TEST_ASSERT_EQUAL(HISTORY_NO_U16, s.hum_pct10);
    TEST_ASSERT_EQUAL(HISTORY_NO_U16, s.press_hpa10);
    TEST_ASSERT_EQUAL(HISTORY_NO_U16, s.co2_ppm);
}

static void test_very_long_pause_does_not_spin(void)
{
    add(20, 40, 1000, 600, 0);
    add(20, 40, 1000, 600, 100000LL * STEP);
    TEST_ASSERT_EQUAL(HISTORY_LEN, h.count);
    TEST_ASSERT_EQUAL(100000LL * STEP, h.bucket_start_ms);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_empty_until_first_bucket_closes);
    RUN_TEST(test_bucket_is_averaged);
    RUN_TEST(test_missing_values_become_null);
    RUN_TEST(test_skipped_buckets_are_gaps);
    RUN_TEST(test_ring_keeps_the_newest_day);
    RUN_TEST(test_out_of_range_readings_are_ignored);
    RUN_TEST(test_very_long_pause_does_not_spin);
    return UNITY_END();
}
