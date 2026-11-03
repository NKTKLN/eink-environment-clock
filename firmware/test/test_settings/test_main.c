#include <string.h>

#include <unity.h>

#include "settings.h"

void setUp(void) {}
void tearDown(void) {}

static void apply(settings_t *s, const char *json)
{
    cJSON *j = cJSON_Parse(json);
    TEST_ASSERT_NOT_NULL_MESSAGE(j, json);
    settings_apply_json(s, j);
    cJSON_Delete(j);
}

static void test_defaults_are_sane(void)
{
    settings_t s;
    settings_defaults(&s);
    TEST_ASSERT_FALSE(settings_sanitize(&s));
    TEST_ASSERT_EQUAL_STRING("MSK-3", s.display.tz_posix);
    TEST_ASSERT_EQUAL(LAYOUT_CLASSIC, s.display.layout);
    TEST_ASSERT_EQUAL(1000, s.sensors.co2_threshold_ppm);
    for (int i = 0; i < METRIC_COUNT; i++) {
        TEST_ASSERT_EQUAL(i, s.display.metrics[i].id);
        TEST_ASSERT_TRUE(s.display.metrics[i].enabled);
    }
}

static void test_round_trip_with_secrets(void)
{
    settings_t a, b;
    settings_defaults(&a);
    strcpy(a.wifi.ssid, "home");
    strcpy(a.wifi.password, "secret pass");
    strcpy(a.mqtt.password, "mq");
    a.display.layout = LAYOUT_CO2;
    a.display.language = LANG_RU;
    a.display.metrics[0].id = METRIC_CO2;
    a.display.metrics[3].id = METRIC_TEMPERATURE;
    a.display.metrics[1].enabled = false;

    cJSON *j = settings_to_json(&a, true);
    settings_defaults(&b);
    settings_apply_json(&b, j);
    cJSON_Delete(j);

    TEST_ASSERT_EQUAL_STRING("home", b.wifi.ssid);
    TEST_ASSERT_EQUAL_STRING("secret pass", b.wifi.password);
    TEST_ASSERT_EQUAL_STRING("mq", b.mqtt.password);
    TEST_ASSERT_EQUAL(LAYOUT_CO2, b.display.layout);
    TEST_ASSERT_EQUAL(LANG_RU, b.display.language);
    for (int i = 0; i < METRIC_COUNT; i++) {
        TEST_ASSERT_EQUAL(a.display.metrics[i].id, b.display.metrics[i].id);
        TEST_ASSERT_EQUAL(a.display.metrics[i].enabled, b.display.metrics[i].enabled);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.3f, b.sensors.temp_offset_c);
    TEST_ASSERT_TRUE(b.update.check);
    TEST_ASSERT_FALSE(b.update.auto_install);
}

static void test_public_json_hides_secrets(void)
{
    settings_t s;
    settings_defaults(&s);
    strcpy(s.wifi.password, "topsecret");
    cJSON *j = settings_to_json(&s, false);
    char *text = cJSON_PrintUnformatted(j);
    TEST_ASSERT_NULL(strstr(text, "topsecret"));
    TEST_ASSERT_NOT_NULL(strstr(text, "\"password_set\":true"));
    cJSON_free(text);
    cJSON_Delete(j);
}

static void test_partial_patch_keeps_other_fields(void)
{
    settings_t s;
    settings_defaults(&s);
    strcpy(s.mqtt.password, "keep");
    apply(&s, "{\"display\":{\"hour24\":true},\"mqtt\":{\"host\":\"ha.lan\"}}");
    TEST_ASSERT_TRUE(s.display.hour24);
    TEST_ASSERT_EQUAL_STRING("ha.lan", s.mqtt.host);
    TEST_ASSERT_EQUAL_STRING("keep", s.mqtt.password);
    TEST_ASSERT_EQUAL(1883, s.mqtt.port);
    TEST_ASSERT_TRUE(s.display.show_date);
}

static void test_invalid_values_are_clamped(void)
{
    settings_t s;
    settings_defaults(&s);
    apply(&s, "{\"display\":{\"layout\":\"nope\",\"tz_posix\":\"1\"},"
              "\"sensors\":{\"co2_threshold_ppm\":99999,"
              "\"temp_offset_c\":-50},"
              "\"mqtt\":{\"port\":70000,\"prefix\":\"a/#/b/\"}}");
    TEST_ASSERT_EQUAL(LAYOUT_CLASSIC, s.display.layout);
    TEST_ASSERT_EQUAL_STRING("MSK-3", s.display.tz_posix);
    TEST_ASSERT_EQUAL(CO2_THRESHOLD_MAX, s.sensors.co2_threshold_ppm);
    TEST_ASSERT_EQUAL_FLOAT(-TEMP_OFFSET_LIMIT, s.sensors.temp_offset_c);
    TEST_ASSERT_EQUAL(1883, s.mqtt.port);
    TEST_ASSERT_EQUAL_STRING("a/_/b", s.mqtt.prefix);
}

static void test_metric_order_repaired(void)
{
    settings_t s;
    settings_defaults(&s);
    /* duplicate + unknown + missing entries */
    apply(&s, "{\"display\":{\"metrics\":["
              "{\"id\":\"co2\",\"enabled\":true},"
              "{\"id\":\"co2\",\"enabled\":false},"
              "{\"id\":\"bogus\"},"
              "{\"id\":\"humidity\",\"enabled\":false}]}}");
    TEST_ASSERT_EQUAL(METRIC_CO2, s.display.metrics[0].id);
    TEST_ASSERT_TRUE(s.display.metrics[0].enabled);
    TEST_ASSERT_EQUAL(METRIC_HUMIDITY, s.display.metrics[1].id);
    TEST_ASSERT_FALSE(s.display.metrics[1].enabled);
    /* missing ones appended, disabled */
    TEST_ASSERT_EQUAL(METRIC_TEMPERATURE, s.display.metrics[2].id);
    TEST_ASSERT_FALSE(s.display.metrics[2].enabled);
    TEST_ASSERT_EQUAL(METRIC_PRESSURE, s.display.metrics[3].id);
}

static void test_long_strings_truncated(void)
{
    settings_t s;
    settings_defaults(&s);
    apply(&s, "{\"wifi\":{\"ssid\":\"0123456789012345678901234567890123456789\"}}");
    TEST_ASSERT_EQUAL(32, strlen(s.wifi.ssid));
}

static void test_layout_names(void)
{
    layout_t l;
    TEST_ASSERT_TRUE(layout_from_name("sensors", &l));
    TEST_ASSERT_EQUAL(LAYOUT_SENSORS, l);
    TEST_ASSERT_FALSE(layout_from_name("Sensors", &l));
    TEST_ASSERT_FALSE(layout_from_name(NULL, &l));
}

/* Settings saved by an older firmware (fewer keys) and by a newer one
 * (unknown keys) must load after an update without losing anything the
 * current firmware knows about. */
static void test_settings_survive_firmware_updates(void)
{
    const char *older =
        "{\"name\":\"Kitchen\",\"wifi\":{\"ssid\":\"home\",\"password\":\"pw\"},"
        "\"display\":{\"hour24\":true,\"layout\":\"co2\"},"
        "\"mqtt\":{\"enabled\":true,\"host\":\"ha.lan\",\"password\":\"mq\"}}";
    const char *newer =
        "{\"name\":\"Kitchen\",\"wifi\":{\"ssid\":\"home\",\"password\":\"pw\",\"bssid\":\"x\"},"
        "\"display\":{\"hour24\":true,\"layout\":\"co2\",\"brightness\":3,"
        "\"metrics\":[{\"id\":\"co2\",\"enabled\":true},{\"id\":\"pm25\",\"enabled\":true}]},"
        "\"update\":{\"auto\":true},"
        "\"mqtt\":{\"enabled\":true,\"host\":\"ha.lan\",\"password\":\"mq\"}}";

    const char *docs[] = { older, newer };
    for (int i = 0; i < 2; i++) {
        settings_t s;
        settings_defaults(&s);
        apply(&s, docs[i]);
        TEST_ASSERT_EQUAL_STRING("Kitchen", s.name);
        TEST_ASSERT_EQUAL_STRING("home", s.wifi.ssid);
        TEST_ASSERT_EQUAL_STRING("pw", s.wifi.password);
        TEST_ASSERT_TRUE(s.display.hour24);
        TEST_ASSERT_EQUAL(LAYOUT_CO2, s.display.layout);
        TEST_ASSERT_TRUE(s.mqtt.enabled);
        TEST_ASSERT_EQUAL_STRING("ha.lan", s.mqtt.host);
        TEST_ASSERT_EQUAL_STRING("mq", s.mqtt.password);
        /* keys the old firmware did not have get their defaults */
        TEST_ASSERT_EQUAL(1000, s.sensors.co2_threshold_ppm);
        TEST_ASSERT_EQUAL(1883, s.mqtt.port);
        /* every metric still present exactly once */
        bool seen[METRIC_COUNT] = { false };
        for (int m = 0; m < METRIC_COUNT; m++) {
            TEST_ASSERT_FALSE(seen[s.display.metrics[m].id]);
            seen[s.display.metrics[m].id] = true;
        }
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_defaults_are_sane);
    RUN_TEST(test_round_trip_with_secrets);
    RUN_TEST(test_public_json_hides_secrets);
    RUN_TEST(test_partial_patch_keeps_other_fields);
    RUN_TEST(test_invalid_values_are_clamped);
    RUN_TEST(test_metric_order_repaired);
    RUN_TEST(test_long_strings_truncated);
    RUN_TEST(test_layout_names);
    RUN_TEST(test_settings_survive_firmware_updates);
    return UNITY_END();
}
