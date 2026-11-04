#include <math.h>
#include <stdio.h>
#include <string.h>

#include <unity.h>

#include "ha_discovery.h"

void setUp(void) {}
void tearDown(void) {}

static const ha_device_t DEV = {
    .device_id = "a1b2c3",
    .name = "Desk clock",
    .base = "eink-clock/a1b2c3",
    .discovery_prefix = "homeassistant",
    .sw_version = "1.0.0",
    .config_url = "http://192.168.1.42/",
};

static void test_topic(void)
{
    char t[128];
    ha_discovery_topic(t, sizeof(t), &DEV);
    TEST_ASSERT_EQUAL_STRING("homeassistant/device/eink_clock_a1b2c3/config", t);
}

static const cJSON *cmp(const cJSON *root, const char *key)
{
    const cJSON *c = cJSON_GetObjectItem(cJSON_GetObjectItem(root, "components"), key);
    TEST_ASSERT_NOT_NULL_MESSAGE(c, key);
    return c;
}

static const char *str(const cJSON *o, const char *key)
{
    const cJSON *v = cJSON_GetObjectItem(o, key);
    TEST_ASSERT_TRUE_MESSAGE(cJSON_IsString(v), key);
    return v->valuestring;
}

static void test_discovery_payload(void)
{
    char *text = ha_discovery_payload(&DEV);
    cJSON *root = cJSON_Parse(text);
    TEST_ASSERT_NOT_NULL(root);

    TEST_ASSERT_EQUAL_STRING("eink-clock/a1b2c3/status",
                             str(root, "availability_topic"));
    const cJSON *dev = cJSON_GetObjectItem(root, "device");
    TEST_ASSERT_EQUAL_STRING("Desk clock", str(dev, "name"));

    const char *keys[] = { "temperature", "humidity", "pressure", "co2",
                           "rssi", "uptime", "co2_high", "notification",
                           "notification_temp", "notification_time",
                           "clear_notification", "reboot", "calibrate_co2",
                           "layout", "co2_threshold" };
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        const cJSON *c = cmp(root, keys[i]);
        str(c, "platform");
        char uid[64];
        snprintf(uid, sizeof(uid), "eink_clock_a1b2c3_%s", keys[i]);
        TEST_ASSERT_EQUAL_STRING(uid, str(c, "unique_id"));
    }

    TEST_ASSERT_EQUAL_STRING("carbon_dioxide", str(cmp(root, "co2"), "device_class"));
    TEST_ASSERT_EQUAL_STRING("eink-clock/a1b2c3/notification/set",
                             str(cmp(root, "notification"), "command_topic"));
    TEST_ASSERT_EQUAL_STRING("eink-clock/a1b2c3/notification_temp/set",
                             str(cmp(root, "notification_temp"), "command_topic"));
    TEST_ASSERT_EQUAL_STRING("eink-clock/a1b2c3/notification_time/set",
                             str(cmp(root, "notification_time"), "command_topic"));
    TEST_ASSERT_EQUAL_STRING("min", str(cmp(root, "notification_time"), "unit_of_measurement"));
    TEST_ASSERT_EQUAL_STRING("reboot", str(cmp(root, "reboot"), "payload_press"));
    TEST_ASSERT_EQUAL(LAYOUT_COUNT,
                      cJSON_GetArraySize(cJSON_GetObjectItem(cmp(root, "layout"), "options")));

    cJSON_Delete(root);
    cJSON_free(text);
}

static void test_state_payload(void)
{
    ha_state_t st = {
        .data = { .temperature_c = 22.345f, .humidity_pct = NAN,
                  .pressure_hpa = 1001.26f, .co2_ppm = 1200 },
        .co2_threshold_ppm = 1000,
        .layout = LAYOUT_CO2,
        .rssi_dbm = -60,
        .uptime_s = 42,
        .notification = "hi",
        .notification_temp = "",
        .temp_notice_min = 15,
    };
    char *text = ha_state_payload(&st);
    cJSON *root = cJSON_Parse(text);
    TEST_ASSERT_NOT_NULL(root);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 22.3f, (float)cJSON_GetObjectItem(root, "temperature")->valuedouble);
    TEST_ASSERT_TRUE(cJSON_IsNull(cJSON_GetObjectItem(root, "humidity")));
    TEST_ASSERT_EQUAL_STRING("ON", str(root, "co2_high"));
    TEST_ASSERT_EQUAL_STRING("co2", str(root, "layout"));
    TEST_ASSERT_EQUAL_STRING("hi", str(root, "notification"));
    TEST_ASSERT_EQUAL_STRING("", str(root, "notification_temp"));
    TEST_ASSERT_EQUAL(15, cJSON_GetObjectItem(root, "notification_time")->valueint);
    cJSON_Delete(root);
    cJSON_free(text);

    st.data.co2_ppm = -1;
    text = ha_state_payload(&st);
    TEST_ASSERT_NOT_NULL(strstr(text, "\"co2_high\":null"));
    cJSON_free(text);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_topic);
    RUN_TEST(test_discovery_payload);
    RUN_TEST(test_state_payload);
    return UNITY_END();
}
