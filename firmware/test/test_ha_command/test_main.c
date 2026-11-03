#include <string.h>

#include <unity.h>

#include "ha_command.h"

void setUp(void) {}
void tearDown(void) {}

static const char *BASE = "eink-clock/a1b2c3";
static ha_command_t c;

static void parse(const char *topic, const char *payload)
{
    ha_parse_command(BASE, "homeassistant", topic, (int)strlen(topic), payload,
                     (int)strlen(payload), &c);
}

static void test_notify_json(void)
{
    parse("eink-clock/a1b2c3/notify", "{\"text\":\"Проветрите\",\"ttl\":600,\"fullscreen\":true}");
    TEST_ASSERT_EQUAL(HA_ACT_NOTIFY, c.action);
    TEST_ASSERT_EQUAL_STRING("Проветрите", c.text);
    TEST_ASSERT_EQUAL(600, c.ttl_s);
    TEST_ASSERT_TRUE(c.fullscreen);

    parse("eink-clock/a1b2c3/notify", "{\"text\":1}");
    TEST_ASSERT_EQUAL(HA_ACT_INVALID, c.action);
}

static void test_text_entity_is_plain(void)
{
    parse("eink-clock/a1b2c3/notification/set", "{\"text\":\"x\"}");
    TEST_ASSERT_EQUAL(HA_ACT_NOTIFY, c.action);
    TEST_ASSERT_EQUAL_STRING("{\"text\":\"x\"}", c.text);
    TEST_ASSERT_EQUAL(0, c.ttl_s);

    parse("eink-clock/a1b2c3/notification/set", "");
    TEST_ASSERT_EQUAL(HA_ACT_NOTIFY, c.action); /* empty text clears */
    TEST_ASSERT_EQUAL_STRING("", c.text);
}

static void test_long_text_cut_on_utf8_boundary(void)
{
    char payload[600] = "";
    for (int i = 0; i < 200; i++) {
        strcat(payload, "ж");
    }
    parse("eink-clock/a1b2c3/notification/set", payload);
    size_t len = strlen(c.text);
    TEST_ASSERT_TRUE(len < NOTIFY_TEXT_MAX);
    TEST_ASSERT_EQUAL(0, len % 2);
}

static void test_commands(void)
{
    parse("eink-clock/a1b2c3/command", "reboot");
    TEST_ASSERT_EQUAL(HA_ACT_REBOOT, c.action);
    parse("eink-clock/a1b2c3/command", "clear_notification");
    TEST_ASSERT_EQUAL(HA_ACT_CLEAR, c.action);
    parse("eink-clock/a1b2c3/command", "calibrate_co2");
    TEST_ASSERT_EQUAL(HA_ACT_CALIBRATE, c.action);
    parse("eink-clock/a1b2c3/command", "rebootx");
    TEST_ASSERT_EQUAL(HA_ACT_INVALID, c.action);
}

static void test_layout_and_threshold(void)
{
    parse("eink-clock/a1b2c3/layout/set", "co2");
    TEST_ASSERT_EQUAL(HA_ACT_LAYOUT, c.action);
    TEST_ASSERT_EQUAL(LAYOUT_CO2, c.layout);
    parse("eink-clock/a1b2c3/layout/set", "big");
    TEST_ASSERT_EQUAL(HA_ACT_INVALID, c.action);

    parse("eink-clock/a1b2c3/co2_threshold/set", "1250.0");
    TEST_ASSERT_EQUAL(HA_ACT_THRESHOLD, c.action);
    TEST_ASSERT_EQUAL(1250, c.threshold);
    parse("eink-clock/a1b2c3/co2_threshold/set", " 900\n");
    TEST_ASSERT_EQUAL(900, c.threshold);
    /* garbage must not reset the threshold to the minimum */
    parse("eink-clock/a1b2c3/co2_threshold/set", "abc");
    TEST_ASSERT_EQUAL(HA_ACT_INVALID, c.action);
    parse("eink-clock/a1b2c3/co2_threshold/set", "99999");
    TEST_ASSERT_EQUAL(HA_ACT_INVALID, c.action);
}

static void test_foreign_topics(void)
{
    parse("homeassistant/status", "online");
    TEST_ASSERT_EQUAL(HA_ACT_HA_ONLINE, c.action);
    parse("homeassistant/status", "offline");
    TEST_ASSERT_EQUAL(HA_ACT_NONE, c.action);
    parse("eink-clock/a1b2c3x/command", "reboot");
    TEST_ASSERT_EQUAL(HA_ACT_NONE, c.action);
    parse("eink-clock/a1b2c3/command/extra", "reboot");
    TEST_ASSERT_EQUAL(HA_ACT_NONE, c.action);
    parse("eink-clock/a1b2c3/state", "{}");
    TEST_ASSERT_EQUAL(HA_ACT_NONE, c.action);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_notify_json);
    RUN_TEST(test_text_entity_is_plain);
    RUN_TEST(test_long_text_cut_on_utf8_boundary);
    RUN_TEST(test_commands);
    RUN_TEST(test_layout_and_threshold);
    RUN_TEST(test_foreign_topics);
    return UNITY_END();
}
