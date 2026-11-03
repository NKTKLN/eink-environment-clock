#include <string.h>

#include <unity.h>

#include "notify.h"

void setUp(void) {}
void tearDown(void) {}

static void test_set_expire_clear(void)
{
    notify_t n;
    notify_init(&n);
    TEST_ASSERT_FALSE(notify_active(&n, 0));

    notify_set(&n, "hello", 10, false, 1000);
    uint32_t seq = n.seq;
    TEST_ASSERT_TRUE(notify_active(&n, 10999));
    TEST_ASSERT_EQUAL(1, notify_remaining_s(&n, 10500));
    TEST_ASSERT_FALSE(notify_active(&n, 11000));
    TEST_ASSERT_EQUAL_STRING("", n.text);
    TEST_ASSERT_NOT_EQUAL(seq, n.seq);
}

static void test_no_ttl_stays(void)
{
    notify_t n;
    notify_init(&n);
    notify_set(&n, "x", 0, true, 0);
    TEST_ASSERT_TRUE(notify_active(&n, 1000000000LL));
    TEST_ASSERT_EQUAL(-1, notify_remaining_s(&n, 5));
    TEST_ASSERT_TRUE(n.fullscreen);
    notify_set(&n, "", 0, false, 0);
    TEST_ASSERT_FALSE(notify_active(&n, 0));
}

static void test_utf8_cut_on_boundary(void)
{
    notify_t n;
    notify_init(&n);
    char text[400];
    text[0] = '\0';
    for (int i = 0; i < 150; i++) {
        strcat(text, "ж"); /* 2 bytes each */
    }
    notify_set(&n, text, 0, false, 0);
    size_t len = strlen(n.text);
    TEST_ASSERT_TRUE(len < NOTIFY_TEXT_MAX);
    TEST_ASSERT_EQUAL(0, len % 2);
}

static void test_parse_json_and_plain(void)
{
    char text[64];
    int ttl;
    bool fs;

    const char *j = "{\"text\":\"Open the window\",\"ttl\":600,\"fullscreen\":true}";
    TEST_ASSERT_TRUE(notify_parse(j, (int)strlen(j), text, sizeof(text), &ttl, &fs));
    TEST_ASSERT_EQUAL_STRING("Open the window", text);
    TEST_ASSERT_EQUAL(600, ttl);
    TEST_ASSERT_TRUE(fs);

    const char *p = "Совещание до 15:00";
    TEST_ASSERT_TRUE(notify_parse(p, (int)strlen(p), text, sizeof(text), &ttl, &fs));
    TEST_ASSERT_EQUAL_STRING(p, text);
    TEST_ASSERT_EQUAL(0, ttl);
    TEST_ASSERT_FALSE(fs);

    const char *bad = "{\"text\":5}";
    TEST_ASSERT_FALSE(notify_parse(bad, (int)strlen(bad), text, sizeof(text), &ttl, &fs));

    /* payload is not NUL-terminated within len */
    TEST_ASSERT_TRUE(notify_parse("abcdef", 3, text, sizeof(text), &ttl, &fs));
    TEST_ASSERT_EQUAL_STRING("abc", text);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_set_expire_clear);
    RUN_TEST(test_no_ttl_stays);
    RUN_TEST(test_utf8_cut_on_boundary);
    RUN_TEST(test_parse_json_and_plain);
    return UNITY_END();
}
