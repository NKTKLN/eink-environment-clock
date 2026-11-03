#include <unity.h>

#include "http_guard.h"

void setUp(void) {}
void tearDown(void) {}

static void test_hosts(void)
{
    TEST_ASSERT_TRUE(http_host_allowed("10.64.66.3"));
    TEST_ASSERT_TRUE(http_host_allowed("192.168.4.1:80"));
    TEST_ASSERT_TRUE(http_host_allowed("eink-clock-156370.local"));
    TEST_ASSERT_TRUE(http_host_allowed("EINK-CLOCK.LOCAL"));
    TEST_ASSERT_TRUE(http_host_allowed("clock"));
    TEST_ASSERT_TRUE(http_host_allowed("clock.lan:8080"));
    TEST_ASSERT_TRUE(http_host_allowed("clock.home.arpa"));
    TEST_ASSERT_TRUE(http_host_allowed("[fe80::1]:80"));
    TEST_ASSERT_TRUE(http_host_allowed(NULL));
    TEST_ASSERT_TRUE(http_host_allowed(""));

    /* public names pointing at the clock: DNS rebinding */
    TEST_ASSERT_FALSE(http_host_allowed("evil.example.com"));
    TEST_ASSERT_FALSE(http_host_allowed("10.64.66.3.nip.io"));
    TEST_ASSERT_TRUE(http_host_allowed("localhost")); /* single label */
    TEST_ASSERT_FALSE(http_host_allowed("x.notlocal"));
    TEST_ASSERT_FALSE(http_host_allowed("a b"));
    TEST_ASSERT_FALSE(http_host_allowed("1.2.3"));
    TEST_ASSERT_FALSE(http_host_allowed("1.2.3.4.5"));
    TEST_ASSERT_FALSE(http_host_allowed("[fe80::1"));
}

static void test_origins(void)
{
    TEST_ASSERT_TRUE(http_origin_allowed(NULL, "10.64.66.3"));
    TEST_ASSERT_TRUE(http_origin_allowed("", "10.64.66.3"));
    TEST_ASSERT_TRUE(http_origin_allowed("http://10.64.66.3", "10.64.66.3"));
    TEST_ASSERT_TRUE(http_origin_allowed("http://clock.local:8080", "clock.local:8080"));
    TEST_ASSERT_TRUE(http_origin_allowed("http://Clock.Local", "clock.local"));

    TEST_ASSERT_FALSE(http_origin_allowed("https://evil.example.com", "10.64.66.3"));
    TEST_ASSERT_FALSE(http_origin_allowed("http://10.64.66.3:81", "10.64.66.3"));
    TEST_ASSERT_FALSE(http_origin_allowed("null", "10.64.66.3")); /* sandboxed iframe */
    TEST_ASSERT_FALSE(http_origin_allowed("http://10.64.66.3", NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_hosts);
    RUN_TEST(test_origins);
    return UNITY_END();
}
