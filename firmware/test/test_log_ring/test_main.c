#include <string.h>

#include <unity.h>

#include "log_ring.h"

void setUp(void) {}
void tearDown(void) {}

static void test_read_after_position(void)
{
    char buf[16], out[32];
    log_ring_t r;
    log_ring_init(&r, buf, sizeof(buf));
    log_ring_write(&r, "hello ", 6);
    uint32_t pos = 0;
    int dropped;
    size_t n = log_ring_read(&r, &pos, out, sizeof(out), &dropped);
    TEST_ASSERT_EQUAL(6, n);
    TEST_ASSERT_EQUAL(6, pos);
    TEST_ASSERT_EQUAL(0, dropped);

    log_ring_write(&r, "world", 5);
    n = log_ring_read(&r, &pos, out, sizeof(out), &dropped);
    out[n] = '\0';
    TEST_ASSERT_EQUAL_STRING("world", out);
}

static void test_wraparound_and_drop(void)
{
    char buf[8], out[32];
    log_ring_t r;
    log_ring_init(&r, buf, sizeof(buf));
    log_ring_write(&r, "0123456789AB", 12); /* bigger than the ring */
    uint32_t pos = 0;
    int dropped;
    size_t n = log_ring_read(&r, &pos, out, sizeof(out), &dropped);
    out[n] = '\0';
    TEST_ASSERT_EQUAL_STRING("456789AB", out);
    TEST_ASSERT_EQUAL(1, dropped);
    TEST_ASSERT_EQUAL(12, pos);

    log_ring_write(&r, "xyz", 3);
    n = log_ring_read(&r, &pos, out, sizeof(out), &dropped);
    out[n] = '\0';
    TEST_ASSERT_EQUAL_STRING("xyz", out);
    TEST_ASSERT_EQUAL(0, dropped);
}

static void test_reader_ahead_after_restart(void)
{
    char buf[8], out[8];
    log_ring_t r;
    log_ring_init(&r, buf, sizeof(buf));
    log_ring_write(&r, "ab", 2);
    uint32_t pos = 500; /* position from before a reboot */
    int dropped;
    size_t n = log_ring_read(&r, &pos, out, sizeof(out), &dropped);
    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_EQUAL(1, dropped);
}

static void test_cap_limits_read(void)
{
    char buf[16], out[4];
    log_ring_t r;
    log_ring_init(&r, buf, sizeof(buf));
    log_ring_write(&r, "abcdefgh", 8);
    uint32_t pos = 0;
    int dropped;
    TEST_ASSERT_EQUAL(4, log_ring_read(&r, &pos, out, sizeof(out), &dropped));
    TEST_ASSERT_EQUAL(4, pos);
    TEST_ASSERT_EQUAL(4, log_ring_read(&r, &pos, out, sizeof(out), &dropped));
    TEST_ASSERT_EQUAL(0, log_ring_read(&r, &pos, out, sizeof(out), &dropped));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_read_after_position);
    RUN_TEST(test_wraparound_and_drop);
    RUN_TEST(test_reader_ahead_after_restart);
    RUN_TEST(test_cap_limits_read);
    return UNITY_END();
}
