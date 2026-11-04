#include <unity.h>

#include "boot_guard.h"

void setUp(void) {}
void tearDown(void) {}

static void test_third_quick_power_on_triggers(void)
{
    bool trig;
    uint8_t c = 0;
    c = boot_guard_step(c, true, &trig);
    TEST_ASSERT_FALSE(trig);
    c = boot_guard_step(c, true, &trig);
    TEST_ASSERT_FALSE(trig);
    c = boot_guard_step(c, true, &trig);
    TEST_ASSERT_TRUE(trig);
    TEST_ASSERT_EQUAL(0, c);
}

static void test_software_reset_breaks_sequence(void)
{
    bool trig;
    uint8_t c = boot_guard_step(0, true, &trig);
    c = boot_guard_step(c, true, &trig);
    c = boot_guard_step(c, false, &trig); /* OTA restart, panic, ... */
    TEST_ASSERT_EQUAL(0, c);
    c = boot_guard_step(c, true, &trig);
    TEST_ASSERT_FALSE(trig);
}

static void test_garbage_counter(void)
{
    bool trig;
    boot_guard_step(250, true, &trig);
    TEST_ASSERT_TRUE(trig);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_third_quick_power_on_triggers);
    RUN_TEST(test_software_reset_breaks_sequence);
    RUN_TEST(test_garbage_counter);
    return UNITY_END();
}
