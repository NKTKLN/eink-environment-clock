#include <stdio.h>
#include <string.h>

#include <unity.h>

#include "release.h"

void setUp(void) {}
void tearDown(void) {}

static void test_version_order(void)
{
    TEST_ASSERT_TRUE(version_compare("1.2.0", "1.10.0") < 0);
    TEST_ASSERT_TRUE(version_compare("v1.2.0", "1.2.0") == 0);
    TEST_ASSERT_TRUE(version_compare("1.2.0-rc1", "1.2.0") < 0);
    TEST_ASSERT_TRUE(version_compare("1.2.0-rc.2", "1.2.0-rc.10") < 0);
    TEST_ASSERT_TRUE(version_compare("1.2.0-alpha", "1.2.0-beta") < 0);
    TEST_ASSERT_TRUE(version_compare("1.2.0-1", "1.2.0-alpha") < 0);
    TEST_ASSERT_TRUE(version_compare("1.2.0+build5", "1.2.0") == 0);
    TEST_ASSERT_TRUE(version_compare("2.0.0", "1.99.99") > 0);
    TEST_ASSERT_TRUE(version_compare("1.2", "1.2.0") == 0);
    TEST_ASSERT_TRUE(version_compare("garbage", "0.0.1") < 0);
    TEST_ASSERT_TRUE(version_compare("1.0.0", "1.0.0-dirty") > 0);
}

static const char *RELEASE =
    "{\"tag_name\":\"v1.3.0\",\"html_url\":\"https://github.com/o/r/releases/tag/v1.3.0\","
    "\"prerelease\":false,\"body\":\"- faster boot\\n- fixes\","
    "\"assets\":[{\"name\":\"firmware.elf\",\"browser_download_url\":\"https://x/elf\",\"size\":1},"
    "{\"name\":\"firmware.bin\",\"browser_download_url\":\"https://github.com/o/r/releases/download/v1.3.0/firmware.bin\",\"size\":1240000}]}";

static void test_parse_release(void)
{
    release_t r;
    TEST_ASSERT_TRUE(release_parse(RELEASE, strlen(RELEASE), "firmware.bin", &r));
    TEST_ASSERT_EQUAL_STRING("1.3.0", r.version);
    TEST_ASSERT_EQUAL_STRING("https://github.com/o/r/releases/download/v1.3.0/firmware.bin", r.url);
    TEST_ASSERT_EQUAL(1240000, r.size);
    TEST_ASSERT_EQUAL_STRING("- faster boot\n- fixes", r.notes);
    TEST_ASSERT_FALSE(r.prerelease);
}

static void test_parse_rejects_missing_asset(void)
{
    release_t r;
    TEST_ASSERT_FALSE(release_parse(RELEASE, strlen(RELEASE), "other.bin", &r));
    const char *no_tag = "{\"assets\":[]}";
    TEST_ASSERT_FALSE(release_parse(no_tag, strlen(no_tag), "firmware.bin", &r));
    TEST_ASSERT_FALSE(release_parse("{", 1, "firmware.bin", &r));
}

static void test_long_notes_cut_on_utf8_boundary(void)
{
    char json[3000];
    char body[1500] = "";
    for (int i = 0; i < 400; i++) {
        strcat(body, "я");
    }
    snprintf(json, sizeof(json),
             "{\"tag_name\":\"1.0.1\",\"body\":\"%s\",\"assets\":[{\"name\":\"firmware.bin\","
             "\"browser_download_url\":\"https://x/f\",\"size\":5}]}", body);
    release_t r;
    TEST_ASSERT_TRUE(release_parse(json, strlen(json), "firmware.bin", &r));
    TEST_ASSERT_EQUAL(0, strlen(r.notes) % 2);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_version_order);
    RUN_TEST(test_parse_release);
    RUN_TEST(test_parse_rejects_missing_asset);
    RUN_TEST(test_long_notes_cut_on_utf8_boundary);
    return UNITY_END();
}
