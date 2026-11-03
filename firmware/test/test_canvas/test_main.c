#include <string.h>

#include <unity.h>

#include "bmp.h"
#include "epd_canvas.h"

static uint8_t fb[EPD_FRAMEBUFFER_SIZE];
static uint8_t fb2[EPD_FRAMEBUFFER_SIZE];

void setUp(void) { canvas_clear(fb, true); }
void tearDown(void) {}

static void test_utf8_decode(void)
{
    const char *s = "Aж€😀";
    TEST_ASSERT_EQUAL_HEX32('A', canvas_utf8_next(&s));
    TEST_ASSERT_EQUAL_HEX32(0x0436, canvas_utf8_next(&s));
    TEST_ASSERT_EQUAL_HEX32(0x20AC, canvas_utf8_next(&s));
    TEST_ASSERT_EQUAL_HEX32(0x1F600, canvas_utf8_next(&s));
    TEST_ASSERT_EQUAL_HEX32(0, canvas_utf8_next(&s));

    const char *bad = "\xD0" "A"; /* truncated sequence */
    TEST_ASSERT_EQUAL_HEX32('?', canvas_utf8_next(&bad));
    TEST_ASSERT_EQUAL_HEX32('A', canvas_utf8_next(&bad));
}

static void test_strlen_and_prefix(void)
{
    TEST_ASSERT_EQUAL(6, canvas_strlen("Привет"));
    TEST_ASSERT_EQUAL(4, canvas_utf8_prefix("Привет", 2));
    TEST_ASSERT_EQUAL(12, canvas_utf8_prefix("Привет", 99));
    TEST_ASSERT_EQUAL(6 * 6 * 2, canvas_text_width("Привет", 2));
}

/* Every Cyrillic letter must have its own glyph, not the '?' fallback. */
static void test_cyrillic_glyphs_present(void)
{
    uint8_t q[EPD_FRAMEBUFFER_SIZE];
    canvas_clear(q, true);
    canvas_text(q, 0, 0, "?", 1, true);

    const char *alphabet =
        "АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдеёжзийклмнопрстуфхцчшщъыьэюя…°";
    const char *p = alphabet;
    char one[8];
    while (*p) {
        const char *start = p;
        canvas_utf8_next(&p);
        memcpy(one, start, (size_t)(p - start));
        one[p - start] = '\0';

        canvas_clear(fb, true);
        canvas_text(fb, 0, 0, one, 1, true);
        TEST_ASSERT_FALSE_MESSAGE(memcmp(fb, q, sizeof(fb)) == 0, one);
    }
}

static void test_rotate180(void)
{
    canvas_set_pixel(fb, 0, 0, true);
    canvas_set_pixel(fb, 10, 20, true);
    canvas_rotate180(fb);
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, CANVAS_W - 1, CANVAS_H - 1));
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, CANVAS_W - 11, CANVAS_H - 21));
    TEST_ASSERT_FALSE(canvas_get_pixel(fb, 0, 0));
    canvas_rotate180(fb);
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 0, 0));
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 10, 20));
}

static void test_invert_and_clipping(void)
{
    canvas_fill_rect(fb, -5, -5, 10, 10, true); /* partly off-screen */
    TEST_ASSERT_TRUE(canvas_get_pixel(fb, 0, 0));
    TEST_ASSERT_FALSE(canvas_get_pixel(fb, 5, 5));
    TEST_ASSERT_FALSE(canvas_get_pixel(fb, -1, 0));
    memcpy(fb2, fb, sizeof(fb));
    canvas_invert(fb2);
    TEST_ASSERT_FALSE(canvas_get_pixel(fb2, 0, 0));
    TEST_ASSERT_TRUE(canvas_get_pixel(fb2, 5, 5));
}

static void test_bmp_export(void)
{
    static uint8_t bmp[BMP_SIZE];
    canvas_set_pixel(fb, 0, 0, true);
    canvas_set_pixel(fb, 9, 1, true);
    bmp_from_canvas(fb, bmp);
    TEST_ASSERT_EQUAL('B', bmp[0]);
    TEST_ASSERT_EQUAL('M', bmp[1]);
    TEST_ASSERT_EQUAL(BMP_SIZE, bmp[2] | bmp[3] << 8 | bmp[4] << 16);
    TEST_ASSERT_EQUAL(CANVAS_W, bmp[18] | bmp[19] << 8);
    /* top-down: first row first; black pixel = bit 0 */
    TEST_ASSERT_EQUAL_HEX8(0x7F, bmp[62]);
    TEST_ASSERT_EQUAL_HEX8(0xBF, bmp[62 + BMP_ROW_BYTES + 1]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_utf8_decode);
    RUN_TEST(test_strlen_and_prefix);
    RUN_TEST(test_cyrillic_glyphs_present);
    RUN_TEST(test_rotate180);
    RUN_TEST(test_invert_and_clipping);
    RUN_TEST(test_bmp_export);
    return UNITY_END();
}
