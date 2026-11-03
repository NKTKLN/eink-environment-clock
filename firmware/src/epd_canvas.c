#include "epd_canvas.h"

#include <string.h>

#include "epd_font5x7.h"
#include "epd_font_cyr.h"

void canvas_clear(uint8_t *fb, bool white)
{
    memset(fb, white ? 0xFF : 0x00, EPD_FRAMEBUFFER_SIZE);
}

/* Landscape (x, y) -> native (source, gate), GxEPD2 rotation 3. */
static inline int pixel_index(int x, int y, uint8_t *mask)
{
    int sx = y;                   /* source: 0..127 */
    int gy = EPD_PANEL_H - 1 - x; /* gate:   0..295 */
    *mask = 0x80 >> (sx & 7);
    return gy * (EPD_PANEL_W / 8) + (sx >> 3);
}

void canvas_set_pixel(uint8_t *fb, int x, int y, bool black)
{
    if (x < 0 || x >= CANVAS_W || y < 0 || y >= CANVAS_H) {
        return;
    }
    uint8_t mask;
    int idx = pixel_index(x, y, &mask);
    if (black) {
        fb[idx] &= ~mask;
    } else {
        fb[idx] |= mask;
    }
}

bool canvas_get_pixel(const uint8_t *fb, int x, int y)
{
    if (x < 0 || x >= CANVAS_W || y < 0 || y >= CANVAS_H) {
        return false;
    }
    uint8_t mask;
    int idx = pixel_index(x, y, &mask);
    return (fb[idx] & mask) == 0;
}

void canvas_fill_rect(uint8_t *fb, int x, int y, int w, int h, bool black)
{
    for (int j = y; j < y + h; j++) {
        for (int i = x; i < x + w; i++) {
            canvas_set_pixel(fb, i, j, black);
        }
    }
}

void canvas_draw_rect(uint8_t *fb, int x, int y, int w, int h, bool black)
{
    canvas_fill_rect(fb, x, y, w, 1, black);
    canvas_fill_rect(fb, x, y + h - 1, w, 1, black);
    canvas_fill_rect(fb, x, y, 1, h, black);
    canvas_fill_rect(fb, x + w - 1, y, 1, h, black);
}

void canvas_invert(uint8_t *fb)
{
    for (int i = 0; i < EPD_FRAMEBUFFER_SIZE; i++) {
        fb[i] = (uint8_t)~fb[i];
    }
}

static uint8_t reverse_bits(uint8_t b)
{
    b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
    b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
    b = (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
    return b;
}

/* The buffer is one linear bit stream (gate-major), so a 180 degree turn
 * is just reading it backwards. */
void canvas_rotate180(uint8_t *fb)
{
    for (int i = 0, j = EPD_FRAMEBUFFER_SIZE - 1; i <= j; i++, j--) {
        uint8_t a = reverse_bits(fb[i]);
        fb[i] = reverse_bits(fb[j]);
        fb[j] = a;
    }
}

void canvas_draw_xbm(uint8_t *fb, int x, int y, const uint8_t *xbm,
                     int w, int h, bool black)
{
    int bytes_per_row = (w + 7) / 8;

    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            uint8_t byte = xbm[j * bytes_per_row + (i >> 3)];
            if (byte & (1 << (i & 7))) { /* XBM: LSB first */
                canvas_set_pixel(fb, x + i, y + j, black);
            }
        }
    }
}

/* ===================================================== */

uint32_t canvas_utf8_next(const char **s)
{
    const uint8_t *p = (const uint8_t *)*s;
    uint32_t cp;
    int extra;

    if (p[0] == 0) {
        return 0;
    }
    if (p[0] < 0x80) {
        *s += 1;
        return p[0];
    } else if ((p[0] & 0xE0) == 0xC0) {
        cp = p[0] & 0x1F;
        extra = 1;
    } else if ((p[0] & 0xF0) == 0xE0) {
        cp = p[0] & 0x0F;
        extra = 2;
    } else if ((p[0] & 0xF8) == 0xF0) {
        cp = p[0] & 0x07;
        extra = 3;
    } else {
        *s += 1;
        return '?';
    }

    for (int i = 1; i <= extra; i++) {
        if ((p[i] & 0xC0) != 0x80) {
            *s += 1;
            return '?';
        }
        cp = (cp << 6) | (p[i] & 0x3F);
    }
    *s += 1 + extra;
    return cp;
}

int canvas_strlen(const char *s)
{
    int n = 0;
    while (canvas_utf8_next(&s) != 0) {
        n++;
    }
    return n;
}

size_t canvas_utf8_prefix(const char *s, int n)
{
    const char *p = s;
    for (int i = 0; i < n && canvas_utf8_next(&p) != 0; i++) {
    }
    return (size_t)(p - s);
}

static const uint8_t *glyph_data(uint32_t cp)
{
    if (cp == 0xB0) {
        return font5x7[FONT5X7_DEGREE_INDEX];
    }
    if (cp >= ' ' && cp <= '~') {
        return font5x7[cp - ' '];
    }

    int lo = 0;
    int hi = (int)(sizeof(font_ext) / sizeof(font_ext[0])) - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (font_ext[mid].codepoint == cp) {
            return font_ext[mid].cols;
        }
        if (font_ext[mid].codepoint < cp) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return font5x7['?' - ' '];
}

/* 5 column bytes per glyph, bit 0 = top row, bit 7 = descender row. */
static void draw_glyph(uint8_t *fb, int x, int y, uint32_t cp, int scale,
                       bool black)
{
    const uint8_t *g = glyph_data(cp);

    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 8; row++) {
            if (bits & (1 << row)) {
                canvas_fill_rect(fb, x + col * scale, y + row * scale,
                                 scale, scale, black);
            }
        }
    }
}

int canvas_text(uint8_t *fb, int x, int y, const char *s, int scale,
                bool black)
{
    uint32_t cp;
    while ((cp = canvas_utf8_next(&s)) != 0) {
        draw_glyph(fb, x, y, cp, scale, black);
        x += CANVAS_CHAR_W * scale;
    }
    return x;
}

int canvas_text_width(const char *s, int scale)
{
    return canvas_strlen(s) * CANVAS_CHAR_W * scale;
}
