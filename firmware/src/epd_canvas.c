#include "epd_canvas.h"

#include <string.h>

#include "epd_font5x7.h"

void canvas_clear(uint8_t *fb, bool white)
{
    memset(fb, white ? 0xFF : 0x00, EPD_FRAMEBUFFER_SIZE);
}

void canvas_set_pixel(uint8_t *fb, int x, int y, bool black)
{
    if (x < 0 || x >= CANVAS_W || y < 0 || y >= CANVAS_H) {
        return;
    }

    /* Landscape (x, y) -> native (source, gate), GxEPD2 rotation 3. */
    int sx = y;                    /* source: 0..127 */
    int gy = EPD_PANEL_H - 1 - x;  /* gate:   0..295 */

    int idx = gy * (EPD_PANEL_W / 8) + (sx >> 3);
    uint8_t mask = 0x80 >> (sx & 7);

    if (black) {
        fb[idx] &= ~mask;
    } else {
        fb[idx] |= mask;
    }
}

void canvas_fill_rect(uint8_t *fb, int x, int y, int w, int h, bool black)
{
    for (int j = y; j < y + h; j++) {
        for (int i = x; i < x + w; i++) {
            canvas_set_pixel(fb, i, j, black);
        }
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

static const uint8_t *glyph_data(char c)
{
    unsigned char u = (unsigned char)c;

    if (u == 0xF8) { /* degree sign, as in the old sketch */
        return font5x7[FONT5X7_DEGREE_INDEX];
    }
    if (u < ' ' || u > '~') {
        u = '?';
    }
    return font5x7[u - ' '];
}

/* Adafruit classic font: 5 column bytes per glyph, bit 0 = top row. */
static void draw_char(uint8_t *fb, int x, int y, char c, int scale,
                      bool black)
{
    const uint8_t *g = glyph_data(c);

    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                if (scale == 1) {
                    canvas_set_pixel(fb, x + col, y + row, black);
                } else {
                    canvas_fill_rect(fb, x + col * scale, y + row * scale,
                                     scale, scale, black);
                }
            }
        }
    }
}

int canvas_text(uint8_t *fb, int x, int y, const char *s, int scale,
                bool black)
{
    for (; *s != '\0'; s++) {
        draw_char(fb, x, y, *s, scale, black);
        x += CANVAS_CHAR_W * scale;
    }
    return x;
}

int canvas_text_width(const char *s, int scale)
{
    return (int)strlen(s) * CANVAS_CHAR_W * scale;
}
