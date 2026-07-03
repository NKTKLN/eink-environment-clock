#include "epd_canvas.h"

#include <string.h>

#include "epd_font.h"

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

int canvas_font_char_width(canvas_font_t font)
{
    return font == FONT_8 ? 6 : font / 2;
}

static const uint8_t *glyph_data(char c, canvas_font_t font, int *n_bytes)
{
    if (c < ' ' || c > '~') {
        c = '?';
    }
    int idx = c - ' ';

    switch (font) {
    case FONT_8:
        *n_bytes = 6;
        return asc2_0806[idx];
    case FONT_12:
        *n_bytes = 12;
        return asc2_1206[idx];
    case FONT_16:
        *n_bytes = 16;
        return asc2_1608[idx];
    case FONT_24:
    default:
        *n_bytes = 36;
        return asc2_2412[idx];
    }
}

/* Glyph layout: bytes run column-major inside horizontal 8-row strips,
 * bit 0 = top row of the strip (WeAct example font format). */
static void draw_char(uint8_t *fb, int x, int y, char c,
                      canvas_font_t font, int scale, bool black)
{
    int n_bytes;
    const uint8_t *g = glyph_data(c, font, &n_bytes);

    int width = canvas_font_char_width(font);
    int col = 0;
    int strip = 0;

    for (int i = 0; i < n_bytes; i++) {
        uint8_t bits = g[i];
        for (int m = 0; m < 8; m++) {
            int row = strip * 8 + m;
            if ((bits & 1) && row < font) {
                if (scale == 1) {
                    canvas_set_pixel(fb, x + col, y + row, black);
                } else {
                    canvas_fill_rect(fb, x + col * scale, y + row * scale,
                                     scale, scale, black);
                }
            }
            bits >>= 1;
        }
        col++;
        if (col == width) {
            col = 0;
            strip++;
        }
    }
}

int canvas_text(uint8_t *fb, int x, int y, const char *s,
                canvas_font_t font, int scale, bool black)
{
    int advance = canvas_font_char_width(font) * scale;

    for (; *s != '\0'; s++) {
        draw_char(fb, x, y, *s, font, scale, black);
        x += advance;
    }
    return x;
}

int canvas_text_width(const char *s, canvas_font_t font, int scale)
{
    return (int)strlen(s) * canvas_font_char_width(font) * scale;
}
