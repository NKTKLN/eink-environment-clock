#include "bmp.h"

#include <string.h>

static void le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void le32(uint8_t *p, uint32_t v)
{
    le16(p, (uint16_t)v);
    le16(p + 2, (uint16_t)(v >> 16));
}

void bmp_from_canvas(const uint8_t *fb, uint8_t *out)
{
    memset(out, 0, BMP_SIZE);

    /* BITMAPFILEHEADER */
    out[0] = 'B';
    out[1] = 'M';
    le32(out + 2, BMP_SIZE);
    le32(out + 10, 62);

    /* BITMAPINFOHEADER, negative height = top-down rows */
    le32(out + 14, 40);
    le32(out + 18, CANVAS_W);
    le32(out + 22, (uint32_t)-CANVAS_H);
    le16(out + 26, 1);
    le16(out + 28, 1);
    le32(out + 34, BMP_ROW_BYTES * CANVAS_H);
    le32(out + 38, 2835);
    le32(out + 42, 2835);
    le32(out + 46, 2);

    /* Palette: index 0 = black ink, index 1 = white paper. */
    out[58] = 0xFF;
    out[59] = 0xFF;
    out[60] = 0xFF;

    uint8_t *px = out + 62;
    for (int y = 0; y < CANVAS_H; y++) {
        uint8_t *row = px + y * BMP_ROW_BYTES;
        for (int x = 0; x < CANVAS_W; x++) {
            if (!canvas_get_pixel(fb, x, y)) {
                row[x >> 3] |= (uint8_t)(0x80 >> (x & 7));
            }
        }
    }
}
