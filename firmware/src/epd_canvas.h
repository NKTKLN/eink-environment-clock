/*
 * Minimal 1bpp drawing canvas over the SSD1680 native framebuffer.
 *
 * Canvas coordinates are landscape: x in [0, 296), y in [0, 128),
 * origin at the top-left corner with the flex cable on the right
 * (same orientation as GxEPD2 rotation 3 used by the old sketch).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "epd_ssd1680.h"

#define CANVAS_W EPD_PANEL_H /* 296 */
#define CANVAS_H EPD_PANEL_W /* 128 */

/* Font sizes: height in pixels, width is height/2 (6px for size 8). */
typedef enum {
    FONT_8 = 8,   /* 6 x 8  */
    FONT_12 = 12, /* 6 x 12 */
    FONT_16 = 16, /* 8 x 16 */
    FONT_24 = 24, /* 12 x 24 */
} canvas_font_t;

void canvas_clear(uint8_t *fb, bool white);
void canvas_set_pixel(uint8_t *fb, int x, int y, bool black);
void canvas_fill_rect(uint8_t *fb, int x, int y, int w, int h, bool black);

/* XBM bitmaps (LSB-first rows), as produced for Adafruit drawXBitmap. */
void canvas_draw_xbm(uint8_t *fb, int x, int y, const uint8_t *xbm,
                     int w, int h, bool black);

/* Draws text scaled by an integer factor; only ink pixels are drawn.
 * Returns the x coordinate right after the last glyph. */
int canvas_text(uint8_t *fb, int x, int y, const char *s,
                canvas_font_t font, int scale, bool black);

int canvas_text_width(const char *s, canvas_font_t font, int scale);

int canvas_font_char_width(canvas_font_t font);
