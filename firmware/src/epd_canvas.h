/*
 * Minimal 1bpp drawing canvas over the SSD1680 native framebuffer.
 *
 * Canvas coordinates are landscape: x in [0, 296), y in [0, 128),
 * origin at the top-left corner with the flex cable on the right
 * (same orientation as GxEPD2 rotation 3 used by the old sketch).
 *
 * Text is UTF-8 and uses the classic Adafruit-GFX 5x7 font with integer
 * scaling plus Cyrillic glyphs drawn in the same style. A glyph cell is
 * 6*scale x 8*scale pixels. Unknown characters render as '?'.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "epd_geometry.h"

#define CANVAS_W EPD_PANEL_H /* 296 */
#define CANVAS_H EPD_PANEL_W /* 128 */

#define CANVAS_CHAR_W 6
#define CANVAS_CHAR_H 8

void canvas_clear(uint8_t *fb, bool white);
void canvas_set_pixel(uint8_t *fb, int x, int y, bool black);
bool canvas_get_pixel(const uint8_t *fb, int x, int y); /* true = black */
void canvas_fill_rect(uint8_t *fb, int x, int y, int w, int h, bool black);
void canvas_draw_rect(uint8_t *fb, int x, int y, int w, int h, bool black);

/* Whole-frame post-processing for the "invert" and "rotate 180" options. */
void canvas_invert(uint8_t *fb);
void canvas_rotate180(uint8_t *fb);

/* XBM bitmaps (LSB-first rows), as produced for Adafruit drawXBitmap. */
void canvas_draw_xbm(uint8_t *fb, int x, int y, const uint8_t *xbm,
                     int w, int h, bool black);

/* Draws text scaled by an integer factor; only ink pixels are drawn.
 * Returns the x coordinate right after the last glyph. */
int canvas_text(uint8_t *fb, int x, int y, const char *s, int scale,
                bool black);

int canvas_text_width(const char *s, int scale);

/* Number of characters (code points) in a UTF-8 string. */
int canvas_strlen(const char *s);

/* Decodes one code point and advances *s. Invalid sequences yield '?'
 * and consume one byte. Returns 0 at the end of the string. */
uint32_t canvas_utf8_next(const char **s);

/* Byte length of the first n code points of s (or strlen if shorter). */
size_t canvas_utf8_prefix(const char *s, int n);
