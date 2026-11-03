/* 1bpp BMP export of the landscape canvas, for the web screen preview. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "epd_canvas.h"

#define BMP_ROW_BYTES ((CANVAS_W + 31) / 32 * 4)
#define BMP_SIZE      (62 + BMP_ROW_BYTES * CANVAS_H)

/* Writes BMP_SIZE bytes into out. */
void bmp_from_canvas(const uint8_t *fb, uint8_t *out);
