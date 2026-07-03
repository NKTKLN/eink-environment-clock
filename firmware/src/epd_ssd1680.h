/*
 * Driver for the WeAct Studio 2.9" B/W e-paper module
 * (YRD0290BBS800F6HP / DEPG0290BS panel, SSD1680 controller, 128 x 296).
 *
 * Init/update sequences follow the panel vendor reference code shipped in
 * the WeActStudio.EpaperModule repository ("Doc/2.9 Inch Black&Write").
 *
 * Anti-ghosting policy implemented here:
 *  - full refresh writes the image to both B/W RAM (0x24) and the
 *    "previous image" RAM (0x26), so the next differential update has a
 *    correct base;
 *  - partial refresh re-syncs RAM 0x26 with the new image after updating;
 *  - the panel border is driven (0x3C=0x05) on full refresh and left
 *    floating (0x3C=0x80) during partials to avoid border greying;
 *  - the panel is put into deep sleep between refreshes.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define EPD_PANEL_W 128 /* sources (short side) */
#define EPD_PANEL_H 296 /* gates (long side) */

/* Native framebuffer: 1bpp, bit set = white, 16 bytes per gate line. */
#define EPD_FRAMEBUFFER_SIZE (EPD_PANEL_W / 8 * EPD_PANEL_H)

/* Initialize SPI bus and control GPIOs. Does not touch the panel yet. */
esp_err_t epd_init(void);

/* Full (flashing) refresh: crisp image, clears ghosting. ~2 s. */
esp_err_t epd_full_refresh(const uint8_t *fb);

/* Differential (partial) refresh of the whole frame: fast, no flash.
 * Must be preceded by at least one full refresh after power-on. */
esp_err_t epd_partial_refresh(const uint8_t *fb);
