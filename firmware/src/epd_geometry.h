/* Panel geometry shared by the driver and the (host-testable) canvas. */
#pragma once

#define EPD_PANEL_W 128 /* sources (short side) */
#define EPD_PANEL_H 296 /* gates (long side) */

/* Native framebuffer: 1bpp, bit set = white, 16 bytes per gate line. */
#define EPD_FRAMEBUFFER_SIZE (EPD_PANEL_W / 8 * EPD_PANEL_H)
