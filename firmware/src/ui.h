#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

typedef struct {
    float temperature_c; /* NAN when unavailable */
    float humidity_pct;  /* NAN when unavailable */
    float pressure_hpa;  /* NAN when unavailable */
    int co2_ppm;         /* -1 when unavailable */
} sensor_data_t;

/* Renders the whole clock face into the framebuffer.
 * time_valid = false renders placeholders instead of time/date. */
void ui_render(uint8_t *fb, const struct tm *now, bool time_valid,
               const sensor_data_t *data);
