/*
 * Sensor history for the charts in the web UI: readings are averaged over
 * fixed buckets and kept in a ring of the last 24 hours (pure, host-tested).
 * Lives in RAM, so it starts empty after every restart.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ui.h"

#define HISTORY_INTERVAL_S 120 /* one point every 2 minutes */
#define HISTORY_LEN        720 /* 24 hours */

/* Missing values: the sensor gave nothing during the whole bucket. */
#define HISTORY_NO_TEMP INT16_MIN
#define HISTORY_NO_U16  UINT16_MAX

typedef struct {
    int16_t temp_c10;    /* 0.1 °C */
    uint16_t hum_pct10;  /* 0.1 % */
    uint16_t press_hpa10; /* 0.1 hPa */
    uint16_t co2_ppm;
} history_sample_t;

typedef struct {
    history_sample_t buf[HISTORY_LEN];
    size_t head;  /* index of the next write */
    size_t count; /* stored samples, up to HISTORY_LEN */

    /* bucket being accumulated */
    int64_t bucket_start_ms; /* -1 = no bucket yet */
    int64_t last_push_ms;    /* end of the newest stored bucket */
    double sum[4];
    int n[4];
} history_t;

void history_init(history_t *h);

/* Adds a reading taken at now_ms (monotonic). When the current bucket is
 * HISTORY_INTERVAL_S old its average is stored; buckets that passed with no
 * readings at all are stored as gaps. */
void history_add(history_t *h, const sensor_data_t *d, int64_t now_ms);

/* Copies up to max samples, oldest first. Returns how many. */
size_t history_copy(const history_t *h, history_sample_t *out, size_t max);

/* Seconds since the newest stored sample's bucket ended, -1 if none. */
int history_age_s(const history_t *h, int64_t now_ms);

/* Writes {"interval_s":..,"age_s":..,"temperature_c":[..],"humidity_pct":[..],
 * "pressure_hpa":[..],"co2_ppm":[..]} through emit in small pieces, with
 * null for gaps. Stays small in memory: no cJSON tree for 2880 numbers. */
typedef void (*history_emit_fn)(void *ctx, const char *text, size_t len);
void history_write_json(const history_sample_t *s, size_t n, int age_s,
                        history_emit_fn emit, void *ctx);
