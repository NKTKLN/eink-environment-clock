#include "history.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

enum { F_TEMP, F_HUM, F_PRESS, F_CO2 };

static const history_sample_t GAP = {
    HISTORY_NO_TEMP, HISTORY_NO_U16, HISTORY_NO_U16, HISTORY_NO_U16
};

void history_init(history_t *h)
{
    memset(h, 0, sizeof(*h));
    h->bucket_start_ms = -1;
    h->last_push_ms = -1;
}

static void reset_bucket(history_t *h, int64_t start_ms)
{
    h->bucket_start_ms = start_ms;
    memset(h->sum, 0, sizeof(h->sum));
    memset(h->n, 0, sizeof(h->n));
}

static void push(history_t *h, const history_sample_t *s, int64_t end_ms)
{
    h->buf[h->head] = *s;
    h->head = (h->head + 1) % HISTORY_LEN;
    if (h->count < HISTORY_LEN) {
        h->count++;
    }
    h->last_push_ms = end_ms;
}

static history_sample_t bucket_average(const history_t *h)
{
    history_sample_t s = GAP;
    if (h->n[F_TEMP]) {
        s.temp_c10 = (int16_t)lround(h->sum[F_TEMP] / h->n[F_TEMP] * 10.0);
    }
    if (h->n[F_HUM]) {
        s.hum_pct10 = (uint16_t)lround(h->sum[F_HUM] / h->n[F_HUM] * 10.0);
    }
    if (h->n[F_PRESS]) {
        s.press_hpa10 = (uint16_t)lround(h->sum[F_PRESS] / h->n[F_PRESS] * 10.0);
    }
    if (h->n[F_CO2]) {
        s.co2_ppm = (uint16_t)lround(h->sum[F_CO2] / h->n[F_CO2]);
    }
    return s;
}

static void accumulate(history_t *h, int field, double v, double lo, double hi)
{
    if (!isnan(v) && v >= lo && v <= hi) {
        h->sum[field] += v;
        h->n[field]++;
    }
}

void history_add(history_t *h, const sensor_data_t *d, int64_t now_ms)
{
    const int64_t step = HISTORY_INTERVAL_S * 1000LL;
    if (h->bucket_start_ms < 0) {
        reset_bucket(h, now_ms);
    }

    /* Close finished buckets: the first with its average, the rest (the
     * clock was busy, e.g. installing an update) as gaps. */
    if (now_ms - h->bucket_start_ms >= step) {
        history_sample_t avg = bucket_average(h);
        push(h, &avg, h->bucket_start_ms + step);
        int64_t start = h->bucket_start_ms + step;
        for (int i = 0; now_ms - start >= step && i < HISTORY_LEN; i++) {
            push(h, &GAP, start + step);
            start += step;
        }
        if (now_ms - start >= step) { /* longer than the whole ring */
            start = now_ms;
        }
        reset_bucket(h, start);
    }

    /* Ranges keep the values inside the sample's integer types. */
    accumulate(h, F_TEMP, d->temperature_c, -100.0, 100.0);
    accumulate(h, F_HUM, d->humidity_pct, 0.0, 100.0);
    accumulate(h, F_PRESS, d->pressure_hpa, 1.0, 2000.0);
    accumulate(h, F_CO2, d->co2_ppm > 0 ? d->co2_ppm : NAN, 1.0, 20000.0);
}

size_t history_copy(const history_t *h, history_sample_t *out, size_t max)
{
    size_t n = h->count < max ? h->count : max;
    /* the newest n samples */
    size_t start = (h->head + HISTORY_LEN - n) % HISTORY_LEN;
    for (size_t i = 0; i < n; i++) {
        out[i] = h->buf[(start + i) % HISTORY_LEN];
    }
    return n;
}

int history_age_s(const history_t *h, int64_t now_ms)
{
    if (h->count == 0 || h->last_push_ms < 0) {
        return -1;
    }
    return (int)((now_ms - h->last_push_ms) / 1000);
}

static void emit_str(history_emit_fn emit, void *ctx, const char *s)
{
    emit(ctx, s, strlen(s));
}

/* Values are batched into ~512-byte pieces to keep the number of HTTP
 * chunks small. */
typedef struct {
    char buf[560];
    size_t len;
    history_emit_fn emit;
    void *ctx;
} batch_t;

static void batch_add(batch_t *b, const char *s)
{
    size_t n = strlen(s);
    if (b->len + n > sizeof(b->buf)) {
        b->emit(b->ctx, b->buf, b->len);
        b->len = 0;
    }
    memcpy(b->buf + b->len, s, n);
    b->len += n;
}

static void batch_flush(batch_t *b)
{
    if (b->len) {
        b->emit(b->ctx, b->buf, b->len);
        b->len = 0;
    }
}

static void tenths(char *out, size_t cap, int v)
{
    int a = v < 0 ? -v : v;
    snprintf(out, cap, "%s%d.%d", v < 0 ? "-" : "", a / 10, a % 10);
}

void history_write_json(const history_sample_t *s, size_t n, int age_s,
                        history_emit_fn emit, void *ctx)
{
    static const char *KEYS[] = { "temperature_c", "humidity_pct",
                                  "pressure_hpa", "co2_ppm" };
    char head[64];
    snprintf(head, sizeof(head), "{\"interval_s\":%d,\"age_s\":%d",
             HISTORY_INTERVAL_S, age_s);
    emit_str(emit, ctx, head);

    batch_t b = { .len = 0, .emit = emit, .ctx = ctx };
    for (int f = 0; f < 4; f++) {
        char key[32];
        snprintf(key, sizeof(key), ",\"%s\":[", KEYS[f]);
        batch_add(&b, key);
        for (size_t i = 0; i < n; i++) {
            char v[16] = "null";
            switch (f) {
            case F_TEMP:
                if (s[i].temp_c10 != HISTORY_NO_TEMP) tenths(v, sizeof(v), s[i].temp_c10);
                break;
            case F_HUM:
                if (s[i].hum_pct10 != HISTORY_NO_U16) tenths(v, sizeof(v), s[i].hum_pct10);
                break;
            case F_PRESS:
                if (s[i].press_hpa10 != HISTORY_NO_U16) tenths(v, sizeof(v), s[i].press_hpa10);
                break;
            default:
                if (s[i].co2_ppm != HISTORY_NO_U16) snprintf(v, sizeof(v), "%u", s[i].co2_ppm);
                break;
            }
            if (i > 0) {
                batch_add(&b, ",");
            }
            batch_add(&b, v);
        }
        batch_add(&b, "]");
    }
    batch_add(&b, "}");
    batch_flush(&b);
}
