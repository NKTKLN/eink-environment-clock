/*
 * Notification slot with optional expiry. Pure logic (times are passed in),
 * shared by the HTTP API, MQTT and the display loop. Not thread-safe by
 * itself: the app guards it with a mutex.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"

#define NOTIFY_TEXT_MAX 256 /* bytes incl. NUL, matches the HA text limit */
#define NOTIFY_TTL_MAX  (7 * 24 * 3600)

typedef struct {
    char text[NOTIFY_TEXT_MAX];
    bool fullscreen;
    int64_t expires_ms; /* 0 = until cleared */
    uint32_t seq;       /* bumped on every change, used to trigger redraws */
} notify_t;

void notify_init(notify_t *n);

/* Empty text clears. ttl_s <= 0 = no expiry. */
void notify_set(notify_t *n, const char *text, int ttl_s, bool fullscreen,
                int64_t now_ms);
void notify_clear(notify_t *n);

/* Expires the notification if its time is up. True if one is showing. */
bool notify_active(notify_t *n, int64_t now_ms);

/* Seconds left, -1 = no expiry, 0 = none showing. */
int notify_remaining_s(const notify_t *n, int64_t now_ms);

/* Parses {"text":"...","ttl":600,"fullscreen":true}. A payload that is not
 * a JSON object is taken as plain text. Returns false only for invalid
 * field types. */
bool notify_parse(const char *payload, int len, char *text, int text_size,
                  int *ttl_s, bool *fullscreen);
