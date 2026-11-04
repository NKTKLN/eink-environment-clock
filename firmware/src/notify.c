#include "notify.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Cuts at the last complete UTF-8 character that fits. */
static void copy_utf8(char *dst, size_t size, const char *src, size_t len)
{
    if (len >= size) {
        len = size - 1;
        while (len > 0 && ((unsigned char)src[len] & 0xC0) == 0x80) {
            len--;
        }
    }
    memcpy(dst, src, len);
    dst[len] = '\0';
}

void notify_init(notify_t *n)
{
    memset(n, 0, sizeof(*n));
}

void notify_set(notify_t *n, const char *text, int ttl_s, bool fullscreen,
                int64_t now_ms)
{
    if (text == NULL || text[0] == '\0') {
        notify_clear(n);
        return;
    }
    copy_utf8(n->text, sizeof(n->text), text, strlen(text));
    n->fullscreen = fullscreen;
    if (ttl_s > NOTIFY_TTL_MAX) {
        ttl_s = NOTIFY_TTL_MAX;
    }
    n->expires_ms = ttl_s > 0 ? now_ms + (int64_t)ttl_s * 1000 : 0;
    n->seq++;
}

void notify_clear(notify_t *n)
{
    if (n->text[0] != '\0') {
        n->text[0] = '\0';
        n->fullscreen = false;
        n->expires_ms = 0;
        n->seq++;
    }
}

bool notify_active(notify_t *n, int64_t now_ms)
{
    if (n->text[0] == '\0') {
        return false;
    }
    if (n->expires_ms != 0 && now_ms >= n->expires_ms) {
        notify_clear(n);
        return false;
    }
    return true;
}

int notify_remaining_s(const notify_t *n, int64_t now_ms)
{
    if (n->text[0] == '\0') {
        return 0;
    }
    if (n->expires_ms == 0) {
        return -1;
    }
    int64_t left = n->expires_ms - now_ms;
    return left > 0 ? (int)((left + 999) / 1000) : 0;
}

bool notify_parse(const char *payload, int len, char *text, int text_size,
                  int *ttl_s, bool *fullscreen)
{
    *ttl_s = 0;
    *fullscreen = false;
    text[0] = '\0';

    /* Skip leading whitespace to detect a JSON object. */
    int i = 0;
    while (i < len && (payload[i] == ' ' || payload[i] == '\n' ||
                       payload[i] == '\r' || payload[i] == '\t')) {
        i++;
    }
    if (i >= len || payload[i] != '{') {
        copy_utf8(text, (size_t)text_size, payload, (size_t)len);
        return true;
    }

    cJSON *json = cJSON_ParseWithLength(payload, (size_t)len);
    if (json == NULL) {
        /* Looked like JSON but is not: show it as is. */
        copy_utf8(text, (size_t)text_size, payload, (size_t)len);
        return true;
    }

    bool ok = true;
    const cJSON *t = cJSON_GetObjectItemCaseSensitive(json, "text");
    const cJSON *ttl = cJSON_GetObjectItemCaseSensitive(json, "ttl");
    const cJSON *fs = cJSON_GetObjectItemCaseSensitive(json, "fullscreen");

    if (cJSON_IsString(t)) {
        copy_utf8(text, (size_t)text_size, t->valuestring,
                  strlen(t->valuestring));
    } else if (t != NULL && !cJSON_IsNull(t)) {
        ok = false;
    }
    if (cJSON_IsNumber(ttl)) {
        *ttl_s = ttl->valuedouble > NOTIFY_TTL_MAX ? NOTIFY_TTL_MAX
                                                   : (int)ttl->valuedouble;
    } else if (ttl != NULL && !cJSON_IsNull(ttl)) {
        ok = false;
    }
    if (cJSON_IsBool(fs)) {
        *fullscreen = cJSON_IsTrue(fs);
    } else if (fs != NULL && !cJSON_IsNull(fs)) {
        ok = false;
    }

    cJSON_Delete(json);
    return ok;
}
