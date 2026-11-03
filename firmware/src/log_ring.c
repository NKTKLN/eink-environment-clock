#include "log_ring.h"

#include <string.h>

void log_ring_init(log_ring_t *r, char *buf, size_t size)
{
    r->buf = buf;
    r->size = size;
    r->total = 0;
}

void log_ring_write(log_ring_t *r, const char *data, size_t len)
{
    if (len > r->size) { /* only the tail fits */
        data += len - r->size;
        r->total += (uint32_t)(len - r->size);
        len = r->size;
    }
    size_t at = r->total % r->size;
    size_t first = r->size - at < len ? r->size - at : len;
    memcpy(r->buf + at, data, first);
    memcpy(r->buf, data + first, len - first);
    r->total += (uint32_t)len;
}

size_t log_ring_read(const log_ring_t *r, uint32_t *pos, char *out,
                     size_t cap, int *dropped)
{
    uint32_t oldest = r->total > r->size ? r->total - (uint32_t)r->size : 0;
    *dropped = 0;
    if (*pos > r->total) { /* reader is ahead (device restarted): restart */
        *pos = oldest;
        *dropped = 1;
    }
    if (*pos < oldest) {
        *pos = oldest;
        *dropped = 1;
    }
    size_t avail = r->total - *pos;
    size_t n = avail < cap ? avail : cap;
    for (size_t i = 0; i < n; i++) {
        out[i] = r->buf[(*pos + i) % r->size];
    }
    *pos += (uint32_t)n;
    return n;
}
