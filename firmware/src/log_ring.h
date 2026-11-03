/*
 * Byte ring buffer for log text with a monotonically growing position, so
 * a reader can ask for "everything after position N" (pure, host-tested).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    char *buf;
    size_t size;
    uint32_t total; /* bytes ever written; position of the next byte */
} log_ring_t;

void log_ring_init(log_ring_t *r, char *buf, size_t size);
void log_ring_write(log_ring_t *r, const char *data, size_t len);

/* Copies bytes from position *pos into out (at most cap). If *pos is older
 * than what the ring still holds, reading starts at the oldest byte and
 * *dropped is set. On return *pos is the position after the copied data.
 * Returns the number of bytes copied. */
size_t log_ring_read(const log_ring_t *r, uint32_t *pos, char *out,
                     size_t cap, int *dropped);
