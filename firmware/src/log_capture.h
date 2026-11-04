/* Keeps the last LOG_CAPTURE_SIZE bytes of ESP_LOG output in RAM for the
 * web developer tools. Output still goes to the USB console as before. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define LOG_CAPTURE_SIZE (12 * 1024)

void log_capture_start(void);

/* See log_ring_read(). Thread-safe. */
size_t log_capture_read(uint32_t *pos, char *out, size_t cap, int *dropped);
