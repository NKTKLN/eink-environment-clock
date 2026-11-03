#include "log_capture.h"

#include <stdarg.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#include "log_ring.h"

static char s_buf[LOG_CAPTURE_SIZE];
static log_ring_t s_ring;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev;

static int capture_vprintf(const char *fmt, va_list args)
{
    /* Format once into a small stack buffer (log lines are short); long
     * lines are cut here but still reach the console in full. */
    char line[200];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (n > 0) {
        size_t len = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1;
        portENTER_CRITICAL(&s_mux);
        log_ring_write(&s_ring, line, len);
        portEXIT_CRITICAL(&s_mux);
    }
    return s_prev != NULL ? s_prev(fmt, args) : vprintf(fmt, args);
}

void log_capture_start(void)
{
    log_ring_init(&s_ring, s_buf, sizeof(s_buf));
    s_prev = esp_log_set_vprintf(capture_vprintf);
}

size_t log_capture_read(uint32_t *pos, char *out, size_t cap, int *dropped)
{
    portENTER_CRITICAL(&s_mux);
    size_t n = log_ring_read(&s_ring, pos, out, cap, dropped);
    portEXIT_CRITICAL(&s_mux);
    return n;
}
