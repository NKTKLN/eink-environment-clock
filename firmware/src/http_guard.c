#include "http_guard.h"

#include <ctype.h>
#include <stddef.h>
#include <string.h>
#include <strings.h>

static const char *LOCAL_ZONES[] = {
    ".local", ".lan", ".home", ".internal", ".localdomain", ".home.arpa",
};

/* Length of the host part, without ":port" (IPv6 literals keep their
 * brackets). */
static size_t host_len(const char *h)
{
    if (h[0] == '[') {
        const char *end = strchr(h, ']');
        return end != NULL ? (size_t)(end - h + 1) : 0;
    }
    const char *colon = strchr(h, ':');
    return colon != NULL ? (size_t)(colon - h) : strlen(h);
}

static bool is_ipv4(const char *h, size_t n)
{
    int dots = 0, digits = 0;
    for (size_t i = 0; i < n; i++) {
        if (h[i] == '.') {
            if (digits == 0) {
                return false;
            }
            dots++;
            digits = 0;
        } else if (isdigit((unsigned char)h[i]) && digits < 3) {
            digits++;
        } else {
            return false;
        }
    }
    return dots == 3 && digits > 0;
}

static bool ends_with(const char *h, size_t n, const char *suffix)
{
    size_t s = strlen(suffix);
    return n > s && strncasecmp(h + n - s, suffix, s) == 0;
}

bool http_host_allowed(const char *host)
{
    if (host == NULL || host[0] == '\0') {
        return true; /* HTTP/1.0 clients may omit it */
    }
    size_t n = host_len(host);
    if (n == 0 || n > 253) {
        return false;
    }
    if (host[0] == '[') {
        return true; /* IPv6 literal */
    }
    if (is_ipv4(host, n)) {
        return true;
    }
    bool has_dot = false;
    for (size_t i = 0; i < n; i++) {
        char c = host[i];
        if (c == '.') {
            has_dot = true;
        } else if (!isalnum((unsigned char)c) && c != '-') {
            return false;
        }
    }
    if (!has_dot) {
        return true; /* single-label name from the local resolver */
    }
    for (size_t i = 0; i < sizeof(LOCAL_ZONES) / sizeof(LOCAL_ZONES[0]); i++) {
        if (ends_with(host, n, LOCAL_ZONES[i])) {
            return true;
        }
    }
    return false;
}

bool http_origin_allowed(const char *origin, const char *host)
{
    if (origin == NULL || origin[0] == '\0') {
        return true;
    }
    if (host == NULL || host[0] == '\0') {
        return false;
    }
    /* Origin is "scheme://host[:port]" and must name this very host. */
    const char *p = strstr(origin, "://");
    if (p == NULL) {
        return false;
    }
    p += 3;
    return strcasecmp(p, host) == 0;
}
